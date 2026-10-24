#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Wrench
import mujoco
import mujoco.viewer
import os
import time
import numpy as np
from ament_index_python.packages import get_package_share_directory

class MuJoCoTorqueNode(Node):
    def __init__(self, model, data):
        super().__init__('mujoco_torque_node')
        self.model = model
        self.data = data

        # Get actuator IDs
        self.left_act_id  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'left_motor')
        self.right_act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'right_motor')

        if self.left_act_id == -1 or self.right_act_id == -1:
            self.get_logger().error("Could not find actuators 'left_motor' / 'right_motor' in model!")

        # Get joint IDs and addresses for mapping state
        self.left_jnt_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'left_motor_joint')
        self.right_jnt_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'right_motor_joint')

        if self.left_jnt_id == -1 or self.right_jnt_id == -1:
            self.get_logger().error("Could not find joints 'left_motor_joint' / 'right_motor_joint' in model!")

        self.left_qpos_adr = model.jnt_qposadr[self.left_jnt_id]
        self.right_qpos_adr = model.jnt_qposadr[self.right_jnt_id]

        self.left_dof_adr = model.jnt_dofadr[self.left_jnt_id]
        self.right_dof_adr = model.jnt_dofadr[self.right_jnt_id]

        # Subscriptions for motor torque commands
        self.create_subscription(Float64, '/left_motor/cmd_torque',  self._cb_left,  10)
        self.create_subscription(Float64, '/right_motor/cmd_torque', self._cb_right, 10)
        # The simulation starts with the first torque command, so the mechanism does not slide while it waits for the controller
        self.cmd_received = False

        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Publisher for force/torque sensor
        self.ft_pub = self.create_publisher(Wrench, '/ee_ft', 10)
        self.ee_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'ee_geom')
        if self.ee_geom_id == -1:
            self.get_logger().warn("Could not find geom 'ee_geom' in model. FT sensor will output 0.")

    def _cb_left(self, msg: Float64):
        self.data.ctrl[self.left_act_id] = msg.data
        self.cmd_received = True

    def _cb_right(self, msg: Float64):
        self.data.ctrl[self.right_act_id] = msg.data
        self.cmd_received = True

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['left_motor_joint', 'right_motor_joint']
        msg.position = [
            float(self.data.qpos[self.left_qpos_adr]),
            float(self.data.qpos[self.right_qpos_adr])
        ]
        msg.velocity = [
            float(self.data.qvel[self.left_dof_adr]),
            float(self.data.qvel[self.right_dof_adr])
        ]
        self.joint_pub.publish(msg)

    def publish_ft(self):
        msg = Wrench()
        if self.ee_geom_id == -1:
            self.ft_pub.publish(msg)
            return

        total_force = np.zeros(3)
        total_torque = np.zeros(3)

        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            if contact.geom1 == self.ee_geom_id or contact.geom2 == self.ee_geom_id:
                c_force = np.zeros(6)
                mujoco.mj_contactForce(self.model, self.data, i, c_force)
                
                # Contact frame is 3x3 rotation matrix (flattened)
                mat = np.array(contact.frame).reshape(3, 3)
                f_world = mat.T @ c_force[:3]
                t_world = mat.T @ c_force[3:]
                
                # force is applied by geom1 on geom2
                if contact.geom1 == self.ee_geom_id:
                    total_force -= f_world
                    total_torque -= t_world
                else:
                    total_force += f_world
                    total_torque += t_world

        msg.force.x = float(total_force[0])
        msg.force.y = float(total_force[1])
        msg.force.z = float(total_force[2])
        msg.torque.x = float(total_torque[0])
        msg.torque.y = float(total_torque[1])
        msg.torque.z = float(total_torque[2])
        self.ft_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    # Resolve model path via ament_index
    pkg_share = get_package_share_directory('pantograph_bot')
    model_path = os.path.join(pkg_share, 'models', 'pantograph_torque.xml')

    model = mujoco.MjModel.from_xml_path(model_path)
    data  = mujoco.MjData(model)

    node = MuJoCoTorqueNode(model, data)

    node.get_logger().info(f"Loaded model from: {model_path}")
    node.get_logger().info("Launching MuJoCo passive viewer for Torque Control...")
    node.get_logger().info("Waiting for the first torque command (simulation paused)...")

    # Body positions for the viewer, without advancing time
    mujoco.mj_forward(model, data)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running() and rclpy.ok():
            start_time = time.time()

            # Process ROS callbacks (non-blocking). The controller sends two commands per cycle
            # (one per motor) and spin_once runs one, so this runs all that are waiting.
            for _ in range(50):
                rclpy.spin_once(node, timeout_sec=0.0)

            # Publish feedback at 100 Hz
            node.publish_joint_states()
            node.publish_ft()

            # Step the simulation dynamically to match the rate (100 Hz)
            if node.cmd_received:
                step_start = data.time
                while data.time - step_start < 0.01:
                    mujoco.mj_step(model, data)

            # Sync the viewer
            viewer.sync()

            # Sleep to maintain 100 Hz rate (10ms loop period)
            elapsed = time.time() - start_time
            sleep_time = 0.01 - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
