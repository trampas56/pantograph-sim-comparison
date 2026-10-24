import rclpy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Wrench
import math

class PantographTorqueDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.timestep = int(self.__robot.getBasicTimeStep())
        
        # Motors
        self.__left_motor = self.__robot.getDevice('left_motor_joint')
        self.__right_motor = self.__robot.getDevice('right_motor_joint')
        
        # Set motors to torque control mode
        self.__left_motor.setPosition(float('inf'))
        self.__right_motor.setPosition(float('inf'))
        self.__left_motor.setTorque(0.0)
        self.__right_motor.setTorque(0.0)
        
        # Position sensors
        self.__left_sensor = self.__robot.getDevice('left_position_sensor')
        self.__right_sensor = self.__robot.getDevice('right_position_sensor')
        self.__left_sensor.enable(self.timestep)
        self.__right_sensor.enable(self.timestep)
        
        # Force/Torque Sensor (TouchSensor on end-effector).
        # NOTE: in this Webots build (R2025a snap) the force/force-3d TouchSensor
        # returns 0 even on real contact, so /ee_ft is effectively always 0. The
        # contact force is instead derived in ForceControl.py from the force balance.
        self.__ft_sensor = self.__robot.getDevice('ee_ft_sensor')
        if self.__ft_sensor:
            self.__ft_sensor.enable(self.timestep)
        
        # ROS 2 Node
        if not rclpy.ok():
            rclpy.init(args=None)
            
        self.__node = rclpy.create_node('pantograph_torque_driver')
        
        # Publishers & Subscribers
        self.__joint_state_pub = self.__node.create_publisher(JointState, '/joint_states', 10)
        self.__ft_pub = self.__node.create_publisher(Wrench, '/ee_ft', 10)
        
        self.__node.create_subscription(Float64, '/left_motor/cmd_force', self.__left_motor_callback, 10)
        self.__node.create_subscription(Float64, '/right_motor/cmd_force', self.__right_motor_callback, 10)

        # Variables to compute velocity from position delta.
        self.__last_left_pos = 0.0
        self.__last_right_pos = 0.0

    def __left_motor_callback(self, msg):
        self.__left_motor.setTorque(msg.data)

    def __right_motor_callback(self, msg):
        self.__right_motor.setTorque(msg.data)

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
        
        # Publish joint state
        msg = JointState()
        msg.header.stamp = self.__node.get_clock().now().to_msg()
        msg.name = ['left_motor_joint', 'right_motor_joint']
        
        left_pos = self.__left_sensor.getValue()
        right_pos = self.__right_sensor.getValue()
        
        dt = self.timestep / 1000.0
        left_vel = (left_pos - self.__last_left_pos) / dt if dt > 0 else 0.0
        right_vel = (right_pos - self.__last_right_pos) / dt if dt > 0 else 0.0
        
        self.__last_left_pos = left_pos
        self.__last_right_pos = right_pos
        
        msg.position = [left_pos, right_pos]
        msg.velocity = [left_vel, right_vel]
        
        self.__joint_state_pub.publish(msg)
        
        # Publish FT sensor data
        if self.__ft_sensor:
            ft_vals = self.__ft_sensor.getValues()  # returns [Fx, Fy, Fz]
            wrench_msg = Wrench()
            wrench_msg.force.x = float(ft_vals[0])
            wrench_msg.force.y = float(ft_vals[1])
            wrench_msg.force.z = float(ft_vals[2])
            self.__ft_pub.publish(wrench_msg)
