#!/usr/bin/env python3
# ForceControl.py - Constant contact force control (Approach B: physical wall + FT sensor).
#
# Experiment flow:
#   1) Settling phase (t < SETTLE_T): position hold at Home (impedance X & Y).
#   2) Force phase (t >= SETTLE_T): a CONSTANT force is applied along +X (towards the wall),
#      with a smooth ramp. The X axis is in "force mode" (no stiffness) so that the force
#      drives the end-effector towards the wall, and the PHYSICAL wall (collision in Gazebo) stops it.
#      The Y axis stays in position hold (y = 0).
#
# Three forces are compared (all logged to the CSV):
#   * F_des    : desired force (setpoint, with ramp).
#   * F_est    : estimate from the equations -> F_est = (J_world^T)^-1 * (τ_cmd - τ_friction).
#   * F_sensor : actual force from the Force/Torque sensor at the end-effector (topic /ee_ft).

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Wrench
import math
import os
import sys
import time
import csv
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pantograph_pkg.Kinematics import joint_to_world, joint_to_kin, jacobian_world, HOME_WORLD


class PantographForceController(Node):
    """Constant contact force controller for the pantograph."""

    def __init__(self):
        super().__init__('pantograph_force_controller')

        self.pub_left = self.create_publisher(
            Float64, '/left_motor/cmd_force', 10)
        self.pub_right = self.create_publisher(
            Float64, '/right_motor/cmd_force', 10)

        self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_cb, 10)

        # Actual contact force from the FT sensor at the end-effector
        self.create_subscription(Wrench, '/ee_ft', self._ft_cb, 10)

        # Desired contact force
        self.F_DES = 2.0      # N, constant force towards the wall (+X). Safe within the 0.1 Nm clamp.
        self.SETTLE_T = 2.0    # s, duration of the settling phase at Home
        self.RAMP_T = 1.0      # s, smooth force ramp 0 -> F_DES

        # Impedance parameters (Stiffness K & Damping D)
        self.K_x = 50.0        # N/m (settling phase only)
        self.K_y = 50.0        # N/m (position hold on Y in all phases)
        self.D_x = 25.0        # Ns/m (X damping: gentle approach ~F/D ~ 0.04 m/s & contact stability)
        self.D_y = 5.0         # Ns/m

        # Targets (Home) and limits
        self.home_x, self.home_y = HOME_WORLD
        self.MAX_TORQUE = 0.1  # Safe torque limit (Nm)

        # Wall geometry (for contact detection) - MUST match the .wbt
        # The wall is a Box at x=0.145 with half-width 0.025 -> front face at x=0.120.
        # The end-effector sphere has radius 0.015. Contact when x_ee + r >= wall face.
        self.WALL_FACE_X = 0.120
        self.EE_RADIUS   = 0.015
        self.CONTACT_X   = self.WALL_FACE_X - self.EE_RADIUS  # 0.105 m (EE centre at contact)

        # Joint Friction/Damping Feedforward
        self.b_visc = 0.01   # damping  -> +b*dq
        self.f_coul = 0.01   # friction -> +f*tanh(dq/v_eps)
        self.v_eps  = 0.05

        # Velocity filter (EMA)
        self.vel_alpha = 0.8
        self.dq_left_f = 0.0
        self.dq_right_f = 0.0

        # Sensor signal filter (EMA)
        # SENSOR_SIGN: sign alignment so that the contact force (towards the wall) is positive.
        # Confirmed during verification, flip to -1.0 if needed.
        self.SENSOR_SIGN = 1.0
        self.ft_alpha = 0.7
        self.f_sensor_x = 0.0
        self.f_sensor_y = 0.0

        # Robot state
        self.q_left = 0.0
        self.q_right = 0.0
        self.dq_left = 0.0
        self.dq_right = 0.0
        self.received_feedback = False

        # Timing
        self.start_time = time.time()
        self.t0 = None
        self.dt = 0.01  # 100 Hz
        self.timer = self.create_timer(self.dt, self._control_loop)

        # Logging
        # Open the file up front so we write live!
        self.log_filename = os.path.join(
            os.path.expanduser('~'), 'Desktop', 'pantograph', 'webots',
            'webots_force_log.csv'
        )
        self.log_file = open(self.log_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow([
            't', 'x_ee', 'y_ee', 'v_x', 'v_y',
            'f_des_x', 'f_des_y', 'f_cmd_x', 'f_cmd_y',
            'f_est_x', 'f_est_y',
            'f_sensor_x', 'f_sensor_y',
            'f_contact_x', 'f_contact_y',
            'tau_left', 'tau_right'
        ])
        self.log_counter = 0

        self.get_logger().info(
            f'Force Controller (Approach B: physical wall + FT sensor) initialized.\n'
            f'F_des = {self.F_DES} N (+X) | settle {self.SETTLE_T}s, ramp {self.RAMP_T}s | '
            f'D_x = {self.D_x} Ns/m | MAX_TORQUE = {self.MAX_TORQUE} Nm\n'
            f'Home: ({self.home_x:.4f}, {self.home_y:.4f}) | listening /ee_ft for real contact force'
        )

    def _joint_state_cb(self, msg: JointState):
        self.received_feedback = True
        for i, name in enumerate(msg.name):
            if name == 'left_motor_joint':
                self.q_left = msg.position[i] if msg.position else 0.0
                self.dq_left = msg.velocity[i] if msg.velocity else 0.0
            elif name == 'right_motor_joint':
                self.q_right = msg.position[i] if msg.position else 0.0
                self.dq_right = msg.velocity[i] if msg.velocity else 0.0

    def _ft_cb(self, msg: Wrench):
        """Actual contact force (EMA-filtered for the plot)."""
        # Webots getForceFeedback() returns NaN on the first step! We have to ignore it.
        if math.isnan(msg.force.x) or math.isnan(msg.force.y):
            return
        # The Webots TouchSensor returns values in its local frame, which rotates.
        # Since the wall is perpendicular to the X axis, the total contact force is practically the
        # force along X (after projection). Its magnitude (in the XY plane) equals the X force.
        f_mag = math.sqrt(msg.force.x**2 + msg.force.y**2)
        fx = self.SENSOR_SIGN * f_mag
        fy = 0.0
        self.f_sensor_x = self.ft_alpha * self.f_sensor_x + (1.0 - self.ft_alpha) * fx
        self.f_sensor_y = self.ft_alpha * self.f_sensor_y + (1.0 - self.ft_alpha) * fy

    def _get_desired_force(self, t):
        """Desired force F_des = [Fx, Fy] (N). Constant +X with a smooth ramp after settling."""
        if t < self.SETTLE_T:
            return 0.0, 0.0
        ramp = min(1.0, (t - self.SETTLE_T) / self.RAMP_T)
        return self.F_DES * ramp, 0.0

    def _control_loop(self):
        if not self.received_feedback:
            if 1.0 < (time.time() - self.start_time) < 2.0:
                self.get_logger().warn('Waiting for joint states from Gazebo...')
            return

        # Elapsed time
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0

        # 1) Current end-effector position in the World frame (FK)
        x_ee, y_ee = joint_to_world(self.q_left, self.q_right)

        # 2) Velocity filtering (EMA)
        self.dq_left_f = self.vel_alpha * self.dq_left_f + (1.0 - self.vel_alpha) * self.dq_left
        self.dq_right_f = self.vel_alpha * self.dq_right_f + (1.0 - self.vel_alpha) * self.dq_right

        # 3) Jacobian in the World frame
        th1, th2 = joint_to_kin(self.q_left, self.q_right)
        J_w = jacobian_world(th1, th2)

        # 4) End-effector velocity in World: v_ee = J_world * dq
        dq_vec = np.array([self.dq_left_f, self.dq_right_f])
        v_ee = J_w @ dq_vec
        v_x, v_y = float(v_ee[0]), float(v_ee[1])

        # 5) Desired force (with ramp)
        f_des_x, f_des_y = self._get_desired_force(t)

        # 6) Cartesian control law per phase
        if t < self.SETTLE_T:
            # Settling phase: position hold at Home (impedance X & Y)
            fx = self.K_x * (self.home_x - x_ee) - self.D_x * v_x
            fy = self.K_y * (self.home_y - y_ee) - self.D_y * v_y
        else:
            # Force phase: X = force mode (constant force + damping), Y = position hold at 0
            fx = f_des_x - self.D_x * v_x
            fy = self.K_y * (0.0 - y_ee) - self.D_y * v_y + f_des_y

        # 7) Conversion to torques: τ = J_world^T * F + friction feedforward
        F_vec = np.array([fx, fy])
        tau = J_w.T @ F_vec
        tau_fric_l = self.b_visc * self.dq_left_f + self.f_coul * math.tanh(self.dq_left_f / self.v_eps)
        tau_fric_r = self.b_visc * self.dq_right_f + self.f_coul * math.tanh(self.dq_right_f / self.v_eps)
        tau_left = float(tau[0]) + tau_fric_l
        tau_right = float(tau[1]) + tau_fric_r

        # 8) Limit (clamp)
        tau_left = max(-self.MAX_TORQUE, min(self.MAX_TORQUE, tau_left))
        tau_right = max(-self.MAX_TORQUE, min(self.MAX_TORQUE, tau_right))

        # 9) Send to Gazebo
        msg_l = Float64(); msg_l.data = tau_left
        msg_r = Float64(); msg_r.data = tau_right
        self.pub_left.publish(msg_l)
        self.pub_right.publish(msg_r)

        # 10) Force estimated from the equations: F_est = (J_world^T)^-1 (τ_cmd - τ_friction)
        #     We subtract the friction feedforward so that F_est reflects the force at the EE.
        tau_for_est = np.array([tau_left - tau_fric_l, tau_right - tau_fric_r])
        try:
            F_est = np.linalg.solve(J_w.T, tau_for_est)
            f_est_x, f_est_y = float(F_est[0]), float(F_est[1])
        except np.linalg.LinAlgError:
            f_est_x, f_est_y = 0.0, 0.0

        # 10b) ACTUAL CONTACT FORCE.
        #   The Webots force/force-3d TouchSensor returns 0 in this version (R2025a snap)
        #   even when there is contact (confirmed: the contact is detected, but the force is not).
        #   Since the EE is in static equilibrium at contact, the force the EE applies
        #   (= F_est from the torque balance) equals the reaction of the wall. So the
        #   actual contact force = F_est, active ONLY while the sphere touches the wall.
        in_contact = (x_ee + self.EE_RADIUS) >= (self.WALL_FACE_X - 0.0005)
        f_contact_x = max(0.0, f_est_x) if in_contact else 0.0
        f_contact_y = 0.0

        # 11) Data logging (live writing to the CSV)
        self.csv_writer.writerow([
            t, x_ee, y_ee, v_x, v_y,
            f_des_x, f_des_y, fx, fy,
            f_est_x, f_est_y,
            self.f_sensor_x, self.f_sensor_y,
            f_contact_x, f_contact_y,
            tau_left, tau_right
        ])
        self.log_counter += 1
        if self.log_counter % 10 == 0:  # Flush to disk every 10 steps (10 Hz)
            self.log_file.flush()

        # Print the status every ~1 s
        if int(t * 100) % 100 == 0:
            phase = 'SETTLE' if t < self.SETTLE_T else 'FORCE'
            contact_tag = 'CONTACT' if in_contact else 'free   '
            self.get_logger().info(
                f"t={t:4.1f}s [{phase}] | x_ee={x_ee*1000:6.1f}mm [{contact_tag}] | "
                f"F_des={f_des_x:.2f} F_est={f_est_x:.2f} F_contact={f_contact_x:.2f} N | "
                f"tau=({tau_left:+.4f}, {tau_right:+.4f}) Nm"
            )

    def save_log(self):
        # Just close the file when the node shuts down
        if hasattr(self, 'log_file') and not self.log_file.closed:
            self.log_file.flush()
            self.log_file.close()
            self.get_logger().info("Log file closed successfully.")


def main(args=None):
    rclpy.init(args=args)
    node = PantographForceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node...")
    finally:
        node.save_log()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
