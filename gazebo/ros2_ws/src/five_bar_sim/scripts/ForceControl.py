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
from Kinematics import joint_to_world, joint_to_kin, jacobian_world, HOME_WORLD


class PantographForceController(Node):
    """Constant contact force controller for the pantograph."""

    def __init__(self):
        super().__init__('pantograph_force_controller')

        self.pub_left = self.create_publisher(
            Float64, '/model/my_pantograph/joint/left_motor_joint/cmd_force', 10)
        self.pub_right = self.create_publisher(
            Float64, '/model/my_pantograph/joint/right_motor_joint/cmd_force', 10)

        self.create_subscription(
            JointState,
            '/world/five_bar_world/model/my_pantograph/joint_state',
            self._joint_state_cb, 10)

        # Actual contact force from the FT sensor at the end-effector
        self.create_subscription(Wrench, '/ee_ft', self._ft_cb, 10)

        # Desired contact force
        self.F_DES = 0.5      # N, constant force towards the wall (+X). Safe within the 0.1 Nm clamp.
        self.SETTLE_T = 2.0    # s, duration of the settling phase at Home
        self.RAMP_T = 1.0      # s, smooth force ramp 0 -> F_DES

        # Impedance parameters (Stiffness K & Damping D)
        # The Y axis is the one that keeps the trajectory STRAIGHT. It has to overcome the
        # disturbance caused by joint friction: 0.01 Nm / 0.08 m ~ 0.125 N at the EE.
        # With K_y = 50 N/m the deviation was 0.125/50 = 2.5 mm (measured: 2.35 mm).
        # With K_y = 600 N/m it drops to ~0.2 mm, and the integral term removes it.
        self.K_x = 50.0        # N/m (settling phase only)
        self.K_y = 600.0       # N/m (position hold on Y in all phases)
        self.D_x = 25.0        # Ns/m (X damping: gentle approach ~F/D ~ 0.02 m/s & contact stability)
        self.D_y = 14.0        # Ns/m (~critical damping for K_y=600 with m_eff~0.08 kg)

        # Integral term on Y (removes the steady-state deviation)
        self.Ki_y = 400.0      # N/(m*s)
        self.I_Y_MAX = 0.15    # N, anti-windup clamp
        self.i_y = 0.0

        # Targets (Home) and limits
        self.home_x, self.home_y = HOME_WORLD
        self.MAX_TORQUE = 0.1  # Safe torque limit (Nm)

        # Joint Friction/Damping Feedforward
        # A plain tanh(dq/v_eps) goes to zero at standstill, so it NEVER broke the
        # static friction: one joint came unstuck first and the other 520 ms later, so
        # the EE traced an arc around the stuck joint instead of a straight line.
        # Now, at standstill, the compensation follows the direction of the COMMANDED
        # torque, so that both joints break free at the same time (see _friction_ff).
        self.b_visc = 0.01     # damping  -> +b*dq
        self.f_coul = 0.01     # Coulomb joint friction (Nm, as in the SDF)
        self.v_eps  = 0.05     # m/s-equivalent velocity threshold
        self.tau_eps = 0.002   # Nm, sensitivity to the command direction at standstill
        self.k_stiction = 0.9  # under-compensation (<1) so that it does not self-oscillate

        # Velocity filter (EMA)
        self.vel_alpha = 0.96   # same time constant (~50 ms) as 0.8 @ 100 Hz
        self.dq_left_f = 0.0
        self.dq_right_f = 0.0

        # Sensor signal filter (EMA)
        # SENSOR_SIGN: sign alignment so that the contact force (towards the wall) is positive.
        # Confirmed during verification, flip to -1.0 if needed.
        self.SENSOR_SIGN = 1.0
        self.ft_alpha = 0.94    # same time constant (~33 ms) as 0.7 @ 100 Hz
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
        self.dt = 0.002  # 500 Hz - Gazebo's JointStatePublisher gives 1000 Hz,
                         # and the stiff Y axis needs ω_n*dt << 1 for stability.
        self.timer = self.create_timer(self.dt, self._control_loop)

        # Logging
        # Open the file up front so we write live!
        self.log_filename = os.path.join(
            os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
            'force_control_log.csv'
        )
        self.log_file = open(self.log_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow([
            't', 'x_ee', 'y_ee', 'v_x', 'v_y',
            'f_des_x', 'f_des_y', 'f_cmd_x', 'f_cmd_y',
            'f_est_x', 'f_est_y',
            'f_sensor_x', 'f_sensor_y',
            'tau_left', 'tau_right'
        ])
        self.log_counter = 0

        self.get_logger().info(
            f'Force Controller (Approach B: physical wall + FT sensor) initialized.\n'
            f'F_des = {self.F_DES} N (+X) | settle {self.SETTLE_T}s, ramp {self.RAMP_T}s | '
            f'D_x = {self.D_x} Ns/m | MAX_TORQUE = {self.MAX_TORQUE} Nm\n'
            f'Straight trajectory: K_y = {self.K_y} N/m, D_y = {self.D_y} Ns/m, '
            f'Ki_y = {self.Ki_y} N/(m*s) | stiction ff = {self.k_stiction} | '
            f'loop {1.0/self.dt:.0f} Hz\n'
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
        fx = self.SENSOR_SIGN * msg.force.x
        fy = self.SENSOR_SIGN * msg.force.y
        self.f_sensor_x = self.ft_alpha * self.f_sensor_x + (1.0 - self.ft_alpha) * fx
        self.f_sensor_y = self.ft_alpha * self.f_sensor_y + (1.0 - self.ft_alpha) * fy

    def _friction_ff(self, dq, tau_cmd):
        """Joint friction compensation, with a static friction (stiction) part.

        In motion it follows the sign of the velocity (kinetic friction). At standstill,
        where tanh(dq/v_eps) vanishes, it follows the sign of the requested torque, so
        that both joints cross the static friction threshold AT THE SAME INSTANT -
        this is what keeps the trajectory straight.
        """
        s_vel = math.tanh(dq / self.v_eps)          # +/-1 in motion, 0 at standstill
        s_cmd = math.tanh(tau_cmd / self.tau_eps)   # the direction we WANT it to move
        s = s_vel + (1.0 - abs(s_vel)) * s_cmd      # smooth transition between the two
        return self.b_visc * dq + self.f_coul * self.k_stiction * s

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
        #    The Y target is the same straight line (y = home_y = 0) in both phases,
        #    so the integral term is shared and needs no reset.
        err_y = self.home_y - y_ee
        self.i_y = max(-self.I_Y_MAX,
                       min(self.I_Y_MAX, self.i_y + self.Ki_y * err_y * self.dt))

        if t < self.SETTLE_T:
            # Settling phase: position hold at Home (impedance X & Y)
            fx = self.K_x * (self.home_x - x_ee) - self.D_x * v_x
            fy = self.K_y * err_y - self.D_y * v_y + self.i_y
        else:
            # Force phase: X = force mode (constant force + damping), Y = position hold at 0
            fx = f_des_x - self.D_x * v_x
            fy = self.K_y * err_y - self.D_y * v_y + self.i_y + f_des_y

        # 7) Conversion to torques: τ = J_world^T * F + friction/stiction feedforward
        F_vec = np.array([fx, fy])
        tau = J_w.T @ F_vec
        tau_fric_l = self._friction_ff(self.dq_left_f, float(tau[0]))
        tau_fric_r = self._friction_ff(self.dq_right_f, float(tau[1]))
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

        # 11) Data logging (live writing to the CSV)
        self.csv_writer.writerow([
            t, x_ee, y_ee, v_x, v_y,
            f_des_x, f_des_y, fx, fy,
            f_est_x, f_est_y,
            self.f_sensor_x, self.f_sensor_y,
            tau_left, tau_right
        ])
        self.log_counter += 1
        if self.log_counter % int(round(0.1 / self.dt)) == 0:  # flush ~10 Hz
            self.log_file.flush()

        # Print the status every ~1 s
        if self.log_counter % int(round(1.0 / self.dt)) == 0:
            phase = 'SETTLE' if t < self.SETTLE_T else 'FORCE'
            self.get_logger().info(
                f"t={t:4.1f}s [{phase}] | x_ee={x_ee*1000:6.1f}mm | "
                f"F_des={f_des_x:.2f} F_est={f_est_x:.2f} F_sensor={self.f_sensor_x:.2f} N | "
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
