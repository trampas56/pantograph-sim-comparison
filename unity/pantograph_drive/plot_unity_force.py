#!/usr/bin/env python3
"""
plot_unity_force.py - Graphical comparison of forces & position from unity_force_log.csv
(Unity five-bar, Phase 2b: force control with J^T). Counterpart of the Gazebo plot_force_results.py.

  (top)     Force at the EE: expected (F_cmd, command = (J^T)^-1 * τ)
            vs actual from the sensor (F_sensor = ContactForceProbe).
  (bottom)  End-effector position (reach) in mm, with the contact & wall face lines,
            so that the moment of impact and the standstill are visible.
"""
import os
import csv
import matplotlib
matplotlib.use('Agg')  # headless-safe (PNG without a display)
import matplotlib.pyplot as plt

# Wall geometry (from PantographWall.cs) - tip sphere r=0.015
EE_RADIUS_MM       = 15.0
WALL_SURFACE_REACH = 0.140    # contact surface (reach offset from the EE home)
EE_HOME_REACH      = 0.0898   # EE reach at home (SDF)

# Relative to the script location (unity/pantograph_drive/) - Unity writes the CSV
# to PantographSim/ (Application.dataPath/..), so this works on any machine/user.
SIM_DIR  = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'PantographSim')
CSV_PATH = os.path.abspath(os.path.join(SIM_DIR, 'unity_force_log.csv'))
OUT_PNG  = os.path.abspath(os.path.join(SIM_DIR, 'unity_force_plot.png'))


def main():
    if not os.path.exists(CSV_PATH):
        print(f"Error: {CSV_PATH} not found - run Play with PantographForceControl first.")
        return
    print(f"Reading log data from: {CSV_PATH}")

    t, reach, f_cmd, f_est, f_sensor = [], [], [], [], []
    with open(CSV_PATH, 'r') as f:
        for row in csv.DictReader(f):
            t.append(float(row['t']))
            reach.append(float(row['ee_reach']) * 1000.0)   # mm
            f_cmd.append(float(row['F_cmd_N']))
            f_est.append(float(row.get('F_est_N', 0.0) or 0.0))   # (J^T)^-1 * τ - shows the saturation
            f_sensor.append(float(row['contactF_N']))
    has_est = any(abs(v) > 1e-9 for v in f_est)   # old CSVs without the column -> no blue line

    if not t:
        print("Error: the CSV has no data.")
        return

    # Reference lines in the controller's reach frame (home = first value)
    home_mm      = reach[0]
    wall_face_mm = home_mm + (WALL_SURFACE_REACH - EE_HOME_REACH) * 1000.0
    contact_mm   = wall_face_mm - EE_RADIUS_MM     # this is where the sphere centre comes to rest

    duration = t[-1] - t[0] if len(t) > 1 else 10
    fig_width = max(11.0, duration / 5.0)
    fig, (ax_f, ax_p) = plt.subplots(2, 1, figsize=(fig_width, 8), sharex=True)
    fig.suptitle('Five-Bar Pantograph (Unity) - Contact Force Control',
                 fontsize=15, fontweight='bold')

    # Top: forces at the EE (3 curves, as in Gazebo)
    ax_f.plot(t, f_cmd, color='#2f3542', linestyle='--', linewidth=2.2,
              label='Desired  F_des (command)')
    if has_est:
        ax_f.plot(t, f_est, color='#1e90ff', linestyle='-', linewidth=2.0,
                  label='Estimated  F_est = (J^T)^-1 * τ  (after the torque limit)')
    ax_f.plot(t, f_sensor, color='#ff4757', linestyle='-', linewidth=2.0,
              label='Actual  F_sensor (FT sensor)')
    ax_f.set_title('Force at the end-effector (towards the wall)', fontsize=12, fontweight='semibold')
    ax_f.set_ylabel('Force (N)', fontsize=11)
    ax_f.grid(True, linestyle=':', alpha=0.6)
    ax_f.legend(loc='best', frameon=True, framealpha=0.9)
    fmax = max(max(f_cmd), max(f_est), max(f_sensor), 1.0)
    ax_f.set_ylim(-0.15, fmax * 1.25)
    # final sensor value as an annotation
    ax_f.annotate(f'{f_sensor[-1]:.2f} N', xy=(t[-1], f_sensor[-1]),
                  xytext=(-60, 14), textcoords='offset points',
                  color='#ff4757', fontweight='bold',
                  arrowprops=dict(arrowstyle='->', color='#ff4757'))

    # Bottom: position (reach)
    ax_p.plot(t, reach, color='#2ed573', linewidth=2.2, label='EE reach (sphere centre)')
    ax_p.axhline(contact_mm, color='#ff6b81', linestyle='--', linewidth=1.5,
                 label=f'Contact ({contact_mm:.0f} mm)')
    ax_p.axhline(wall_face_mm, color='#7f8fa6', linestyle=':', linewidth=1.5,
                 label=f'Wall face ({wall_face_mm:.0f} mm)')
    ax_p.set_title('End-effector position (reach)', fontsize=12, fontweight='semibold')
    ax_p.set_xlabel('Time (s)', fontsize=11)
    ax_p.set_ylabel('reach (mm)', fontsize=11)
    ax_p.grid(True, linestyle=':', alpha=0.6)
    ax_p.legend(loc='best', frameon=True, framealpha=0.9)

    plt.tight_layout(rect=(0, 0, 1, 0.97))
    plt.savefig(OUT_PNG, dpi=150)
    print(f"Plot saved to: {OUT_PNG}")
    print(f"Final: F_cmd={f_cmd[-1]:.3f} N | F_sensor={f_sensor[-1]:.3f} N | "
          f"reach={reach[-1]:.1f} mm (contact at {contact_mm:.1f} mm)")


if __name__ == '__main__':
    main()
