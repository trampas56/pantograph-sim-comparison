#!/usr/bin/env python3
"""
plot_force_results.py - Graphical comparison of the forces in force_control_log.csv.

Produces a figure with:
  (top)     Actual force along X from the FT sensor (F_sensor). The requested force
            (F_des) is drawn only as a horizontal reference line - it is a constant setpoint,
            not a measured quantity.
  (bottom)  End-effector x position (mm) with the contact (105 mm) and wall face (120 mm) lines,
            so that the moment of impact and the standstill are visible.
"""
import os
import csv
import matplotlib
matplotlib.use('Agg')  # headless-safe (save the PNG without a display)
import matplotlib.pyplot as plt

# Geometry (m): tip sphere r=0.015, front face of the wall at x=0.120
EE_RADIUS_MM = 15.0
WALL_FACE_MM = 120.0
CONTACT_X_MM = WALL_FACE_MM - EE_RADIUS_MM  # 105 mm


def find_csv():
    # realpath: follows the install symlink -> same folder as ForceControl.py (source package).
    current_dir = os.path.dirname(os.path.realpath(__file__))
    candidates = [
        os.path.join(current_dir, 'force_control_log.csv'),
        os.path.join(os.path.dirname(current_dir), 'force_control_log.csv'),
        os.path.join(os.path.dirname(os.path.dirname(current_dir)), 'force_control_log.csv'),
    ]
    for p in candidates:
        if os.path.exists(p):
            return p
    print("Error: force_control_log.csv not found. Searched:")
    for p in candidates:
        print(f"  - {p}")
    return None


def main():
    csv_path = find_csv()
    if not csv_path:
        return
    print(f"Reading log data from: {csv_path}")

    t, x_ee = [], []
    f_des, f_sensor = [], []

    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            t.append(float(row['t']))
            x_ee.append(float(row['x_ee']) * 1000.0)  # mm
            f_des.append(float(row.get('f_des_x', 0.0) or 0.0))
            f_sensor.append(float(row.get('f_sensor_x', 0.0) or 0.0))

    if not t:
        print("Error: CSV has no data rows.")
        return

    duration = t[-1] - t[0] if len(t) > 1 else 10
    # Widen the figure with the duration, so the data is not "squeezed" if it runs for 5 minutes
    fig_width = max(11.0, duration / 5.0) 
    fig, (ax_f, ax_p) = plt.subplots(2, 1, figsize=(fig_width, 8), sharex=True)
    fig.suptitle('Five-Bar Pantograph - Contact Force Control',
                 fontsize=15, fontweight='bold')

    # Top: actual force along X (sensor only)
    # F_des ramps 0 -> setpoint, so the final value is the max.
    f_setpoint = max(f_des) if f_des else 0.0
    ax_f.axhline(f_setpoint, color='#2f3542', linestyle='--', linewidth=1.5,
                 label=f'Requested F_des ({f_setpoint:.1f} N)')
    ax_f.plot(t, f_sensor, color='#ff4757', linestyle='-', linewidth=2.2,
              label='Actual F_sensor (FT sensor)')
    ax_f.set_title('Force along X at the end-effector', fontsize=12, fontweight='semibold')
    ax_f.set_ylabel('Force (N)', fontsize=11)
    ax_f.grid(True, linestyle=':', alpha=0.6)
    ax_f.legend(loc='best', frameon=True, framealpha=0.9)
    fmax = max(max(f_sensor), f_setpoint, 0.1)
    fmin = min(min(f_sensor), 0.0)
    ax_f.set_ylim(fmin - 0.05 * fmax, fmax * 1.25)

    # Bottom: position x_ee
    ax_p.plot(t, x_ee, color='#2ed573', linewidth=2.2, label='x_ee (centre)')
    ax_p.axhline(CONTACT_X_MM, color='#ff6b81', linestyle='--', linewidth=1.5,
                 label=f'Contact ({CONTACT_X_MM:.0f} mm)')
    ax_p.axhline(WALL_FACE_MM, color='#7f8fa6', linestyle=':', linewidth=1.5,
                 label=f'Wall face ({WALL_FACE_MM:.0f} mm)')
    ax_p.set_title('X position of the end-effector', fontsize=12, fontweight='semibold')
    ax_p.set_xlabel('Time (s)', fontsize=11)
    ax_p.set_ylabel('x (mm)', fontsize=11)
    ax_p.grid(True, linestyle=':', alpha=0.6)
    ax_p.legend(loc='best', frameon=True, framealpha=0.9)

    plt.tight_layout(rect=(0, 0, 1, 0.97))

    out_primary = os.path.join(os.path.dirname(csv_path), 'force_control_plot.png')
    plt.savefig(out_primary, dpi=150)
    print(f"Plot saved to: {out_primary}")

    pkg_dir = "/home/leuteris/Desktop/pantograph/gazebo /ros2_ws/src/five_bar_sim"
    if os.path.isdir(pkg_dir):
        out_pkg = os.path.join(pkg_dir, 'force_control_plot.png')
        if os.path.abspath(out_pkg) != os.path.abspath(out_primary):
            plt.savefig(out_pkg, dpi=150)
            print(f"Plot also saved to: {out_pkg}")


if __name__ == '__main__':
    main()
