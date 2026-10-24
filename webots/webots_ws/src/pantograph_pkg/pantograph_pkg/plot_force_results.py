#!/usr/bin/env python3
"""
plot_force_results.py - Graphical comparison of the forces in force_control_log.csv.

Produces a figure with:
  (top)     Contact force on the wall along X. With SHOW_DES_EST = True the desired
            (F_des) and the estimate from the equations (F_est = (J^T)^-1 * τ) are added.
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

# Desired (F_des) and estimated (F_est) in the top plot, False = contact force only
SHOW_DES_EST = False


def ema_smooth(y, alpha=0.9):
    """Low-pass EMA filter for DISPLAY ONLY. With sampling ~0.01 s and
    alpha=0.9 the time constant is ~0.1 s -> the line rises in ~0.25 s instead of instantly.
    It changes neither the control nor the CSV data (applied only to the plot)."""
    out = []
    s = 0.0
    for v in y:
        s = alpha * s + (1.0 - alpha) * v
        out.append(s)
    return out


def find_csv():
    # realpath: follows the install symlink -> same folder as ForceControl.py (source package).
    current_dir = os.path.dirname(os.path.realpath(__file__))
    candidates = [
        os.path.join(current_dir, 'webots_force_log.csv'),
        os.path.join(os.path.dirname(current_dir), 'webots_force_log.csv'),
        os.path.join(os.path.dirname(os.path.dirname(current_dir)), 'webots_force_log.csv'),
    ]
    for p in candidates:
        if os.path.exists(p):
            return p
    print("Error: webots_force_log.csv not found. Searched:")
    for p in candidates:
        print(f"  - {p}")
    return None

def main():
    csv_path = find_csv()
    if not csv_path:
        return
    print(f"Reading log data from: {csv_path}")

    t, x_ee = [], []
    f_des, f_est, f_contact = [], [], []

    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            t.append(float(row['t']))
            x_ee.append(float(row['x_ee']) * 1000.0)  # mm
            f_des.append(float(row.get('f_des_x', 0.0) or 0.0))
            f_est.append(float(row.get('f_est_x', 0.0) or 0.0))
            # Actual contact force (from the balance, active only during contact).
            # New column f_contact_x, falls back to the old f_sensor_x for old CSVs.
            f_contact.append(float(row.get('f_contact_x', row.get('f_sensor_x', 0.0)) or 0.0))

    if not t:
        print("Error: CSV has no data rows.")
        return

    # Smoothing for DISPLAY ONLY (the CSV stays untouched): the red line
    # rises smoothly instead of instantly at the moment of contact. Larger alpha = smoother
    # (alpha=0.97 -> rise ~0.7 s, it also rounds off the contact bounce).
    f_contact = ema_smooth(f_contact, alpha=0.97)

    duration = t[-1] - t[0] if len(t) > 1 else 10
    # Widen the figure with the duration, so the data is not "squeezed" if it runs for 5 minutes
    fig_width = max(11.0, duration / 5.0) 
    fig, (ax_f, ax_p) = plt.subplots(2, 1, figsize=(fig_width, 8), sharex=True)
    fig.suptitle('Five-Bar Pantograph - Contact Force Control',
                 fontsize=15, fontweight='bold')

    # Top: forces along X
    shown = [f_contact]  # the lines that are shown set the y-axis limits
    if SHOW_DES_EST:
        ax_f.plot(t, f_des, color='#2f3542', linestyle='--', linewidth=2.2,
                  label='Desired F_des')
        ax_f.plot(t, f_est, color='#1e90ff', linestyle='-', linewidth=2.0,
                  label='Estimated F_est  =(J^T)^-1 * τ')
        shown += [f_des, f_est]
    ax_f.plot(t, f_contact, color='#ff4757', linestyle='-', linewidth=2.4,
              label='Contact force on the wall')
    ax_f.set_title('Force along X at the end-effector', fontsize=12, fontweight='semibold')
    ax_f.set_ylabel('Force (N)', fontsize=11)
    ax_f.grid(True, linestyle=':', alpha=0.6)
    ax_f.legend(loc='best', frameon=True, framealpha=0.9)
    fmax = max([max(y) for y in shown] + [1.0])
    fmin = min([min(y) for y in shown] + [0.0])
    ax_f.set_ylim(fmin - 0.2, fmax * 1.25)

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

    out_primary = os.path.join(os.path.dirname(csv_path), 'webots_force_control_plot.png')
    plt.savefig(out_primary, dpi=150)
    print(f"Plot saved to: {out_primary}")


if __name__ == '__main__':
    main()
