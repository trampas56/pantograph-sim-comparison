#!/usr/bin/env python3
"""
plot_force_results.py - Γραφική σύγκριση δυνάμεων από το force_control_log.csv.

Παράγει διάγραμμα με:
  (πάνω)  Πραγματική δύναμη κατά X από τον FT αισθητήρα (F_sensor). Η ζητούμενη δύναμη
          (F_des) σχεδιάζεται μόνο ως οριζόντια γραμμή αναφοράς - είναι σταθερό setpoint,
          όχι μετρούμενο μέγεθος.
  (κάτω)  Θέση x του end-effector (mm) με γραμμές επαφής (105 mm) και όψης τοίχου (120 mm),
          ώστε να φαίνεται η στιγμή πρόσκρουσης και η ακινητοποίηση.
"""
import os
import csv
import matplotlib
matplotlib.use('Agg')  # headless-safe (αποθήκευση PNG χωρίς display)
import matplotlib.pyplot as plt

# Γεωμετρία (m): η σφαίρα tip r=0.015, μπροστινή όψη τοίχου στο x=0.120
EE_RADIUS_MM = 15.0
WALL_FACE_MM = 120.0
CONTACT_X_MM = WALL_FACE_MM - EE_RADIUS_MM  # 105 mm


def find_csv():
    # realpath: ακολουθεί το symlink του install -> ίδιος φάκελος με το ForceControl.py (πηγαίο πακέτο).
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
    # Αυξάνουμε το πλάτος της εικόνας ανάλογα με τη διάρκεια, ώστε να μη "ζουλιούνται" τα δεδομένα αν τρέξει για 5 λεπτά
    fig_width = max(11.0, duration / 5.0) 
    fig, (ax_f, ax_p) = plt.subplots(2, 1, figsize=(fig_width, 8), sharex=True)
    fig.suptitle('Five-Bar Pantograph - Έλεγχος Δύναμης Επαφής',
                 fontsize=15, fontweight='bold')

    # Πάνω: Πραγματική δύναμη κατά X (μόνο ο αισθητήρας)
    # Το F_des έχει ράμπα 0 -> setpoint, η τελική τιμή είναι το max.
    f_setpoint = max(f_des) if f_des else 0.0
    ax_f.axhline(f_setpoint, color='#2f3542', linestyle='--', linewidth=1.5,
                 label=f'Ζητούμενη F_des ({f_setpoint:.1f} N)')
    ax_f.plot(t, f_sensor, color='#ff4757', linestyle='-', linewidth=2.2,
              label='Πραγματική F_sensor (FT αισθητήρας)')
    ax_f.set_title('Δύναμη κατά X στο end-effector', fontsize=12, fontweight='semibold')
    ax_f.set_ylabel('Δύναμη (N)', fontsize=11)
    ax_f.grid(True, linestyle=':', alpha=0.6)
    ax_f.legend(loc='best', frameon=True, framealpha=0.9)
    fmax = max(max(f_sensor), f_setpoint, 0.1)
    fmin = min(min(f_sensor), 0.0)
    ax_f.set_ylim(fmin - 0.05 * fmax, fmax * 1.25)

    # Κάτω: Θέση x_ee
    ax_p.plot(t, x_ee, color='#2ed573', linewidth=2.2, label='x_ee (κέντρο)')
    ax_p.axhline(CONTACT_X_MM, color='#ff6b81', linestyle='--', linewidth=1.5,
                 label=f'Επαφή ({CONTACT_X_MM:.0f} mm)')
    ax_p.axhline(WALL_FACE_MM, color='#7f8fa6', linestyle=':', linewidth=1.5,
                 label=f'Όψη τοίχου ({WALL_FACE_MM:.0f} mm)')
    ax_p.set_title('Θέση X του end-effector', fontsize=12, fontweight='semibold')
    ax_p.set_xlabel('Χρόνος (s)', fontsize=11)
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
