#!/usr/bin/env python3
"""
plot_unity_force.py - Γραφική σύγκριση δυνάμεων & θέσης από το unity_force_log.csv
(Unity five-bar, Phase 2β: έλεγχος δύναμης με J^T). Αντίστοιχο του Gazebo plot_force_results.py.

  (πάνω)  Δύναμη στο EE: Αναμενόμενη (F_cmd, εντολή = (J^T)^-1 * τ)
          vs Πραγματική από αισθητήρα (F_sensor = ContactForceProbe).
  (κάτω)  Θέση (reach) του end-effector σε mm, με γραμμές Επαφής & Όψης τοίχου,
          ώστε να φαίνεται η στιγμή πρόσκρουσης και η ακινητοποίηση.
"""
import os
import csv
import matplotlib
matplotlib.use('Agg')  # headless-safe (PNG χωρίς display)
import matplotlib.pyplot as plt

# Γεωμετρία τοίχου (από PantographWall.cs) - η σφαίρα tip r=0.015
EE_RADIUS_MM       = 15.0
WALL_SURFACE_REACH = 0.140    # επιφάνεια επαφής (reach offset από το EE home)
EE_HOME_REACH      = 0.0898   # reach του EE στο home (SDF)

# Σχετικά με τη θέση του script (unity/pantograph_drive/) - το Unity γράφει το CSV
# στο PantographSim/ (Application.dataPath/..), άρα δουλεύει σε οποιοδήποτε μηχάνημα/χρήστη.
SIM_DIR  = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'PantographSim')
CSV_PATH = os.path.abspath(os.path.join(SIM_DIR, 'unity_force_log.csv'))
OUT_PNG  = os.path.abspath(os.path.join(SIM_DIR, 'unity_force_plot.png'))


def main():
    if not os.path.exists(CSV_PATH):
        print(f"Error: δεν βρέθηκε {CSV_PATH} - τρέξε πρώτα το Play με τον PantographForceControl.")
        return
    print(f"Reading log data from: {CSV_PATH}")

    t, reach, f_cmd, f_est, f_sensor = [], [], [], [], []
    with open(CSV_PATH, 'r') as f:
        for row in csv.DictReader(f):
            t.append(float(row['t']))
            reach.append(float(row['ee_reach']) * 1000.0)   # mm
            f_cmd.append(float(row['F_cmd_N']))
            f_est.append(float(row.get('F_est_N', 0.0) or 0.0))   # (J^T)^-1 * τ - δείχνει τον κορεσμό
            f_sensor.append(float(row['contactF_N']))
    has_est = any(abs(v) > 1e-9 for v in f_est)   # παλιά CSV χωρίς τη στήλη -> χωρίς μπλε γραμμή

    if not t:
        print("Error: το CSV δεν έχει δεδομένα.")
        return

    # Γραμμές αναφοράς στο σύστημα reach του ελεγκτή (home = πρώτη τιμή)
    home_mm      = reach[0]
    wall_face_mm = home_mm + (WALL_SURFACE_REACH - EE_HOME_REACH) * 1000.0
    contact_mm   = wall_face_mm - EE_RADIUS_MM     # εκεί ακινητοποιείται το κέντρο της σφαίρας

    duration = t[-1] - t[0] if len(t) > 1 else 10
    fig_width = max(11.0, duration / 5.0)
    fig, (ax_f, ax_p) = plt.subplots(2, 1, figsize=(fig_width, 8), sharex=True)
    fig.suptitle('Five-Bar Pantograph (Unity) - Έλεγχος Δύναμης Επαφής',
                 fontsize=15, fontweight='bold')

    # Πάνω: Δυνάμεις στο EE (3 καμπύλες, όπως Gazebo)
    ax_f.plot(t, f_cmd, color='#2f3542', linestyle='--', linewidth=2.2,
              label='Επιθυμητή  F_des (εντολή)')
    if has_est:
        ax_f.plot(t, f_est, color='#1e90ff', linestyle='-', linewidth=2.0,
                  label='Εκτιμώμενη  F_est = (J^T)^-1 * τ  (μετά το όριο ροπής)')
    ax_f.plot(t, f_sensor, color='#ff4757', linestyle='-', linewidth=2.0,
              label='Πραγματική  F_sensor (FT αισθητήρας)')
    ax_f.set_title('Δύναμη στο end-effector (προς τον τοίχο)', fontsize=12, fontweight='semibold')
    ax_f.set_ylabel('Δύναμη (N)', fontsize=11)
    ax_f.grid(True, linestyle=':', alpha=0.6)
    ax_f.legend(loc='best', frameon=True, framealpha=0.9)
    fmax = max(max(f_cmd), max(f_est), max(f_sensor), 1.0)
    ax_f.set_ylim(-0.15, fmax * 1.25)
    # τελική τιμή αισθητήρα ως annotation
    ax_f.annotate(f'{f_sensor[-1]:.2f} N', xy=(t[-1], f_sensor[-1]),
                  xytext=(-60, 14), textcoords='offset points',
                  color='#ff4757', fontweight='bold',
                  arrowprops=dict(arrowstyle='->', color='#ff4757'))

    # Κάτω: Θέση (reach)
    ax_p.plot(t, reach, color='#2ed573', linewidth=2.2, label='reach EE (κέντρο σφαίρας)')
    ax_p.axhline(contact_mm, color='#ff6b81', linestyle='--', linewidth=1.5,
                 label=f'Επαφή ({contact_mm:.0f} mm)')
    ax_p.axhline(wall_face_mm, color='#7f8fa6', linestyle=':', linewidth=1.5,
                 label=f'Όψη τοίχου ({wall_face_mm:.0f} mm)')
    ax_p.set_title('Θέση (reach) του end-effector', fontsize=12, fontweight='semibold')
    ax_p.set_xlabel('Χρόνος (s)', fontsize=11)
    ax_p.set_ylabel('reach (mm)', fontsize=11)
    ax_p.grid(True, linestyle=':', alpha=0.6)
    ax_p.legend(loc='best', frameon=True, framealpha=0.9)

    plt.tight_layout(rect=(0, 0, 1, 0.97))
    plt.savefig(OUT_PNG, dpi=150)
    print(f"Plot saved to: {OUT_PNG}")
    print(f"Τελικά: F_cmd={f_cmd[-1]:.3f} N | F_sensor={f_sensor[-1]:.3f} N | "
          f"reach={reach[-1]:.1f} mm (επαφή στα {contact_mm:.1f} mm)")


if __name__ == '__main__':
    main()
