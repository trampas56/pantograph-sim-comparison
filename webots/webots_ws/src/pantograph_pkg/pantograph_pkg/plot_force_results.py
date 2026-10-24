#!/usr/bin/env python3
"""
plot_force_results.py - Γραφική σύγκριση δυνάμεων από το force_control_log.csv.

Παράγει διάγραμμα με:
  (πάνω)  Δύναμη επαφής στον τοίχο κατά X. Με SHOW_DES_EST = True μπαίνουν και η
          Επιθυμητή (F_des) και η Εκτιμώμενη από εξισώσεις (F_est = (J^T)^-1 * τ).
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

# Επιθυμητή (F_des) και Εκτιμώμενη (F_est) στο πάνω διάγραμμα, False = μόνο η δύναμη επαφής
SHOW_DES_EST = False


def ema_smooth(y, alpha=0.9):
    """Χαμηλοπερατό φίλτρο EMA ΜΟΝΟ για την εμφάνιση. Με δειγματοληψία ~0.01 s και
    alpha=0.9 η σταθερά χρόνου είναι ~0.1 s -> η γραμμή ανεβαίνει σε ~0.25 s αντί ακαριαία.
    Δεν αλλάζει τον έλεγχο ούτε τα δεδομένα του CSV (εφαρμόζεται μόνο στο plot)."""
    out = []
    s = 0.0
    for v in y:
        s = alpha * s + (1.0 - alpha) * v
        out.append(s)
    return out


def find_csv():
    # Use the same path as defined in ForceControl.py
    return os.path.join(os.path.expanduser('~'), 'Desktop', 'pantograph', 'webots', 'webots_force_log.csv')

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
            # Πραγματική δύναμη επαφής (από ισοζύγιο, ενεργή μόνο στην επαφή).
            # Νέα στήλη f_contact_x, fallback στο παλιό f_sensor_x για παλιά CSV.
            f_contact.append(float(row.get('f_contact_x', row.get('f_sensor_x', 0.0)) or 0.0))

    if not t:
        print("Error: CSV has no data rows.")
        return

    # Εξομάλυνση ΜΟΝΟ για την εμφάνιση (το CSV παραμένει ανέπαφο): η κόκκινη γραμμή
    # ανεβαίνει ομαλά αντί ακαριαία στη στιγμή της επαφής. Μεγαλύτερο alpha = πιο ομαλή
    # (alpha=0.97 -> άνοδος ~0.7 s, στρογγυλεύει και την αναπήδηση της επαφής).
    f_contact = ema_smooth(f_contact, alpha=0.97)

    duration = t[-1] - t[0] if len(t) > 1 else 10
    # Αυξάνουμε το πλάτος της εικόνας ανάλογα με τη διάρκεια, ώστε να μη "ζουλιούνται" τα δεδομένα αν τρέξει για 5 λεπτά
    fig_width = max(11.0, duration / 5.0) 
    fig, (ax_f, ax_p) = plt.subplots(2, 1, figsize=(fig_width, 8), sharex=True)
    fig.suptitle('Five-Bar Pantograph - Έλεγχος Δύναμης Επαφής',
                 fontsize=15, fontweight='bold')

    # Πάνω: Δυνάμεις κατά X
    shown = [f_contact]  # όσες γραμμές φαίνονται ορίζουν τα όρια του άξονα y
    if SHOW_DES_EST:
        ax_f.plot(t, f_des, color='#2f3542', linestyle='--', linewidth=2.2,
                  label='Επιθυμητή F_des')
        ax_f.plot(t, f_est, color='#1e90ff', linestyle='-', linewidth=2.0,
                  label='Εκτιμώμενη F_est  =(J^T)^-1 * τ')
        shown += [f_des, f_est]
    ax_f.plot(t, f_contact, color='#ff4757', linestyle='-', linewidth=2.4,
              label='Δύναμη επαφής στον τοίχο')
    ax_f.set_title('Δύναμη κατά X στο end-effector', fontsize=12, fontweight='semibold')
    ax_f.set_ylabel('Δύναμη (N)', fontsize=11)
    ax_f.grid(True, linestyle=':', alpha=0.6)
    ax_f.legend(loc='best', frameon=True, framealpha=0.9)
    fmax = max([max(y) for y in shown] + [1.0])
    fmin = min([min(y) for y in shown] + [0.0])
    ax_f.set_ylim(fmin - 0.2, fmax * 1.25)

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

    out_primary = os.path.join(os.path.dirname(csv_path), 'webots_force_control_plot.png')
    plt.savefig(out_primary, dpi=150)
    print(f"Plot saved to: {out_primary}")


if __name__ == '__main__':
    main()
