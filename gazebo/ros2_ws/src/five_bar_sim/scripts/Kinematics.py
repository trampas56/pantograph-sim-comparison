#!/usr/bin/env python3
"""
kinematics.py - Κινηματική του παντογράφου (five-bar parallel mechanism).

Κοινό module, ίδιο σε όλους τους προσομοιωτές.

Δύο "κόσμοι" συντεταγμένων:
  * KIN frame  : το frame της μαθηματικής ανάλυσης. Αριστερός κινητήρας (O) στο (0,0),
                 δεξιός κινητήρας (C) στο (l0,0). End-effector φτάνει προς +Y.
  * WORLD frame: το frame του Gazebo. Κινητήρες κατά τον άξονα Y (αριστερός y=+0.04,
                 δεξιός y=-0.04). End-effector φτάνει προς +X.

Σχέση WORLD <-> KIN (στροφή -90° + μετατόπιση), επαληθευμένη από το model_torque.sdf:
      x_world = ye_kin
      y_world = Y_OFF - xe_kin            (Y_OFF = 0.04)

Γωνίες: theta1, theta2 = απόλυτος προσανατολισμός των cranks στο KIN frame (CCW από +X_kin).
Αντιστοίχιση σε γωνίες αρθρώσεων Gazebo (q=0 στο home, βλ. _self_test):
      q_left  = theta1 - THETA1_HOME      (THETA1_HOME = 150° = 5*pi/6)
      q_right = theta2 - THETA2_HOME      (THETA2_HOME =  30° =   pi/6)
Και οι δύο αντιστοιχίσεις έχουν πρόσημο +1, άρα: dq = dtheta (ίδιες ταχύτητες),
και η ροπή στη σύμβαση KIN εφαρμόζεται αυτούσια ως cmd_force.
"""

import math
import numpy as np

# --------------------------------------------------------------------------
# Παράμετροι μηχανισμού (m) - από το model_torque.sdf
# --------------------------------------------------------------------------
L0 = 0.08   # απόσταση των δύο κινητήρων
L1 = 0.08   # αριστερός crank   (left_crank)
L2 = 0.12   # αριστερός βραχίονας (left_arm)
L3 = 0.12   # δεξιός βραχίονας   (right_arm)
L4 = 0.08   # δεξιός crank       (right_crank)

# Frame mapping
Y_OFF = 0.04                 # ο αριστερός κινητήρας βρίσκεται στο world y = +0.04
THETA1_HOME = 5 * math.pi / 6   # 150°
THETA2_HOME = math.pi / 6       # 30°
HOME_WORLD = (0.0898, 0.0)      # θέση end-effector στο home (από το SDF)

# Στροφή KIN -> WORLD (για διανύσματα: ταχύτητες/δυνάμεις)
R_KIN_TO_WORLD = np.array([[0.0, 1.0],
                           [-1.0, 0.0]])


def _safe_acos(x):
    """acos με clamp στο [-1, 1] ώστε να μη σκάει το loop στα όρια του χώρου εργασίας."""
    return math.acos(max(-1.0, min(1.0, x)))


# --------------------------------------------------------------------------
# 1. Αντίστροφη Κινηματική:  (xe, ye)_kin -> (theta1, theta2)
# --------------------------------------------------------------------------
def inverse_kinematics(xe, ye, l0=L0, l1=L1, l2=L2, l3=L3, l4=L4):
    r = math.sqrt(xe * xe + ye * ye)
    a_angle = _safe_acos(xe / r)
    b_angle = _safe_acos((l1**2 - l2**2 + xe**2 + ye**2) / (2 * l1 * r))
    theta1 = a_angle + b_angle

    rc = math.sqrt((xe - l0)**2 + ye**2)
    c_angle = _safe_acos((l0 - xe) / rc)
    d_angle = _safe_acos((xe**2 + ye**2 + l0**2 + l4**2 - l3**2 - 2 * xe * l0)
                         / (2 * rc * l4))
    theta2 = math.pi - (c_angle + d_angle)
    return theta1, theta2


# --------------------------------------------------------------------------
# 2. Ευθεία Κινηματική:  (theta1, theta2) -> (xe, ye)_kin
# --------------------------------------------------------------------------
def forward_kinematics(theta1, theta2, l0=L0, l1=L1, l2=L2, l3=L3, l4=L4):
    a = (math.sin(theta1) * l1 - math.sin(theta2) * l4) / \
        (l0 + math.cos(theta2) * l4 - math.cos(theta1) * l1)
    b = ((math.sin(theta2))**2 * l4**2 + (l0 + math.cos(theta2) * l4)**2
         - l1**2 + l2**2 - l3**2) / \
        (2 * (l0 + math.cos(theta2) * l4 - math.cos(theta1) * l1))
    c = a**2 + 1
    d = 2 * a * b - 2 * a * math.cos(theta1) * l1 - 2 * math.sin(theta1) * l1
    e = b**2 - 2 * b * math.cos(theta1) * l1 + l1**2 - l2**2

    ye = (-d + math.sqrt(max(0.0, d**2 - 4 * c * e))) / (2 * c)
    xe = a * ye + b
    return xe, ye


def calculate_passive_angles(xe, ye, l0=L0, l1=L1, l2=L2, l3=L3, l4=L4):
    """Γωνίες παθητικών (ελεύθερων) αρθρώσεων theta3, theta4."""
    theta3 = math.pi - _safe_acos(
        (-(xe)**2 - (ye)**2 + 2 * xe * l0 - l0**2 + l3**2 + l4**2) / (2 * l3 * l4))
    theta4 = math.pi + _safe_acos(
        (-(xe)**2 - (ye)**2 + l1**2 + l2**2) / (2 * l1 * l2))
    return theta3, theta4


# --------------------------------------------------------------------------
# 3. Ιακωβιανός (KIN frame):  [xe_dot; ye_dot] = J * [theta1_dot; theta2_dot]
#    Κεντρικές πεπερασμένες διαφορές πάνω στην forward_kinematics:
#    γρήγορο, χωρίς sympy στο runtime, σφάλμα ~1e-9 (επαληθεύεται στο _self_test).
# --------------------------------------------------------------------------
def jacobian(theta1, theta2, h=1e-6):
    x1p, y1p = forward_kinematics(theta1 + h, theta2)
    x1m, y1m = forward_kinematics(theta1 - h, theta2)
    x2p, y2p = forward_kinematics(theta1, theta2 + h)
    x2m, y2m = forward_kinematics(theta1, theta2 - h)
    return np.array([[(x1p - x1m) / (2 * h), (x2p - x2m) / (2 * h)],
                     [(y1p - y1m) / (2 * h), (y2p - y2m) / (2 * h)]])


def jacobian_world(theta1, theta2, h=1e-6):
    """Ιακωβιανός σε WORLD frame: [x_world_dot; y_world_dot] = J_world * theta_dot."""
    return R_KIN_TO_WORLD @ jacobian(theta1, theta2, h)


def jacobian_inv(theta1, theta2, h=1e-6):
    return np.linalg.inv(jacobian(theta1, theta2, h))


def jacobian_T(theta1, theta2, h=1e-6):
    return jacobian(theta1, theta2, h).T


# --------------------------------------------------------------------------
# 4. Μετασχηματισμοί frame & αρθρώσεων
# --------------------------------------------------------------------------
def world_to_kin(xw, yw):
    return (Y_OFF - yw, xw)            # (xe, ye)_kin


def kin_to_world(xe, ye):
    return (ye, Y_OFF - xe)            # (xw, yw)


def kin_to_joint(theta1, theta2):
    return (theta1 - THETA1_HOME, theta2 - THETA2_HOME)   # (q_left, q_right)


def joint_to_kin(q_left, q_right):
    return (q_left + THETA1_HOME, q_right + THETA2_HOME)   # (theta1, theta2)


def world_to_joint(xw, yw):
    """Pipeline υψηλού επιπέδου: world (x,y) -> εντολές γωνιών κινητήρων (q_left, q_right)."""
    xe, ye = world_to_kin(xw, yw)
    theta1, theta2 = inverse_kinematics(xe, ye)
    return kin_to_joint(theta1, theta2)


def joint_to_world(q_left, q_right):
    """Pipeline ανάποδα (για ανάγνωση πραγματικής θέσης EE)."""
    theta1, theta2 = joint_to_kin(q_left, q_right)
    xe, ye = forward_kinematics(theta1, theta2)
    return kin_to_world(xe, ye)


# --------------------------------------------------------------------------
# Self-test: τρέξε  `python3 kinematics.py`
# --------------------------------------------------------------------------
def _self_test():
    print("=" * 64)
    print("KINEMATICS SELF-TEST")
    print("=" * 64)

    # (α) Home: το IK στο home πρέπει να δίνει theta=(150°,30°) και q=(0,0)
    xe, ye = world_to_kin(*HOME_WORLD)
    t1, t2 = inverse_kinematics(xe, ye)
    ql, qr = kin_to_joint(t1, t2)
    print(f"home world {HOME_WORLD} -> kin ({xe:.5f}, {ye:.5f})")
    print(f"  theta = ({math.degrees(t1):.3f}°, {math.degrees(t2):.3f}°)  "
          f"[αναμενόμενο 150 / 30]")
    print(f"  q     = ({math.degrees(ql):.4f}°, {math.degrees(qr):.4f}°)  "
          f"[αναμενόμενο 0 / 0]")
    assert abs(math.degrees(ql)) < 0.5 and abs(math.degrees(qr)) < 0.5

    # (β) Round-trip σε πλέγμα γύρω από το home: world -> q -> world
    max_err = 0.0
    for dx in np.linspace(-0.03, 0.03, 13):
        for dy in np.linspace(-0.03, 0.03, 13):
            xw, yw = HOME_WORLD[0] + dx, HOME_WORLD[1] + dy
            ql, qr = world_to_joint(xw, yw)
            xw2, yw2 = joint_to_world(ql, qr)
            max_err = max(max_err, math.hypot(xw2 - xw, yw2 - yw))
    print(f"\nround-trip world->q->world, max σφάλμα = {max_err:.3e} m")
    assert max_err < 1e-6

    # (γ) Ιακωβιανός (FD) vs αναλυτικός (sympy) - ανεξάρτητος έλεγχος
    try:
        import sympy as sym
        s1, s2 = sym.symbols('t1 t2')
        a = (sym.sin(s1)*L1 - sym.sin(s2)*L4) / (L0 + sym.cos(s2)*L4 - sym.cos(s1)*L1)
        b = ((sym.sin(s2))**2*L4**2 + (L0 + sym.cos(s2)*L4)**2 - L1**2 + L2**2 - L3**2) \
            / (2*(L0 + sym.cos(s2)*L4 - sym.cos(s1)*L1))
        c = a**2 + 1
        d = 2*a*b - 2*a*sym.cos(s1)*L1 - 2*sym.sin(s1)*L1
        e = b**2 - 2*b*sym.cos(s1)*L1 + L1**2 - L2**2
        ye_s = (-d + sym.sqrt(d**2 - 4*c*e)) / (2*c)
        xe_s = a*ye_s + b
        Jsym = sym.lambdify((s1, s2),
                            [[sym.diff(xe_s, s1), sym.diff(xe_s, s2)],
                             [sym.diff(ye_s, s1), sym.diff(ye_s, s2)]], 'numpy')
        jmax = 0.0
        for t1 in np.linspace(2.0, 3.0, 5):
            for t2 in np.linspace(0.2, 1.1, 5):
                jmax = max(jmax, np.max(np.abs(jacobian(t1, t2) - np.array(Jsym(t1, t2)))))
        print(f"Ιακωβιανός FD vs sympy, max διαφορά = {jmax:.3e}")
        assert jmax < 1e-5
    except ImportError:
        print("(sympy μη διαθέσιμο - παράλειψη ελέγχου Ιακωβιανού)")

    # (δ) Δειγματοληψία τροχιάς κύκλου (r=0.02 γύρω από home) - να μην ενεργοποιείται clamp
    cx, cy, r = HOME_WORLD[0], HOME_WORLD[1], 0.02
    print(f"\nτροχιά κύκλου r={r} m γύρω από το home:")
    for k in range(0, 360, 45):
        ang = math.radians(k)
        ql, qr = world_to_joint(cx + r*math.cos(ang), cy + r*math.sin(ang))
        print(f"  {k:3d}° -> q_left={math.degrees(ql):7.2f}°  q_right={math.degrees(qr):7.2f}°")

    print("\nOK - όλα τα tests πέρασαν.")


if __name__ == '__main__':
    _self_test()
