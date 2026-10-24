Διπλωματική εργασία - Τράμπας Ελευθέριος
Δοκιμή και Αξιολόγηση Λογισμικών Προσομοίωσης με Εφαρμογή σε Παντογράφο

Εδώ είναι ο κώδικας της διπλωματικής. Ο ίδιος παντογράφος είναι στημένος σε τέσσερις προσομοιωτές (Gazebo, Webots, MuJoCo και Unity), και σε όλους τρέχει το ίδιο πείραμα, όπου το άκρο πιέζει έναν τοίχο με 0,5, 1 και 2 N.

Ο φάκελος πρέπει να μπει στο Desktop με το όνομα pantograph, δηλαδή ~/Desktop/pantograph. Έτσι είναι γραμμένες οι εντολές στο Παράρτημα Α της εργασίας (Thesis_4625.pdf). Επίσης ο ελεγκτής του Webots έχει αυτή τη διαδρομή μέσα στον κώδικα (~/Desktop/pantograph/webots), οπότε αν ο φάκελος μπει κάπου αλλού πρέπει να αλλάξει.

Τι έχει μέσα:

- gazebo/ros2_ws/src/five_bar_sim: το ROS 2 package για τον Gazebo
- webots/webots_ws/src/pantograph_pkg: το ROS 2 package για το Webots
- mujoco/mujoco_ws/src/pantograph_bot: το ROS 2 package για τον MuJoCo
- unity/PantographSim: το project του Unity
- unity/ROS-TCP-Connector: package που χρειάζεται το project του Unity για να ανοίξει
- unity/pantograph_drive: το script για το γράφημα του Unity (plot_unity_force.py) και το Kinematics.py
- unity/experiments-2026-10-02-damping: η δοκιμή με την απόσβεση και το βήμα στο Unity (ενότητα 10.4)
- thesis/data/force: τα 12 csv με τις μετρήσεις του κεφαλαίου 11
- thesis/scripts/fig_force_levels.py: βγάζει από αυτά τα csv τις εικόνες 11.1, 11.2 και 11.3

Δεν έβαλα τους φακέλους build, install και log των workspace ούτε το Library του Unity, γιατί είναι μεγάλοι και φτιάχνονται ξανά. Για το ROS 2 αρκεί ένα colcon build μέσα σε κάθε workspace. Το Unity φτιάχνει το Library μόνο του την πρώτη φορά που ανοίγει το project, αλλά θέλει internet για να κατεβάσει το URDF Importer από το GitHub.

Δούλεψα σε Ubuntu 24.04 με ROS 2 Jazzy, Gazebo Harmonic, Webots R2025a, MuJoCo (από το pip) και Unity 6000.4.7f1. Οι ακριβείς εκδόσεις είναι στον Πίνακα 6.1 της εργασίας.
