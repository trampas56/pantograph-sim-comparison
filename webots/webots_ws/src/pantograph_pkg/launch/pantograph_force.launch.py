import os
import getpass
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

def webots_waiting(tmp_dir):
    # Ανοιχτό Webots με αυτόν τον κόσμο = socket όπου περιμένει τον <extern> controller (port 1234).
    # Το /proc/net/unix έχει μόνο ζωντανά sockets, όχι αρχεία που έμειναν από Webots που έσκασε.
    socket_path = os.path.join(tmp_dir, 'webots', getpass.getuser(), '1234', 'ipc', 'five_bar_mechanism', 'extern')
    with open('/proc/net/unix') as f:
        return any(line.rstrip('\n').endswith(' ' + socket_path) for line in f)

def generate_launch_description():
    package_dir = get_package_share_directory('pantograph_pkg')
    world_path = os.path.join(package_dir, 'worlds', 'pantograph_world.wbt')

    # Το Webots από μενού ή εντολή webots (snap) περιμένει στο ~/snap/webots/common/tmp, όπου η γέφυρα δεν ψάχνει
    webots_open = webots_waiting('/tmp')
    if not webots_open and webots_waiting(os.path.expanduser('~/snap/webots/common/tmp')):
        return LaunchDescription([launch.actions.LogInfo(msg=(
            'Το Webots άνοιξε μέσα από το snap (μενού ή εντολή webots) και η γέφυρα ROS 2 δεν μπορεί να συνδεθεί. '
            f'Κλείσ\' το και άνοιξέ το με: /snap/webots/current/usr/share/webots/webots {world_path}'))])

    # 1. Άνοιγμα του Webots με τον ίδιο κόσμο, εκτός αν είναι ήδη ανοιχτό
    #    (τότε η γέφυρα συνδέεται σε εκείνο και δεν ανοίγει δεύτερο)
    if webots_open:
        webots = launch.actions.LogInfo(msg='Το Webots είναι ήδη ανοιχτό: συνδέομαι σε αυτό')
    else:
        webots = WebotsLauncher(
            world=world_path
        )
    
    # 2. Εκκίνηση της γέφυρας ROS 2 (φορτώνει το torque URDF)
    robot_description_path = os.path.join(package_dir, 'resource', 'pantograph_torque_ros2.urdf')
    pantograph_driver = WebotsController(
        robot_name='five_bar_mechanism',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )
    
    # 3. Εκκίνηση του Force Control Script
    force_node = Node(
        package='pantograph_pkg',
        executable='pantograph_force',
        name='pantograph_force_controller',
        output='screen'
    )

    return LaunchDescription([
        webots,
        pantograph_driver,
        force_node,
        # Όταν κλείσει το Webots σταματάνε όλα (αν ήταν ήδη ανοιχτό, το καταλαβαίνουμε από τη γέφυρα)
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=pantograph_driver if webots_open else webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
