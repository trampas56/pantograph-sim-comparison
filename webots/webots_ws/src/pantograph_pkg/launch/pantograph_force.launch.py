import os
import getpass
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

def webots_waiting(tmp_dir):
    # Webots open with this world = a socket where it waits for the <extern> controller (port 1234).
    # /proc/net/unix only lists live sockets, not files left behind by a Webots that crashed.
    socket_path = os.path.join(tmp_dir, 'webots', getpass.getuser(), '1234', 'ipc', 'five_bar_mechanism', 'extern')
    with open('/proc/net/unix') as f:
        return any(line.rstrip('\n').endswith(' ' + socket_path) for line in f)

def generate_launch_description():
    package_dir = get_package_share_directory('pantograph_pkg')
    world_path = os.path.join(package_dir, 'worlds', 'pantograph_world.wbt')

    # Webots started from the menu or the webots command (snap) waits in ~/snap/webots/common/tmp, where the bridge does not look
    webots_open = webots_waiting('/tmp')
    if not webots_open and webots_waiting(os.path.expanduser('~/snap/webots/common/tmp')):
        return LaunchDescription([launch.actions.LogInfo(msg=(
            'Webots was opened through the snap (menu or webots command) and the ROS 2 bridge cannot connect. '
            f'Close it and open it with: /snap/webots/current/usr/share/webots/webots {world_path}'))])

    # 1. Open Webots with the same world, unless it is already open
    #    (then the bridge connects to that one and does not open a second one)
    if webots_open:
        webots = launch.actions.LogInfo(msg='Webots is already open: connecting to it')
    else:
        webots = WebotsLauncher(
            world=world_path
        )
    
    # 2. Start the ROS 2 bridge (loads the torque URDF)
    robot_description_path = os.path.join(package_dir, 'resource', 'pantograph_torque_ros2.urdf')
    pantograph_driver = WebotsController(
        robot_name='five_bar_mechanism',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )
    
    # 3. Start the Force Control Script
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
        # When Webots closes everything stops (if it was already open, we notice through the bridge)
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=pantograph_driver if webots_open else webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
