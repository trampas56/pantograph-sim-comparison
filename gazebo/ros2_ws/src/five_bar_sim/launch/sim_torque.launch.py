import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_five_bar   = get_package_share_directory('five_bar_sim')

    # model_torque.sdf: Torque Control (ApplyJointForce plugins)
    sdf_file       = os.path.join(pkg_five_bar, 'models', 'model_torque.sdf')
    sdf_world_file = os.path.join(pkg_five_bar, 'worlds', 'world.sdf')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['"', sdf_world_file, '" -r -v 4']}.items()
    )

    spawn_model = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'my_pantograph',
            '-file', sdf_file,
            '-x', '0.0', '-y', '0.0', '-z', '0.0'
        ],
        output='screen'
    )

    # Bridge for Torque Control:
    # ROS 2 -> Gazebo: cmd_force topics for the motors
    # Gazebo -> ROS 2: joint_state for feedback (PD controller)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/my_pantograph/joint/left_motor_joint/cmd_force@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/my_pantograph/joint/right_motor_joint/cmd_force@std_msgs/msg/Float64]gz.msgs.Double',
            '/world/five_bar_world/model/my_pantograph/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            # Gazebo -> ROS 2: Force/Torque sensor at the end-effector (actual contact force)
            '/ee_ft@geometry_msgs/msg/Wrench[gz.msgs.Wrench',
        ],
        output='screen'
    )

    return LaunchDescription([gazebo, spawn_model, bridge])
