from launch_ros.actions import Node 
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    package_dir = get_package_share_directory('omnicare_vision')

    real_robot = LaunchConfiguration('real_robot')
    launch_arg_real_robot = DeclareLaunchArgument(
        name='real_robot',
        default_value='True',
        choices=['True', 'False']
    )

    camera_config_file = os.path.join(package_dir, 'config', 'camera.yaml')
    node_camera = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        output='screen',
        parameters=[camera_config_file],
        respawn=True,
        respawn_delay=0.5,
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('real_robot'))
    )

    return LaunchDescription([
        launch_arg_real_robot,
        node_camera,
    ])