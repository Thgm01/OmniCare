from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('base_description')

    xacro_file = os.path.join(share_dir, 'urdf', 'Base.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()
    
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf}
        ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    gz_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=['/home/robot/gazebo_models:',get_package_share_directory('base_description'),'/simulation/models:/opt/ros/humble/share/turtlebot3_gazebo/models']
    )


    world_path = DeclareLaunchArgument(
        name='world_path', 
        default_value='', 
        description=''
    )

    gzserver = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('gazebo_ros'), '/launch/gzserver.launch.py']),
            launch_arguments = {
                'verbose': 'false',
                'world': LaunchConfiguration('world_path')
            }.items()
        )

    gzclient = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('gazebo_ros'), '/launch/gzclient.launch.py']),
            # condition=IfCondition(LaunchConfiguration('simulation_gui'))
        )

    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'Base',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    return LaunchDescription([
        gz_model_path,
        world_path,
        declare_namespace_cmd,
        robot_state_publisher_node,
        joint_state_publisher_node,
        gzserver,
        gzclient,
        urdf_spawn_node,
    ])
