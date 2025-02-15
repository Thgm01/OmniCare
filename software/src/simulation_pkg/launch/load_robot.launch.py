from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess,RegisterEventHandler, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    urdf_path = get_package_share_path('omnicare_description')
    rviz_path = get_package_share_path('simulation_pkg')

    default_model_path = urdf_path / 'urdf/robot.xacro'
    default_rviz_config_path = rviz_path / 'config/rviz/robot.rviz'

    gui_arg = DeclareLaunchArgument(name='gui', default_value='false', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')
    use_sim_time_arg = DeclareLaunchArgument(name='use_sim_time', default_value='true',
                                            description='Flag to enable use_sim_time')

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)
    
    # xacro_file = os.path.join(urdf_tutorial_path, 'urdf', 'robot.xacro')
    robot_description_config = xacro.process_file(default_model_path)
    robot_urdf = robot_description_config.toxml()
    
    controller_params_file = get_package_share_path('omnicare_control_pkg') / 'config/ros2_control/omnidirectional_controller.yaml'


    log_level = DeclareLaunchArgument(
        name='log_level', 
        default_value='INFO', 
        choices=['DEBUG','INFO','WARN','ERROR','FATAL'],
        description='Flag to set log level'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'), 
            'robot_description': robot_urdf
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui')),
        output='log',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )

    twist_mux_params = os.path.join(get_package_share_directory('simulation_pkg'),'config/nav','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
    )
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                controller_params_file]
    )



    omni_base_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["omnidirectional_controller"],
    )

    omni_base_controller_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[omni_base_controller_spawner]
        )
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    joint_state_broadcaster_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_state_broadcaster_spawner]
        )
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='own_log',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        arguments=['-d', LaunchConfiguration('rvizconfig'), '--ros-args', '--log-level', LaunchConfiguration('log_level')]

    )

    teleop = Node(
        name='teleop_twist_keyboard',
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        output='log',
        prefix=["xterm -hold -e"],
        remappings=[
            # ('/cmd_vel', '/demo/cmd_vel'),
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )

    spawn_entity = Node(
    	package='gazebo_ros', 
    	executable='spawn_entity.py',
        arguments=[
            '-entity', 'sam_bot', 
            '-topic', 'robot_description', 
            '--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[os.path.join(get_package_share_path('omnicare_description'), 'config/ros2_control/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )


    
    return LaunchDescription([
        declare_namespace_cmd,
        log_level,
        gui_arg,
        model_arg,
        rviz_arg,
        use_sim_time_arg,
        # joint_state_publisher_node, //publica as juntas
        # joint_state_publisher_gui_node, //controla as juntas
        robot_state_publisher_node, # publica o robô em si (URDF)
        spawn_entity, #Spawn do robô
        rviz_node, #RVIZ2 para debug
        # twist_mux, #MUX de prioridade das velocidades
        # omni_base_controller_spawner, #ROS2_CONTROL
        # omni_base_controller_event_handler, #ROS2_CONTROL
        # joint_state_broadcaster_spawner, #ROS2_CONTROL
        # joint_state_broadcaster_event_handler, #ROS2_CONTROL
        # robot_localization_node

    ])
