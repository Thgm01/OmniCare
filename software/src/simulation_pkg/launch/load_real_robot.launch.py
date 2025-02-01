from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess,RegisterEventHandler, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import xacro
import os


def generate_launch_description():

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    urdf_tutorial_path = get_package_share_path('simulation_pkg')

    default_model_path = urdf_tutorial_path / 'urdf/robot.xacro'
    default_rviz_config_path = urdf_tutorial_path / 'config/rviz/robot.rviz'

    gui_arg = DeclareLaunchArgument(name='gui', default_value='false', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')
    use_sim_time_arg = DeclareLaunchArgument(name='use_sim_time', default_value='false',
                                            description='Flag to enable use_sim_time')


    xacro_file = os.path.join(urdf_tutorial_path, 'urdf', 'robot.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    
    # robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
    #                                    value_type=str)
    # controller_params_file = get_package_share_path('base_description') / 'config/omnidirectional_controller.yaml'


    log_level = DeclareLaunchArgument(
        name='log_level', 
        default_value='INFO', 
        choices=['DEBUG','INFO','WARN','ERROR','FATAL'],
        description='Flag to set log level'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='log',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'), 
            'robot_description': robot_urdf
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )

    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     condition=UnlessCondition(LaunchConfiguration('gui')),
    #     output='log',
    #     parameters=[{
    #         'use_sim_time': LaunchConfiguration('use_sim_time')
    #     }],
    #     arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    # )

    # controller_manager = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[{'robot_description': robot_description}, controller_params_file]
    # )

    # delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    # omni_base_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["omnidirectional_controller"],
    # )


    # delayed_omni_drive_spawner = RegisterEventHandler(
    #     event_handler=OnProcessStart(
    #         target_action=controller_manager,
    #         on_start=[omni_base_controller_spawner],
    #     )
    # )

    # joint_state_broadcaster_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_state_broadcaster"],
    # )

    # delayed_joint_broad_spawner = RegisterEventHandler(
    #     event_handler=OnProcessStart(
    #         target_action=controller_manager,
    #         on_start=[joint_state_broadcaster_spawner],
    #     )
    # )

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

    
    return LaunchDescription([
        declare_namespace_cmd,
        log_level,
        gui_arg,
        model_arg,
        rviz_arg,
        use_sim_time_arg,
        # joint_state_publisher_node,
        # joint_state_publisher_gui_node,
        robot_state_publisher_node,
        # delayed_controller_manager,
        # delayed_omni_drive_spawner,
        # delayed_joint_broad_spawner,
        
        spawn_entity,
        # robot_localization_node,

        rviz_node
        # teleop
    ])
