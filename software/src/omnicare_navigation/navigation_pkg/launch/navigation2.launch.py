from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable

def generate_launch_description():

#     simulation = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([get_package_share_directory('omnicare_simulation'), '/launch/simulation.launch.py']),
#            launch_arguments={
#                 'world_path': [get_package_share_directory('omnicare_simulation'), '/simulation/worlds/simple_room_with_fixed_boxes.world'],
#             }.items(),
#     )

# # -----------------------------------------------------


#     load_robot = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([get_package_share_directory('omnicare_bringup'),'/launch/load_sim_robot.launch.py']),
#         launch_arguments={
#             'rvizconfig': [get_package_share_directory('omnicare_simulation'), '/config/rviz/navigation.rviz'],
#         }.items()

        
#     )


# -----------------------------------------------------

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('nav2_bringup'),'/launch/bringup_launch.py']),
        launch_arguments={
            # 'namespace': LaunchConfiguration('namespace'),
            'use_namespace': 'False',
            'slam': 'False',
            'map': [get_package_share_directory('navigation_pkg'),'/config/map/QuintoAndarFull.yaml'],
            'use_sim_time': 'False',
            # 'params_file': [get_package_share_directory('nav2_bringup'),'/params/nav2_params.yaml'],
            'params_file': [get_package_share_directory('navigation_pkg'),'/config/nav/nav2_params.yaml'],
            'autostart': 'True',
            'use_composition': 'True',
            'use_respawn': 'False'
        }.items()
    )


    # checkpoint = Node(
    #     package='simulation_pkg',
    #     executable='checkpoints',
    #     name='checkpoints',
    #     parameters=[{
    #         'checkpoints_file': get_package_share_directory('simulation_pkg')+'/config/map/checkpoints.json'
    #     }],
    #     arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    # )

# -----------------------------------------------------

    ld = LaunchDescription()

    # ld.add_action(simulation)
    # ld.add_action(load_robot)
    ld.add_action(bringup_cmd)
    # ld.add_action(checkpoint)

    return ld
