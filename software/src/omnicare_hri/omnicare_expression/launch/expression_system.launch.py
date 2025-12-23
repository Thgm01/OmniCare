from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # ======= Nó principal de Expressões (VLC + controle de timeline) =======
        Node(
            package='omnicare_expression',
            executable='expressionsNode',
            name='expressions_node',
            output='screen'
        ),

        # ======= Interface para o buzzer =======
        Node(
            package='omnicare_expression',
            executable='alertNode',
            name='alertNode',
            output='screen'
        ),

        # ======= Serviço de Expressões (mover braço, buzzer, etc.) =======
        Node(
            package='omnicare_expression',
            executable='expressionService',
            name='expression_service',
            output='screen'
        ),        
    ])
