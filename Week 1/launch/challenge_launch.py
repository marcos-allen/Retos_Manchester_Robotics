from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        # Nodo generador
        Node(
            package='reto_1_equipo_5',
            executable='signal_generator',
            name='signal_generator_node',
            output='screen'
        ),

        # Nodo procesador
        Node(
            package='reto_1_equipo_5',
            executable='signal_processor',
            name='signal_processor_node',
            output='screen'
        ),

        # rqt_plot
        Node(
            package='rqt_plot',
            executable='rqt_plot',
            name='rqt_plot',
            arguments=[
                '/signal',
                '/proc_signal'
            ],
            output='screen'
        )

    ])
