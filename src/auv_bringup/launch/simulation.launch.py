from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='auv_map_server',
            executable='map_server_node',
            name='map_server',
            output='screen'
        ),

        Node(
            package='auv_sensor_sim',
            executable='sonar_sim_node',
            name='sonar_sim',
            output='screen'
        )

    ])
