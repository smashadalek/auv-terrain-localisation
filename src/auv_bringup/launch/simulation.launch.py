from launch import LaunchDescription

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    rviz_config = PathJoinSubstitution([
        FindPackageShare('auv_bringup'),
        'config',
        'auv.rviz'
    ])

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
        ),

        Node(
            package='auv_particle_filter',
            executable='particle_filter_node',
            name='particle_filter',
            output='screen'
        ),

        Node(
            package='auv_vehicle_sim',
            executable='vehicle_sim_node',
            name='vehicle_sim',
            output='screen'
        ),

        Node(
            package='auv_visualisation',
            executable='visualisation_node',
            name='auv_visualisation',
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', rviz_config],
            output='screen'
        )

    ])
