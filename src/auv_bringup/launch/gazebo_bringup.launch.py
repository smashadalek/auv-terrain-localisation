from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    world_file = PathJoinSubstitution([
        FindPackageShare('auv_bringup'),
        'worlds',
        'simple_auv.sdf'
    ])

    rviz_config = PathJoinSubstitution([
        FindPackageShare('auv_bringup'),
        'config',
        'auv.rviz'
    ])

    gazebo = ExecuteProcess(
        cmd=[
            'gz',
            'sim',
            '-r',
            world_file
        ],
        output='screen'
    )

    ground_truth_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/simulation/ground_truth_pose'
            '@geometry_msgs/msg/PoseStamped'
            '[gz.msgs.Pose'
        ],
        output='screen'
    )

    map_world_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'map',
            '--child-frame-id', 'auv_world'
        ],
        output='screen'
    )

    visualisation = Node(
        package='auv_visualisation',
        executable='visualisation_node',
        name='auv_visualisation',
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        ground_truth_bridge,
        map_world_tf,
        visualisation,
        rviz,
    ])