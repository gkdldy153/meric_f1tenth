from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('pure_pursuit'),
        'config',
        'config.yaml'
    )

    pure_pursuit = Node(
        package='pure_pursuit',
        executable='pure_pursuit',
        name='pure_pursuit',
        parameters=[config]
    )

    waypoint_visualizer_node = Node(
        package='pure_pursuit',
        executable='waypoint_visualizer',
        name='waypoint_visualizer_node',
        parameters=[config]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(get_package_share_directory(
            'pure_pursuit'), 'launch', 'pure_pursuit.rviz')],
    )
    
    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_laser',
        arguments=['0.15', '0', '0.1', '0', '0', '0', 'base_link', 'laser']
    )


    # finalize
    ld.add_action(static_tf_laser)
    ld.add_action(rviz_node)
    ld.add_action(pure_pursuit)
    ld.add_action(waypoint_visualizer_node)

    return ld
