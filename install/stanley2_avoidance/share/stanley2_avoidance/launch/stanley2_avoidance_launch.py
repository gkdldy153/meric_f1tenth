from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('stanley2_avoidance'),
        'config',
        'config.yaml'
    )

    stanley2_avoidance_node = Node(
        package='stanley2_avoidance',
        executable='stanley2_avoidance',
        name='stanley2_avoidance',
        parameters=[config]
    )

    '''rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(get_package_share_directory(
            'stanley2_avoidance'), 'launch', 'stanley_avoidance.rviz')]
    )'''

    waypoint_visualizer_node = Node(
        package='pure_pursuit',
        executable='waypoint_visualizer',
        name='waypoint_visualizer_node',
        parameters=[config]
    )

    # finalize
    # ld.add_action(rviz_node)
    ld.add_action(stanley2_avoidance_node)
    ld.add_action(waypoint_visualizer_node)

    return ld
