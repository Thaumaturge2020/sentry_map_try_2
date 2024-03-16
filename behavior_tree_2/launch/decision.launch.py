from launch import LaunchDescription
from launch_ros.actions import Node

package_dir = "/home/ubuntu/sentry_ws/src/sentry_decision/src/behavior_tree_2"

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="behavior_tree_2",
                executable="btree",
                name="test_node",
                output="screen",
                # parameters=[{"agoalX": 14.0}, {"agoalY": 14.0}, {"useTest": True}],
                parameters=[
                    {"agoalX": 11.0}, 
                    {"agoalY": 3.5}, 
                    {"useTest": True},
                    {"initPosX": 6.6},
                    {"initPosY": 7.3}
                ],
            ),
        ]
    )
