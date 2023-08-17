from codecs import escape_encode
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node"
    )

    turtle_spawner_node = Node(
        package="my_final_homework",
        executable="turtle_spawner",
        parameters=[
            {"spawn_milliseconds": 500}
        ]
    )

    turtle_controller_node = Node(
        package="my_final_homework",
        executable="turtle_controller",
        parameters=[
            {"catch_closest": False}
        ]
    )
    ld.add_action(turtlesim_node)
    ld.add_action(turtle_spawner_node)
    ld.add_action(turtle_controller_node)

    return ld