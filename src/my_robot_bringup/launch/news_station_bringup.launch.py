from struct import pack
from launch import LaunchDescription
from launch_ros.actions import Node
# from my_py_pkg.my_py_pkg import robot_news_station, smartphone

def generate_launch_description():
    ld = LaunchDescription()

    # remap_number_topic = ("number", "my_number")

    robot_news_station_node = Node(
        package="my_cpp_pkg",
        executable="robot_news_station",
        name="robot1",
        parameters=[{
            'robot_name': 'robot'
        }]
    )
    robot_news_station1_node = Node(
        package="my_cpp_pkg",
        executable="robot_news_station",
        name="robot2",
        parameters=[{
            'robot_name': 'robot2'
        }]
    )
    robot_news_station2_node = Node(
        package="my_cpp_pkg",
        executable="robot_news_station",
        name="robot3",
        parameters=[{
            'robot_name': 'robot3'
        }]
    )
    robot_news_station3_node = Node(
        package="my_cpp_pkg",
        executable="robot_news_station",
        name="robot4",
        parameters=[{
            'robot_name': 'robot4'
        }]
    )
    robot_news_station4_node = Node(
        package="my_cpp_pkg",
        executable="robot_news_station",
        name="robot5",
        parameters=[{
            'robot_name': 'robot5'
        }]
    )

    smartphone_node = Node(
        package="my_cpp_pkg",
        executable="smartphone"
    )

    ld.add_action(robot_news_station_node)
    ld.add_action(robot_news_station1_node)
    ld.add_action(robot_news_station2_node)
    ld.add_action(robot_news_station3_node)
    ld.add_action(robot_news_station4_node)
    ld.add_action(smartphone_node)
    return ld