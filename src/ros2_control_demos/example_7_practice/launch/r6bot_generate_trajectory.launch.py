from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("r6bot_control_system"),
                    "description",
                    "urdf",
                    "r6bot.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    trajectory_generator_node = Node(
        package="r6bot_control_system",
        executable="generate_trajectory",
        name="trajectory_generator_node",
        parameters=[robot_description],
    )

    nodes_to_start = [trajectory_generator_node]
    return LaunchDescription(nodes_to_start)
