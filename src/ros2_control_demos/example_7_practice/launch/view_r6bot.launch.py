from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ####################################################################################################################
    # 获取关节状态发布节点
    ####################################################################################################################
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    ####################################################################################################################
    # 获取机器人状态发布节点
    ####################################################################################################################
    # 获取urdf文件
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

    # 将urdf文件转换为robot_description
    robot_description = {"robot_description": robot_description_content}

    # 发布机器人状态
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    ####################################################################################################################
    # 获取rviz节点
    ####################################################################################################################
    # 获取rviz配置文件
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("r6bot_control_system"), "description", "rviz", "view_r6bot.rviz"]
    )

    # 启动rviz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    ####################################################################################################################
    # 准备启动节点
    ####################################################################################################################
    # 启动节点
    nodes = [
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
    ]

    return LaunchDescription(nodes)
