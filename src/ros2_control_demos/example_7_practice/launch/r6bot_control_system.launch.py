from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    ####################################################################################################################
    # 获取控制器节点
    ####################################################################################################################
    # 获取控制器配置文件
    robot_controllers_config_file = PathJoinSubstitution(
        [
            FindPackageShare("r6bot_control_system"),
            "r6bot_controller.yaml",
        ]
    )

    # 启动控制器节点
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers_config_file],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output="both",
    )

    ####################################################################################################################
    # 获取机器人状态发布节点
    ####################################################################################################################
    # 将urdf文件转换为robot_description
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

    # 发布机器人状态
    robot_description = {"robot_description": robot_description_content}

    # 发布机器人状态
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    ####################################################################################################################
    # 获取rviz节点
    ####################################################################################################################
    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("r6bot_control_system"),
            "description",
            "rviz",
            "view_r6bot.rviz"
        ]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    return