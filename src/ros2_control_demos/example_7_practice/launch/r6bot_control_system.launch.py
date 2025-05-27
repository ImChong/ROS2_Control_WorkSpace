from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

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

    # 启动关节状态广播器节点
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    ####################################################################################################################
    # 准备启动节点
    ####################################################################################################################
    nodes = [
        robot_state_pub_node,
        control_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(nodes)