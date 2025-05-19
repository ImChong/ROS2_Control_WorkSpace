from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable,PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("ros2_control_demo_example_1_hardware"),
                    "description",
                    "urdf",
                    "rrbot.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ros2_control_demo_example_1_hardware"), "description", "rviz", "rrbot.rviz"]
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("ros2_control_demo_example_1_hardware"),
            "controller",
            "config",
            "rrbot_controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--controller-manager", "/controller_manager"],
    )

    # 启动顺序：先启动控制器，再启动joint_state_broadcaster，最后启动rviz
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # 修改为先启动robot_controller_spawner，然后再启动joint_state_broadcaster_spawner
    # 从节点列表中移除joint_state_broadcaster_spawner，改为通过事件处理器触发
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        robot_controller_spawner,  # 首先启动robot_controller_spawner
        delay_joint_state_broadcaster_after_robot_controller_spawner,  # 然后通过事件处理器启动joint_state_broadcaster_spawner
        delay_rviz_after_joint_state_broadcaster_spawner,  # 最后通过事件处理器启动rviz
    ]

    return LaunchDescription(nodes)