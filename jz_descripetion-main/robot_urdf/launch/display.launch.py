#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("jz_robot_description")

    default_urdf_path = PathJoinSubstitution(
        [
            package_share,
            "urdf",
            "robot urdf.10.8.SLDASM.urdf",
        ]
    )
    default_rviz_config = PathJoinSubstitution(
        [
            package_share,
            "config",
            "view_urdf.rviz",
        ]
    )

    robot_namespace_arg = DeclareLaunchArgument(
        "robot_namespace",
        default_value="robot1",
        description="Robot namespace used by live joint state topics, for example robot1.",
    )
    urdf_path_arg = DeclareLaunchArgument(
        "urdf_path",
        default_value=default_urdf_path,
        description="Absolute path to the robot URDF file.",
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz_config,
        description="Absolute path to the RViz2 config file.",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Start RViz2 when true.",
    )
    use_joint_bridge_arg = DeclareLaunchArgument(
        "use_joint_bridge",
        default_value="true",
        description="Start the live robot joint-state-to-URDF bridge when true.",
    )
    use_joint_state_gui_arg = DeclareLaunchArgument(
        "use_joint_state_gui",
        default_value="true",
        description="Start a joint-state GUI that publishes /joint_states when true.",
    )
    joint_state_gui_mode_arg = DeclareLaunchArgument(
        "joint_state_gui_mode",
        default_value="live",
        description="Joint-state GUI mode: live subscribes to robot state, manual enables sliders.",
    )
    joint_bridge_script_arg = DeclareLaunchArgument(
        "joint_bridge_script",
        default_value="/home/test/urdf_joint_state_bridge.py",
        description="Path to the Python joint state bridge script.",
    )
    joint_state_gui_script_arg = DeclareLaunchArgument(
        "joint_state_gui_script",
        default_value=PathJoinSubstitution(
            [
                package_share,
                "scripts",
                "simple_joint_state_gui.py",
            ]
        ),
        description="Path to the local joint state slider GUI script.",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        arguments=[LaunchConfiguration("urdf_path")],
    )

    joint_state_bridge = ExecuteProcess(
        cmd=["/usr/bin/python3", LaunchConfiguration("joint_bridge_script")],
        name="urdf_joint_state_bridge_process",
        output="screen",
        additional_env={
            "ROBOT_NS": LaunchConfiguration("robot_namespace"),
        },
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("use_joint_bridge"),
                    "' == 'true' and '",
                    LaunchConfiguration("use_joint_state_gui"),
                    "' != 'true'",
                ]
            )
        ),
    )

    joint_state_gui = ExecuteProcess(
        cmd=[
            "/usr/bin/python3",
            LaunchConfiguration("joint_state_gui_script"),
            "--urdf",
            LaunchConfiguration("urdf_path"),
            "--robot-namespace",
            LaunchConfiguration("robot_namespace"),
            "--mode",
            LaunchConfiguration("joint_state_gui_mode"),
        ],
        name="simple_joint_state_gui_process",
        output="screen",
        additional_env={
            "ROBOT_NS": LaunchConfiguration("robot_namespace"),
        },
        condition=IfCondition(LaunchConfiguration("use_joint_state_gui")),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(
        [
            robot_namespace_arg,
            urdf_path_arg,
            rviz_config_arg,
            use_rviz_arg,
            use_joint_bridge_arg,
            use_joint_state_gui_arg,
            joint_state_gui_mode_arg,
            joint_bridge_script_arg,
            joint_state_gui_script_arg,
            joint_state_bridge,
            joint_state_gui,
            robot_state_publisher,
            rviz,
        ]
    )
