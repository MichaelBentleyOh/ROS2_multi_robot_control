# simple_test/launch/single_robot.launch.py
import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ---------- Launch arguments ----------
    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock if true",
    )

    # ---------- Paths ----------
    pkg_share = FindPackageShare("simple_test").find("simple_test")
    world_file = os.path.join(pkg_share, "empty.sdf")
    urdf_file = os.path.join(pkg_share, "simple_diff_drive_robot2.urdf")

    # ---------- Robot description ----------
    robot_description = xacro.process_file(urdf_file).toxml()

    # ---------- Gazebo/IGN ----------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_ign_gazebo"),
                "launch",
                "ign_gazebo.launch.py",
            )
        ),
        launch_arguments={"ign_args": f"-r -v 0 {world_file}"}.items(),
    )

    # ---------- Spawn entity ----------
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",
            "simple_diff_drive_robot2",
            "-string",
            robot_description,
            "-allow_renaming",
            "true",
            "-x",
            "1",
            "-y",
            "0",
            "-z",
            "0.15",
        ],
    )

    # ---------- Robot-state-publisher ----------
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="robot2",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"publish_robot_description": True},  # latched topic 송출
            {"use_sim_time": use_sim_time},
        ],
    )

    # ---------- ros2_control controllers ----------
    # load_jsb = ExecuteProcess(
    #     cmd=[
    #         "ros2",
    #         "control",
    #         "load_controller",
    #         "-c",
    #         "/robot2/controller_manager",
    #         "--set-state",
    #         "active",
    #         "joint_state_broadcaster",
    #     ],
    #     output="screen",
    # )
    load_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/robot2/controller_manager"],
        output="screen",
    )

    # load_vel = ExecuteProcess(
    #     cmd=[
    #         "ros2",
    #         "control",
    #         "load_controller",
    #         "-c",
    #         "/robot2/controller_manager",
    #         "--set-state",
    #         "active",
    #         "velocity_controller",
    #     ],
    #     output="screen",
    # )
    load_vel = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller", "--controller-manager", "/robot2/controller_manager"],
        output="screen",
    )
    # Spawn 완료 후 컨트롤러 로드
    load_after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=[load_jsb, load_vel])
    )

    # ---------- LaunchDescription ----------
    return LaunchDescription(
        [
            declare_sim_time,
            # gazebo,
            spawn,
            rsp,
            load_after_spawn,
        ]
    )
