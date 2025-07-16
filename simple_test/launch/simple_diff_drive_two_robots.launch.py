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
from launch.actions import TimerAction


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
    robot1_urdf_file = os.path.join(pkg_share, "simple_diff_drive_robot1.urdf")
    robot2_urdf_file = os.path.join(pkg_share, "simple_diff_drive_robot2.urdf")
    vel1_params = os.path.join(pkg_share, "robot_controllers_robot1.yaml")
    vel2_params = os.path.join(pkg_share, "robot_controllers_robot2.yaml")
    # ---------- Robot description ----------
    robot1_description = xacro.process_file(robot1_urdf_file).toxml()
    robot2_description = xacro.process_file(robot2_urdf_file).toxml()

    # ---------- Gazebo/IGN ----------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_ign_gazebo"),
                "launch",
                "ign_gazebo.launch.py",
            )
        ),
        launch_arguments={
                        "ign_args": f"-r -v 0 {world_file}"
                        }.items(),
    )

    # ---------- Spawn entity ----------
    robot1_spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",
            "simple_diff_drive_robot1",
            "-string",
            robot1_description,
            "-allow_renaming",
            "true",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0.15",
        ],
    )
    robot2_spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",
            "simple_diff_drive_robot2",
            "-string",
            robot2_description,
            "-allow_renaming",
            "true",
            "-x",
            "0",
            "-y",
            "2",
            "-z",
            "0.15",
        ],
    )

    # ---------- Robot-state-publisher ----------
    r1sp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="robot1",
        output="screen",
        parameters=[
            {"robot_description": robot1_description},
            {"publish_robot_description": True},
            {"use_sim_time": use_sim_time},
        ],
        # If you want to remap tf topics, uncomment the next line
        # It makes an unique tf topic for each robot
        # But there is a drawback that it can not be shown in rviz2
        # remappings=[('/tf', '/robot1/tf'), ('/tf_static', '/robot1/tf_static')]
    )
    r2sp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="robot2",
        output="screen",
        parameters=[
            {"robot_description": robot2_description},
            {"publish_robot_description": True},
            {"use_sim_time": use_sim_time},
        ],
        # If you want to remap tf topics, uncomment the next line
        # It makes an unique tf topic for each robot
        # But there is a drawback that it can not be shown in rviz2
        # remappings=[('/tf', '/robot2/tf'), ('/tf_static', '/robot2/tf_static')] 
    )

    # ---------- ros2_control controllers ----------
    load_j1sb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/robot1/controller_manager"],
        output="screen",
    )
    load_j2sb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/robot2/controller_manager"],
        output="screen",
    )

    load_vel1 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller", "-c", "/robot1/controller_manager", 
                   "--controller-manager-timeout", "1000000",
                   "-p", vel1_params],
        output="screen",
    )
    load_vel2 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller", "-c", "/robot2/controller_manager",
                   "--controller-manager-timeout", "1000000",
                   "-p", vel2_params],
        output="screen",
    )

    # --------------------Execution order-------------------------
    load_after_spawn1 = RegisterEventHandler(
        OnProcessExit(target_action=robot1_spawn, on_exit=[load_j1sb])
    )
    load_after_spawn2 = RegisterEventHandler(
        OnProcessExit(target_action=robot2_spawn, on_exit=[load_j2sb])
    )
    load_vel1_event = RegisterEventHandler(
        OnProcessExit(target_action=load_j1sb, on_exit=[load_vel1])
    )
    load_vel2_event = RegisterEventHandler(
        OnProcessExit(target_action=load_j2sb, on_exit=[load_vel2])
    )

    return LaunchDescription(
        [
            declare_sim_time,
            gazebo,
            r1sp,
            r2sp,
            robot1_spawn,
            robot2_spawn,

            load_after_spawn1,
            load_after_spawn2,
            
            load_vel1_event,
            TimerAction(period=6.0, actions=[
                load_vel2_event
            ]) # Delay loading of robot2 controller to ensure robot1 is ready(important for simulation stability)
        ]
    )