"""Bring up the ABB cell: ros2_control, controllers and, optionally, MoveIt and RViz.

    ros2 launch abb_cell_bringup bringup.launch.py hardware:=mock
    ros2 launch abb_cell_bringup bringup.launch.py hardware:=isaac moveit:=true
    ros2 launch abb_cell_bringup bringup.launch.py hardware:=real rws_ip:=192.168.125.1

Only the ros2_control hardware plugin changes between the three; the
controllers, MoveIt and any code that sends trajectories stay the same.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def launch_setup(context):
    hardware = context.launch_configurations["hardware"]
    rws_ip = context.launch_configurations["rws_ip"]
    use_moveit = context.launch_configurations["moveit"] == "true"
    use_rviz = context.launch_configurations["rviz"] == "true"
    # Isaac Sim publishes /clock; mock and real hardware run on wall time.
    use_sim_time = hardware == "isaac"

    xacro_file = os.path.join(
        get_package_share_directory("abb_cell_description"), "urdf", "abb_cell.urdf.xacro"
    )
    xacro_args = {"hardware": hardware, "rws_ip": rws_ip}
    robot_description = ParameterValue(
        Command(["xacro ", xacro_file] + [f" {k}:={v}" for k, v in xacro_args.items()]),
        value_type=str,
    )
    controllers = os.path.join(
        get_package_share_directory("abb_cell_bringup"), "config", "controllers.yaml"
    )

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
            output="screen",
        ),
        # The controller manager reads the URDF from /robot_description.
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[controllers, {"use_sim_time": use_sim_time}],
            remappings=[("~/robot_description", "/robot_description")],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "joint_trajectory_controller",
                       "--controller-manager", "/controller_manager"],
            output="screen",
        ),
    ]

    if use_moveit or use_rviz:
        from moveit_configs_utils import MoveItConfigsBuilder

        moveit_config = (
            MoveItConfigsBuilder("abb_irb1200_5_90", package_name="abb_irb1200_5_90_moveit_config")
            .robot_description(file_path=xacro_file, mappings=xacro_args)
            .robot_description_semantic(file_path="config/abb_irb1200_5_90.srdf.xacro")
            .trajectory_execution(file_path="config/moveit_controllers.yaml")
            .planning_pipelines(pipelines=["ompl"])
            .to_moveit_configs()
        )

    if use_moveit:
        nodes.append(
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                parameters=[moveit_config.to_dict(), {"use_sim_time": use_sim_time}],
                output="screen",
            )
        )

    if use_rviz:
        rviz_config = os.path.join(
            get_package_share_directory("abb_irb1200_5_90_moveit_config"), "rviz", "moveit.rviz"
        )
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.planning_pipelines,
                    {"use_sim_time": use_sim_time},
                ],
                output="log",
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "hardware", default_value="mock", choices=["mock", "isaac", "real"],
            description="ros2_control target: mock hardware, Isaac Sim or the real ABB controller.",
        ),
        DeclareLaunchArgument(
            "rws_ip", default_value="None",
            description="IP of the ABB controller (hardware:=real only).",
        ),
        DeclareLaunchArgument("moveit", default_value="false", choices=["true", "false"],
                              description="Start MoveIt's move_group."),
        DeclareLaunchArgument("rviz", default_value="false", choices=["true", "false"],
                              description="Start RViz with the MoveIt panel."),
        OpaqueFunction(function=launch_setup),
    ])
