import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import TimerAction

def spawner(name, delay):
    return TimerAction(period=delay, actions=[
        Node(package="controller_manager", executable="spawner",
             arguments=[name], output="screen")
    ])

def launch_setup(context, *args, **kwargs):

    launch_rviz = LaunchConfiguration("launch_rviz")

    moveit_config = (
        MoveItConfigsBuilder("rosie")
        .robot_description(file_path="config/rosie_rev_isaacsim.urdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )
    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # RViz
    rviz_config_file = (
        get_package_share_directory("rosie_moveit_config") + "/config/moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "arm_world", "leftend_effector_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("rosie_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="both",
    )

    nodes_to_start = [
        ros2_control_node,
        robot_state_publisher,
        move_group_node,
        static_tf,
    ]

    return nodes_to_start


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )

    # Load controllers with timer to avoid too many participant in same time on ROS domain ID
    load_controllers = [
        spawner("joint_state_broadcaster", 0.0),
        spawner("left_joint_trajectory_controller", 0.5),
        spawner("right_joint_trajectory_controller", 1.0),
        spawner("left_robotiq_gripper_controller", 1.5),
        spawner("right_robotiq_gripper_controller", 2.0),
    ]

    return LaunchDescription(declared_arguments + load_controllers +[OpaqueFunction(function=launch_setup)])