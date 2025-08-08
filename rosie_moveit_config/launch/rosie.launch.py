from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from moveit_configs_utils import MoveItConfigsBuilder
from launch.conditions import IfCondition

def generate_launch_description():
    # --- Arguments optionnels ---
    use_rviz = LaunchConfiguration("use_rviz", default="true")

    # --- Construit la config MoveIt ---
    moveit_config = (
        MoveItConfigsBuilder(robot_name="rosie", package_name="rosie_moveit_config")
        .robot_description(file_path="config/rosie.urdf")
        .robot_description_semantic(file_path="config/rosie.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        # charge ce que tu veux comme pipelines
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # --- Paths utiles ---
    pkg_share = get_package_share_directory("rosie_moveit_config")
    ros2_control_yaml = PathJoinSubstitution([pkg_share, "config", "ros2_controllers.yaml"])

    # --- robot_state_publisher ---
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],  # robot_description
    )

    # --- ros2_control_node ---
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,               # robot_description
            {"use_sim_time": False},
            ros2_control_yaml
        ],
        output="screen",
    )

    # --- Spawner unique pour TOUS les contrôleurs ---
    controllers = [
        "joint_state_broadcaster",
        "left_joint_trajectory_controller",
        "right_joint_trajectory_controller",
        "left_robotiq_gripper_controller",
        "right_robotiq_gripper_controller",
    ]
    spawner_all = Node(
        package="controller_manager",
        executable="spawner",
        arguments=controllers,
        output="screen",
    )

    # --- move_group ---
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),  # robot_description, semantic, kinematics, pipelines, trajectory_execution, controllers, ...
            {"use_sim_time": False},
            # tu peux aussi forcer ici le pipeline par défaut :
            # {"planning_pipelines": ["ompl"]},
        ],
    )

    # --- RViz (optionnel) ---
    rviz_config = PathJoinSubstitution([pkg_share, "config", "moveit.rviz"])
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[moveit_config.robot_description, moveit_config.robot_description_semantic],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            robot_state_pub,
            ros2_control_node,
            spawner_all,
            move_group_node,
            rviz_node,
        ]
    )
