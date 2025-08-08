from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("rosie")
        .robot_description(file_path="config/rosie.urdf")
        .robot_description_semantic(file_path="config/rosie.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    return LaunchDescription(
        [
            Node(
                package="rosie_moveit_cpp_demos",
                executable="rosie_motion_planning_api_tutorial",
                name="rosie_motion_planning_api_tutorial",
                output="screen",
                parameters=[moveit_config.to_dict()],
            )
        ]
    )