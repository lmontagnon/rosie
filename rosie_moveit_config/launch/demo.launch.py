from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="rosie", package_name="rosie_moveit_config"
        )
        .robot_description(file_path="config/rosie.urdf")
        .robot_description_semantic(file_path="config/rosie.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    return generate_demo_launch(moveit_config)
