# Author: Ludovic Montagnon

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    # Initialize Arguments
    sim_gazebo = LaunchConfiguration("sim_gazebo")
    sim_ignition = LaunchConfiguration("sim_ignition")
    robot_type = LaunchConfiguration("robot_type")
    dof = LaunchConfiguration("dof")
    vision = LaunchConfiguration("vision")
    # General arguments
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    robot_name = LaunchConfiguration("robot_name")
    prefix = LaunchConfiguration("prefix")
    robot_pos_controller = LaunchConfiguration("robot_pos_controller")
    robot_hand_controller = LaunchConfiguration("robot_hand_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    gripper = LaunchConfiguration("gripper")
    world = LaunchConfiguration("world")

    robot_controllers = PathJoinSubstitution(
        # https://answers.ros.org/question/397123/how-to-access-the-runtime-value-of-a-launchconfiguration-instance-within-custom-launch-code-injected-via-an-opaquefunction-in-ros2/
        [
            FindPackageShare(description_package),
            "config",
            controllers_file,
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "rviz_config.rviz"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "robot_ip:=xxx.yyy.zzz.www",
            " ",
            "name:=",
            robot_name,
            " ",
            "arm:=",
            robot_type,
            " ",
            "dof:=",
            dof,
            " ",
            "vision:=",
            vision,
            " ",
            "prefix:=",
            prefix,
            " ",
            "sim_gazebo:=",
            sim_gazebo,
            " ",
            "sim_ignition:=",
            sim_ignition,
            " ",
            "simulation_controllers:=",
            robot_controllers,
            " ",
            "gripper:=",
            gripper,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content.perform(context)}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    left_robot_traj_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_joint_trajectory_controller", "-c", "/controller_manager"],
    )
    
    right_robot_traj_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_joint_trajectory_controller", "-c", "/controller_manager"],
    )

    both_arms_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["both_arms_joint_trajectory_controller", "--inactive", "-c", "/controller_manager"],
    )

    robot_pos_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_pos_controller, "--inactive", "-c", "/controller_manager"],
    )

    robot_model = robot_type.perform(context)
    is_gen3_lite = "false"
    if robot_model == "gen3_lite":
        is_gen3_lite = "true"

    left_robot_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_robotiq_gripper_controller", "-c", "/controller_manager"],
        condition=UnlessCondition(is_gen3_lite),
    )
    
    right_robot_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_robotiq_gripper_controller", "-c", "/controller_manager"],
        condition=UnlessCondition(is_gen3_lite),
    )

    # --- clock bridge -------------------------------------------------
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/world/rosie_world/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock",
            "--ros-args", "-r", "/world/rosie_world/clock:=/clock",
        ],
        output="screen",
    )

    # Gazebo nodes
    gzserver = ExecuteProcess(
        cmd=["gzserver", "-s", "libgazebo_ros_init.so", "-s", "libgazebo_ros_factory.so", ""],
        output="screen",
        condition=IfCondition(sim_gazebo),
    )

    # Gazebo client
    gzclient = ExecuteProcess(
        cmd=["gzclient"],
        output="screen",
        condition=IfCondition(sim_gazebo),
    )

    # gazebo = IncludeLaunchDescription(
    # PythonLaunchDescriptionSource(
    # [PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])]
    # ),
    # launch_arguments={"verbose": "false"}.items(),
    # )

    # Spawn robot
    gazebo_spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_robot",
        arguments=["-entity", robot_name, "-topic", "robot_description"],
        output="screen",
        condition=IfCondition(sim_gazebo),
    )

    ignition_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content,
            "-name",
            robot_name,
            "-allow_renaming",
            "true",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.3",
            "-R",
            "0.0",
            "-P",
            "0.0",
            "-Y",
            "0.0",
        ],
        condition=IfCondition(sim_ignition),
    )


    ignition_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={
            "gz_args": PythonExpression([
                 '"-r -v 3 " + "', LaunchConfiguration("world"), '"'
            ])
        }.items(),
        condition=IfCondition(sim_ignition),
    )

    # --- sensor bridges ----------------------------------------------
    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
        	"/wrist_mounted_camera/image@ignition.msgs.Image@sensor_msgs/msg/Image",
        	"/wrist_mounted_camera/depth_image@ignition.msgs.Image@sensor_msgs/msg/Image",
        	"/wrist_mounted_camera/points@ignition.msgs.PointCloudPacked@sensor_msgs/msg/PointCloud2",
        	"/wrist_mounted_camera/camera_info@ignition.msgs.CameraInfo@sensor_msgs/msg/CameraInfo",
    ],
        output="screen",
    )

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="rosie", package_name="rosie_moveit_config"
        )
        .robot_description(file_path=get_package_share_directory("rosie_description")
            + "/urdf/rosie.urdf.xacro")
        .robot_description_semantic(file_path=get_package_share_directory("rosie_moveit_config")
            + "/config/rosie.srdf")
        .trajectory_execution(file_path=get_package_share_directory("rosie_moveit_config")
            + "/config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .moveit_cpp(
            file_path=get_package_share_directory("moveit2_tutorials")
            + "/config/dual_motion_planning_python_api_tutorial.yaml"
        )
        .to_moveit_configs()
    )

    moveit_py_node = Node(
        name="moveit_py",
        package="rosie_bringup",
        executable=LaunchConfiguration("example_file"),
        output="both",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time}
        ],
    )

    nodes_to_start = [
        bridge,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        left_robot_traj_controller_spawner,
        right_robot_traj_controller_spawner,
        both_arms_controller,
        robot_pos_controller_spawner,
        left_robot_hand_controller_spawner,
        right_robot_hand_controller_spawner,
        gzserver,
        gzclient,
        gazebo_spawn_robot,
        ignition_launch_description,
        ignition_spawn_entity,
        gazebo_bridge,
        #moveit_py_node
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    # Simulation specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_ignition",
            default_value="true",
            description="Use Ignition for simulation",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_gazebo",
            default_value="false",
            description="Use Gazebo Classic for simulation",
        )
    )
    # Robot specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_type",
            description="Type/series of robot.",
            choices=["gen3", "gen3_lite"],
            default_value="gen3",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "dof",
            description="DoF of robot.",
            choices=["6", "7"],
            default_value="7",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "vision",
            description="Use arm mounted realsense",
            choices=["true", "false"],
            default_value="true",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="ros2_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="rosie_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="rosie.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_name",
            default_value="rosie",
            description="Robot name.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_pos_controller",
            default_value="twist_controller",
            description="Robot controller to start.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulated clock",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper",
            default_value="robotiq_2f_85",
            choices=["robotiq_2f_85", "robotiq_2f_140", "gen3_lite_2f", ""],
            description="Gripper to use",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="false", description="Launch RViz?")
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution(
                [FindPackageShare("rosie_bringup"), "worlds", "rosie_world.sdf"]
            ),
            description="Chemin absolu du monde Ignition à charger",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "example_file",
            default_value="dual_motion_planning_python_api_tutorial.py",
            description="Python API tutorial file name",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
