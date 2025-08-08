#!/usr/bin/env python3
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
 
    pkg_rosie_description = get_package_share_directory('rosie_description')

    # Acquiring robot description XACRO file
    xacro_file = os.path.join(pkg_rosie_description, 'urdf', 'rosie_nav.urdf.xacro')
    assert os.path.exists(xacro_file), "The rosie.xacro doesn't exist in " + str(xacro_file)
    default_rviz_config_path = os.path.join(pkg_rosie_description, 'rviz','rviz_config.rviz')

    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    model_arg = DeclareLaunchArgument(name='model', default_value=str(xacro_file),
                                      description='Absolute path to robot urdf file')


    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)
    # Your added nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )



    return LaunchDescription([
        gui_arg,
        model_arg,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        
    ])
