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
    xacro_file = os.path.join(pkg_rosie_description, 'urdf', 'rosie.urdf.xacro')
    assert os.path.exists(xacro_file), "The rosie.xacro doesn't exist in " + str(xacro_file)
    default_rviz_config_path = os.path.join(pkg_rosie_description, 'rviz','rviz_config.rviz')

   
    
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')

   
   
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return LaunchDescription([
        
     
        rviz_arg,
        
        rviz_node
    ])
