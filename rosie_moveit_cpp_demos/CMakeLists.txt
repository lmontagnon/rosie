cmake_minimum_required(VERSION 3.8)
project(rosie_moveit_cpp_demos)

# --- Flags / standard ---
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# MoveGroupInterface (classique)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)

# MoveItCpp (nécessaires pour dual_arms_planner_moveitcpp.cpp)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(visualization_msgs REQUIRED)

# --- Executables ---
# add_executable(dual_arms_planner_movegroup
#   src/dual_arms_planner_movegroup.cpp)
# ament_target_dependencies(dual_arms_planner_movegroup
#   rclcpp
#   moveit_ros_planning_interface
#   moveit_msgs
#   geometry_msgs
#   tf2_geometry_msgs
#   moveit_visual_tools
# )

add_executable(dual_arms_planner_movegroup_launch
  src/dual_arms_planner_movegroup_launch.cpp)
ament_target_dependencies(dual_arms_planner_movegroup_launch
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  tf2_geometry_msgs
)

# add_executable(dual_arms_planner_moveitcpp
#   src/dual_arms_planner_moveitcpp.cpp)
# ament_target_dependencies(dual_arms_planner_moveitcpp
#   rclcpp
#   moveit_core
#   moveit_ros_planning
#   moveit_ros_planning_interface
#   geometry_msgs
#   visualization_msgs
#   tf2_geometry_msgs
# )

# add_executable(rosie_motion_planning_api_tutorial
#   src/rosie_motion_planning_api_tutorial.cpp)
# ament_target_dependencies(rosie_motion_planning_api_tutorial
#   rclcpp
#   moveit_core
#   moveit_ros_planning
#   moveit_ros_planning_interface
#   geometry_msgs
#   visualization_msgs
#   tf2_geometry_msgs
#   moveit_visual_tools
# )

# --- Install ---
install(TARGETS
  # dual_arms_planner_movegroup
  dual_arms_planner_movegroup_launch
  # dual_arms_planner_moveitcpp
  #rosie_motion_planning_api_tutorial
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
