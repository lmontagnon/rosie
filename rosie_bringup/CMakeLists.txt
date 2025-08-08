cmake_minimum_required(VERSION 3.5)
project(rosie_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python executables
install(PROGRAMS
  scripts/test_huge_joint_space_jump.py
  scripts/dual_motion_planning_python_api_tutorial.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
