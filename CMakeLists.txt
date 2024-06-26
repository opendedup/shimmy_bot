cmake_minimum_required(VERSION 3.8)
project(shimmy_bot)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY description config launch scripts
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/nav_to_pose.py
  scripts/robot_navigator.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
