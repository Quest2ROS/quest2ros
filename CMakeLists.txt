cmake_minimum_required(VERSION 3.8)
project(quest2ros)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/OVR2ROSInputs.msg"
  "msg/OVR2ROSHapticFeedback.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

install(PROGRAMS
  scripts/ros2quest.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
