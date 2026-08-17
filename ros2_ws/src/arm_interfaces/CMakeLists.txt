cmake_minimum_required(VERSION 3.8)
project(arm_interfaces)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CameraFrame.msg"
  "srv/PlanPath.srv"
  "srv/Grasp.srv"
  "action/PickAndPlace.action"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs trajectory_msgs
)

ament_package()
