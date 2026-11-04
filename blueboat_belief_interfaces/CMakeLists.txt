cmake_minimum_required(VERSION 3.10)
project(blueboat_belief_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BeliefContact.msg"
  "msg/BeliefGrid.msg"
  "srv/QueryBelief.srv"
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_package()
