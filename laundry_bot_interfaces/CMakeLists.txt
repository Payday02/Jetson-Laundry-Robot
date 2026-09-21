cmake_minimum_required(VERSION 3.8)
project(laundry_bot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AxisHome.srv"
  "srv/EnableMotors.srv"
  "srv/ReadEncoder.srv"
  "srv/EmergencyStop.srv"
  "srv/SetCurrent.srv"
  "srv/StartEpisode.srv"
  "srv/StopEpisode.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
