cmake_minimum_required(VERSION 3.16)
project(cobot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObbDetection.msg"
  "msg/ObbDetectionArray.msg"
  "msg/CanTarget.msg"
  "msg/CanTargetArray.msg"
  "msg/CobotStatus.msg"
  "msg/LayerReport.msg"
  "srv/VerifyTarget.srv"
  "srv/GetLayerHeight.srv"
  "srv/LocateDot.srv"
  "srv/CalibrationSample.srv"
  DEPENDENCIES std_msgs
)

ament_package()
