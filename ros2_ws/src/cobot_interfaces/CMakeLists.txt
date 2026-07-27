cmake_minimum_required(VERSION 3.16)
project(cobot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObbDetection.msg"
  "msg/ObbDetectionArray.msg"
  "msg/CanTarget.msg"
  "msg/CanTargetArray.msg"
  "msg/CobotStatus.msg"
  "msg/TaskStatus.msg"
  "msg/LayerReport.msg"
  "srv/VerifyTarget.srv"
  "srv/GetLayerHeight.srv"
  "srv/LocateDot.srv"
  "srv/CalibrationSample.srv"
  "action/PickCan.action"
  "action/MoveToPose.action"
  DEPENDENCIES std_msgs action_msgs
)

ament_package()
