cmake_minimum_required(VERSION 3.16)
project(cobot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObbDetection.msg"
  "msg/ObbDetectionArray.msg"
  "msg/Target.msg"
  "msg/TargetArray.msg"
  "msg/CobotStatus.msg"
  "msg/TaskStatus.msg"
  "msg/LayerReport.msg"
  "msg/KeyValue.msg"
  "msg/Heartbeat.msg"
  "msg/NodeHealth.msg"
  "msg/HealthSummary.msg"
  "msg/PickEvent.msg"
  "msg/Alarm.msg"
  "msg/CobotState.msg"
  "msg/JobResult.msg"
  "msg/SensorAssert.msg"
  "msg/SystemState.msg"
  "msg/WorkflowInfo.msg"
  "msg/WorkflowList.msg"
  "msg/ToolFile.msg"
  "msg/UserFrame.msg"
  "msg/ControllerProfile.msg"
  "msg/CalibrationMarker.msg"
  "srv/StartWorkflow.srv"
  "srv/VerifyTarget.srv"
  "srv/GetLayerHeight.srv"
  "srv/CalibrationSample.srv"
  "srv/CalibrationBegin.srv"
  "srv/SetOutputBit.srv"
  "srv/ReadIoBit.srv"
  "srv/MotionStop.srv"
  "srv/SetMotionProfile.srv"
  "srv/ReadIntVar.srv"
  "srv/WriteIntVar.srv"
  "srv/ReadPositionVar.srv"
  "srv/WritePositionVar.srv"
  "srv/GetCurrentPose.srv"
  "srv/GetActiveAlarms.srv"
  "srv/SetActiveTool.srv"
  "srv/ConvertPose.srv"
  "srv/ConvertTrajectories.srv"
  "srv/SaveTeachPoint.srv"
  "srv/SetTeachPoint.srv"
  "action/PickCan.action"
  "action/Hide.action"
  "action/MoveToPose.action"
  "action/Detect.action"
  "action/Scan.action"
  DEPENDENCIES std_msgs action_msgs
)

ament_package()
