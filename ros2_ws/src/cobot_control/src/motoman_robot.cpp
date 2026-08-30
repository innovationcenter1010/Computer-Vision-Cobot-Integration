#include "cobot_control/motoman_robot.hpp"

#include <algorithm>
#include <iomanip>
#include <sstream>

namespace cobot_control
{

std::string RobotStatus::text() const
{
  if (ok()) {return message;}
  return "[" + std::to_string(code) + "] " + message;
}

std::string Pose::str() const
{
  std::ostringstream o;
  o << std::fixed << std::setprecision(1)
    << "(" << x << ", " << y << ", " << z << ", " << rx << ", " << ry << ", " << rz << ")";
  return o.str();
}

namespace
{

IOType toIoType(IoKind k)
{
  return k == IoKind::GeneralInput ? IOType::GeneralInput : IOType::GeneralOutput;
}

PositionData toPositionData(const Pose & p, int32_t tool_number)
{
  PositionData d{};
  d.coordinateType = CoordinateType::BaseCoordinate;
  d.toolNumber = static_cast<UINT32>(tool_number < 0 ? 0 : tool_number);
  d.userCoordinateNumber = 0;
  // AxisIndex::CartesianAxis: X=0, Y, Z, Rx, Ry, Rz
  d.axisData[0] = p.x;
  d.axisData[1] = p.y;
  d.axisData[2] = p.z;
  d.axisData[3] = p.rx;
  d.axisData[4] = p.ry;
  d.axisData[5] = p.rz;
  return d;
}

Pose fromPositionData(const PositionData & d)
{
  Pose p;
  p.x = d.axisData[0];
  p.y = d.axisData[1];
  p.z = d.axisData[2];
  p.rx = d.axisData[3];
  p.ry = d.axisData[4];
  p.rz = d.axisData[5];
  return p;
}

}  // namespace

MotomanRobot::MotomanRobot(std::string ip, ConnectOptions opts)
: ip_(std::move(ip)), opts_(opts) {}

MotomanRobot::~MotomanRobot()
{
  disconnect(connected_.load());
}

template<class F>
RobotStatus MotomanRobot::withController(F && f)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (!controller_) {return RobotStatus::notConnected();}
  return f(*controller_);
}

// ---------------------------------------------------------------- connection

RobotStatus MotomanRobot::connect()
{
  StatusInfo status{};
  // Nothing else can use a null controller, so OpenConnection (which may
  // block for timeout_ms * retries) runs outside the lock.
  MotomanController * ctrl = YMConnect::OpenConnection(
    ip_, status, opts_.timeout_ms, opts_.retries, opts_.logging);

  if (!ctrl || !status.IsOk()) {
    if (ctrl) {YMConnect::CloseConnection(ctrl);}
    if (status.IsOk()) {return {1, "OpenConnection returned no controller"};}
    return RobotStatus::from(status);
  }

  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (controller_) {YMConnect::CloseConnection(controller_);}
    controller_ = ctrl;
  }

  RobotStatus st = withController(
    [](MotomanController & c) {
      return RobotStatus::from(c.ControlCommands->SetServos(SignalStatus::ON));
    });
  if (!st.ok()) {
    disconnect(false);
    return {st.code, "SetServos(ON): " + st.message};
  }

  // A killed session can leave points queued in the controller; the next
  // MotionStart would run them before ours. Best-effort.
  RobotStatus cleared = withController(
    [](MotomanController & c) {
      return RobotStatus::from(c.MotionManager->ClearAllTrajectory());
    });

  connected_ = true;
  if (cleared.ok()) {return RobotStatus::okay("connected, trajectory cleared");}
  return RobotStatus::okay("connected (clear trajectory failed: " + cleared.text() + ")");
}

RobotStatus MotomanRobot::disconnect(bool servos_off)
{
  std::lock_guard<std::mutex> lock(mtx_);
  connected_ = false;
  if (!controller_) {return RobotStatus::okay("already disconnected");}

  std::string note = "disconnected";
  if (servos_off) {
    StatusInfo st = controller_->ControlCommands->SetServos(SignalStatus::OFF);
    if (!st.IsOk()) {note += " (servo off failed: " + st.Message + ")";}
  }
  YMConnect::CloseConnection(controller_);
  controller_ = nullptr;
  return RobotStatus::okay(note);
}

// ---------------------------------------------------------------- state / IO

RobotStatus MotomanRobot::readState(ControllerState & out)
{
  return withController(
    [&](MotomanController & c) {
      ControllerStateData d{};
      StatusInfo st = c.Status->ReadState(d);
      if (!st.IsOk()) {return RobotStatus::from(st);}
      out.servo_on = d.isServoOn;
      out.running = d.isRunning;
      out.in_hold = d.isInHold;
      out.alarming = d.isAlarming;
      out.erroring = d.isErroring;
      out.control_mode = static_cast<int>(d.controlMode);
      return RobotStatus::okay();
    });
}

RobotStatus MotomanRobot::readIoByte(IoKind kind, uint16_t group, uint8_t & out)
{
  return withController(
    [&](MotomanController & c) {
      UINT8 v = 0;
      StatusInfo st = c.Io->ReadByte(toIoType(kind), group, v);
      out = v;
      return RobotStatus::from(st);
    });
}

RobotStatus MotomanRobot::readIoBit(IoKind kind, uint16_t group, uint8_t bit, bool & out)
{
  return withController(
    [&](MotomanController & c) {
      bool v = false;
      StatusInfo st = c.Io->ReadBit(toIoType(kind), group, bit, v);
      out = v;
      return RobotStatus::from(st);
    });
}

RobotStatus MotomanRobot::writeOutputBit(uint16_t group, uint8_t bit, bool value)
{
  return withController(
    [&](MotomanController & c) {
      return RobotStatus::from(c.Io->WriteBit(IOType::GeneralOutput, group, bit, value));
    });
}

RobotStatus MotomanRobot::readCurrentPose(int32_t tool_number, Pose & out)
{
  return withController(
    [&](MotomanController & c) {
      PositionData d{};
      StatusInfo st = c.ControlGroup->ReadPositionData(
        ControlGroupId::R1, CoordinateType::BaseCoordinate, 0,
        static_cast<UINT8>(tool_number < 0 ? 0 : tool_number), d);
      if (!st.IsOk()) {return RobotStatus::from(st);}
      out = fromPositionData(d);
      return RobotStatus::okay();
    });
}

RobotStatus MotomanRobot::activeAlarms(std::vector<Alarm> & out)
{
  return withController(
    [&](MotomanController & c) {
      ActiveAlarms a{};
      StatusInfo st = c.Faults->GetActiveAlarms(a);
      if (!st.IsOk()) {return RobotStatus::from(st);}
      out.clear();
      const size_t n = std::min<size_t>(a.numberOfActiveAlarms, a.alarms.size());
      for (size_t i = 0; i < n; ++i) {
        out.push_back(
          {a.alarms[i].code, a.alarms[i].subcode, a.alarms[i].name, a.alarms[i].time});
      }
      return RobotStatus::okay();
    });
}

RobotStatus MotomanRobot::clearFaults()
{
  return withController(
    [](MotomanController & c) {return RobotStatus::from(c.Faults->ClearAllFaults());});
}

RobotStatus MotomanRobot::setServos(bool on)
{
  return withController(
    [&](MotomanController & c) {
      return RobotStatus::from(
        c.ControlCommands->SetServos(on ? SignalStatus::ON : SignalStatus::OFF));
    });
}

RobotStatus MotomanRobot::setHold(bool on)
{
  return withController(
    [&](MotomanController & c) {
      return RobotStatus::from(
        c.ControlCommands->SetHold(on ? SignalStatus::ON : SignalStatus::OFF));
    });
}

RobotStatus MotomanRobot::pendantMessage(const std::string & text)
{
  return withController(
    [&](MotomanController & c) {
      return RobotStatus::from(c.ControlCommands->DisplayStringToPendant(text.substr(0, 30)));
    });
}

// ---------------------------------------------------------------- motion

RobotStatus MotomanRobot::addPoint(const Pose & pose, const MotionProfile & profile)
{
  return withController(
    [&](MotomanController & c) {
      const PositionData target = toPositionData(pose, profile.tool_number);
      MotionAccelDecel attrs{};
      attrs.accelRatio = profile.accel_ratio;
      attrs.decelRatio = profile.decel_ratio;

      // The SDK derived structs set interpolationType themselves and the
      // speed unit follows: JointMotion = VJ (percent), LinearMotion = V (mm/s).
      if (profile.interpolation == Interp::Linear) {
        LinearMotion m(
          ControlGroupId::R1, target, profile.speed, SpeedType::V, profile.accuracy_mm, attrs);
        return RobotStatus::from(c.MotionManager->AddPointToTrajectory(m));
      }
      JointMotion m(
        ControlGroupId::R1, target, profile.speed, SpeedType::VJ, profile.accuracy_mm, attrs);
      return RobotStatus::from(c.MotionManager->AddPointToTrajectory(m));
    });
}

RobotStatus MotomanRobot::motionStart()
{
  return withController(
    [](MotomanController & c) {return RobotStatus::from(c.MotionManager->MotionStart());});
}

RobotStatus MotomanRobot::motionStop(bool clear_trajectory)
{
  return withController(
    [&](MotomanController & c) {
      return RobotStatus::from(c.MotionManager->MotionStop(clear_trajectory));
    });
}

RobotStatus MotomanRobot::clearTrajectory()
{
  return withController(
    [](MotomanController & c) {
      return RobotStatus::from(c.MotionManager->ClearAllTrajectory());
    });
}

RobotStatus MotomanRobot::motionProgress(MotionProgress & out)
{
  return withController(
    [&](MotomanController & c) {
      StatusInfo st{};
      const DOUBLE64 pct = c.MotionManager->GetMotionTargetProgress(ControlGroupId::R1, st);
      if (!st.IsOk()) {return RobotStatus::from(st);}
      INT32 steps = 0;
      st = c.MotionManager->GetRemainingStepsInTrajectory(steps);
      if (!st.IsOk()) {return RobotStatus::from(st);}
      out.percent = pct;
      out.steps_remaining = steps;
      // Same rule the pybind wrapper used (HasReachedTarget).
      out.reached = (pct >= 99.0 && steps <= 1) || steps == 0;
      return RobotStatus::okay();
    });
}

RobotStatus MotomanRobot::isReachable(
  const Pose & pose, int32_t tool_number, bool & reachable, std::string & why)
{
  return withController(
    [&](MotomanController & c) {
      PositionData current{};
      StatusInfo st = c.ControlGroup->ReadPositionData(
        ControlGroupId::R1, CoordinateType::Pulse, 0, 0, current);
      if (!st.IsOk()) {return RobotStatus::from(st);}

      PositionData target = toPositionData(pose, tool_number);
      PositionData joints{};
      st = c.Kinematics->ConvertPositionFromCartesian(
        ControlGroupId::R1, target, KinematicConversions::CartesianPosToJointAngle,
        KinematicType::Delta, current.axisData, joints);
      if (!st.IsOk()) {
        reachable = false;
        why = "IK failed: [" + std::to_string(st.StatusCode) + "] " + st.Message;
        return RobotStatus::okay();
      }
      reachable = true;
      why.clear();
      return RobotStatus::okay();
    });
}

// ---------------------------------------------------------------- variables

RobotStatus MotomanRobot::readInt(uint16_t index, int32_t & out)
{
  return withController(
    [&](MotomanController & c) {
      IntegerVariableData d{};
      StatusInfo st = c.Variables->IntegerVariable->Read(index, d);
      if (!st.IsOk()) {return RobotStatus::from(st);}
      out = d.value;
      return RobotStatus::okay();
    });
}

RobotStatus MotomanRobot::writeInt(uint16_t index, int32_t value)
{
  return withController(
    [&](MotomanController & c) {
      IntegerVariableData d(static_cast<INT16>(value), index);
      return RobotStatus::from(c.Variables->IntegerVariable->Write(d));
    });
}

RobotStatus MotomanRobot::readPositionVar(uint16_t index, Pose & out)
{
  return withController(
    [&](MotomanController & c) {
      RobotPositionVariableData d{};
      StatusInfo st = c.Variables->RobotPositionVariable->Read(index, d);
      if (!st.IsOk()) {return RobotStatus::from(st);}
      out = fromPositionData(d.positionData);
      return RobotStatus::okay();
    });
}

RobotStatus MotomanRobot::writePositionVar(uint16_t index, const Pose & pose)
{
  return withController(
    [&](MotomanController & c) {
      RobotPositionVariableData d{};
      d.variableIndex = index;
      d.positionData = toPositionData(pose, 0);
      return RobotStatus::from(c.Variables->RobotPositionVariable->Write(d));
    });
}

}  // namespace cobot_control
