#include "MotomanRobot.hpp"
#include <stdexcept>
#include "YMConnect.h"
#include <thread>
#include <chrono>
#include <future>
//#include <pybind11/pybind11.h>
//#include <pybind11/stl.h>


using namespace std::chrono_literals;

const std::chrono::milliseconds MOTION_POLLING_INTERVAL = 50ms;

MotomanRobot::MotomanRobot(const std::string& ip)
    : ip(ip), controller(nullptr), connected(false)
{
}

MotomanRobot::~MotomanRobot()
{
    disconnect();
}

void MotomanRobot::connect()
{
    StatusInfo status{};

    controller = YMConnect::OpenConnection(ip, status);

    if (!controller || !status.IsOk())
    {
        throw std::runtime_error("Connect failed: " + status.Message);
    }

    connected = true;
}

void MotomanRobot::disconnect()
{
    if (controller)
    {
        YMConnect::CloseConnection(controller);
        controller = nullptr;
    }

    connected = false;
}


// -------------------- SETUP --------------------

void MotomanRobot::set_motion_profile(DOUBLE64 speed, DOUBLE64 accel, DOUBLE64 decel)
{
    // This function can be used to set up motion profiles for the robot. For simplicity, we will use the same profile for all motions in this example.
    // In a more complex implementation, you could allow different profiles for different motions or groups.

    robot_motion_attributes.accelRatio = accel;
    robot_motion_attributes.decelRatio = decel;
    robot_speed = speed;
}



// -------------------- INTEGER --------------------

int MotomanRobot::read_int(uint16_t index)
{
    if (!connected) throw std::runtime_error("Not connected");

    IntegerVariableData data{};
    StatusInfo st = controller->Variables->IntegerVariable->Read(index, data);

    if (!st.IsOk())
        throw std::runtime_error(st.Message);

    return data.value;
}

void MotomanRobot::write_int(uint16_t index, int value)
{
    if (!connected) throw std::runtime_error("Not connected");

    IntegerVariableData data(value, index);

    StatusInfo st = controller->Variables->IntegerVariable->Write(data);

    if (!st.IsOk())
        throw std::runtime_error(st.Message);
}

// -------------------- POSITION --------------------

Position MotomanRobot::read_position(uint16_t index)
{
    if (!connected) throw std::runtime_error("Not connected");

    RobotPositionVariableData data{};

    StatusInfo st =
        controller->Variables->RobotPositionVariable->Read(index, data);

    if (!st.IsOk())
        throw std::runtime_error(st.Message);

    Position p;

    // YMConnect mapping: axisData[0]=X, [1]=Y, [2]=Z
    p.x = data.positionData.axisData[0];
    p.y = data.positionData.axisData[1];
    p.z = data.positionData.axisData[2];
    p.rx = data.positionData.axisData[3];
    p.ry = data.positionData.axisData[4];   
    p.rz = data.positionData.axisData[5];

    return p;
}

void MotomanRobot::write_position(uint16_t index, const Position& pos)
{
    if (!connected) throw std::runtime_error("Not connected");

    RobotPositionVariableData data{};
    data.variableIndex = index;

    data.positionData.axisData[0] = pos.x;
    data.positionData.axisData[1] = pos.y;
    data.positionData.axisData[2] = pos.z;
    data.positionData.axisData[3] = pos.rx;
    data.positionData.axisData[4] = pos.ry;
    data.positionData.axisData[5] = pos.rz;

    StatusInfo st =
        controller->Variables->RobotPositionVariable->Write(data);

    if (!st.IsOk())
        throw std::runtime_error(st.Message);
}

// -------------------- MOTION --------------------

void MotomanRobot::motion_start()
{
    if (!connected)
        throw std::runtime_error("Not connected");

    StatusInfo st = controller->MotionManager->MotionStart();

    if (!st.IsOk())
        throw std::runtime_error("MotionStart failed: " + st.Message);
}

void MotomanRobot::motion_stop(bool clearTrajectory)
{
    if (!connected)
        throw std::runtime_error("Not connected");

    StatusInfo st =
        controller->MotionManager->MotionStop(clearTrajectory);

    if (!st.IsOk())
        throw std::runtime_error("MotionStop failed: " + st.Message);
}

void MotomanRobot::clear_trajectory()
{
    if (!connected)
        //throw std::runtime_error("Not connected");

    StatusInfo st =
        controller->MotionManager->ClearAllTrajectory();

    //if (!st.IsOk())
      //  throw std::runtime_error("ClearAllTrajectory failed: " + st.Message);
}

// -------------------- TRAJECTORY --------------------

void MotomanRobot::add_point_to_trajectory(const Position& pos)
{
    if (!connected)
        throw std::runtime_error("Not connected");

    std::cout << "Adding point to trajectory: (" << pos.x << ", " << pos.y << ", " << pos.z << ", "
              << pos.rx << ", " << pos.ry << ", " << pos.rz << ")" << std::endl;

    Motion motion;

    // ---------------- Core configuration ----------------
    motion.groupId = ControlGroupId::R1; // R1 by default

    motion.interpolationType = robot_interpolation_type; // MoveJoint by default, can be changed to MoveLinear or MoveCircular as needed

    motion.position.coordinateType = robot_coordinate_type;

    // ---------------- Position mapping ----------------
    motion.position.axisData[0] = pos.x;
    motion.position.axisData[1] = pos.y;
    motion.position.axisData[2] = pos.z;
    motion.position.axisData[3] = pos.rx;
    motion.position.axisData[4] = pos.ry;
    motion.position.axisData[5] = pos.rz;

    // ---------------- Motion defaults ----------------
    motion.speed = robot_speed; // or expose later as parameter
    motion.speedType = SpeedType::VJ; // safe default for joint motion style

    motion.attributes = robot_motion_attributes;

    motion.accuracy = -1;

    // ---------------- Execute ----------------
    StatusInfo st =
        controller->MotionManager->AddPointToTrajectory(motion);

    if (!st.IsOk())
    {
        throw std::runtime_error(
            "AddPointToTrajectory failed: " + st.Message);
    }
}

double MotomanRobot::get_motion_progress()
{

    if (!connected)
        throw std::runtime_error("Not connected");

    StatusInfo st;

    double progress = controller->MotionManager->GetMotionTargetProgress(default_control_group, st);

    if (!st.IsOk())
        throw std::runtime_error("GetMotionTargetProgress failed: " + st.Message);

    return progress;


}

bool MotomanRobot::HasReachedTarget()
{
    StatusInfo status;

    DOUBLE64 progress = controller->MotionManager->GetMotionTargetProgress(ControlGroupId::R1, status);
    if (!status.IsOk())
    {
        return false;
    }

    INT32 stepsRemaining = 0;
    StatusInfo stepStatus = controller->MotionManager->GetRemainingStepsInTrajectory(stepsRemaining);

    if (!stepStatus.IsOk())
    {
        return false;
    }

    std::cout << "Motion progress: " << progress << "%, Steps remaining: " << stepsRemaining << std::endl;

    return (progress >= 99.0 && stepsRemaining <=1) || stepsRemaining == 0; // Consider target reached if progress is 95% or more and 1 or fewer steps remain, or if progress is 99% or more regardless of steps remaining (to account for any potential discrepancies in step counting)
}

void MotomanRobot::write_io_bit(UINT32 index, bool value)
{
    if (!connected)
        throw std::runtime_error("Not connected");


    StatusInfo st = controller->Io->WriteBit(index,value);

    if (!st.IsOk())
        throw std::runtime_error("Write IoBit failed: " + st.Message);
}

void MotomanRobot::write_general_output_bit(UINT16 group, UINT8 bitPosition, bool ioBit)
{
    if (!connected)
        throw std::runtime_error("Not connected");

    StatusInfo st = controller->Io->WriteBit(
        IOType::GeneralOutput,
        group,
        bitPosition,
        ioBit);

    if (!st.IsOk())
        throw std::runtime_error(
            "WriteBit failed: " + st.Message);
}

bool MotomanRobot::read_general_output_bit(UINT16 group, UINT8 bitPosition)
{
    if (!connected)
        throw std::runtime_error("Not connected");

    bool ioBit;

    StatusInfo st = controller->Io->ReadBit(
        IOType::GeneralOutput,
        group,
        bitPosition,
        ioBit);

    return ioBit;

}





