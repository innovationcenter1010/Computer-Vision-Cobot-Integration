#include "MotomanRobot.hpp"
#include "YMConnect.h"

#include <stdexcept>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;


MotomanRobot::MotomanRobot(const std::string& ip)
    : ip(ip), controller(nullptr), connected(false)
{
}

MotomanRobot::~MotomanRobot()
{
    disconnect();
}

robot_result MotomanRobot::connect()
{
    StatusInfo status{};

    controller = YMConnect::OpenConnection(ip, status);

    connected = status.IsOk();
    running = status.IsOk();
   


    //thread for polling the connection to update the connected status
    connectionwatchdog_thread = std::thread(&MotomanRobot::connection_watchdog_loop, this);

    robot_result result{};
    result.status = status;
    result.payload = status.IsOk();

    return result;
}

robot_result MotomanRobot::disconnect()
{
    if (controller)
    {
        YMConnect::CloseConnection(controller);
        controller = nullptr;
    }

    connected = false;
    running = false; 

    robot_result result{};
    result.status = StatusInfo{ 0, "Disconnected successfully" };
    result.payload = false;

    return result;
}

void MotomanRobot::connection_watchdog_loop()
{
    while (running)
    {
        bool ok = ping_robot();

        connected = ok;

        std::this_thread::sleep_for(500ms);
    }
}

bool MotomanRobot::ping_robot()
{
    if (!controller)
        return false;

    SystemInfoData systemInfo;

    StatusInfo st = controller->Status->ReadSystemInformation(
        SystemInfoId::R1,
        systemInfo
    );

    if (!st.IsOk())
        return false;

    // Example field name — adjust to actual API structure
    std::string checked_software_version = systemInfo.softwareVersion;

    return checked_software_version == software_version;
}

// -------------------- SETUP --------------------

StatusInfo MotomanRobot::set_motion_profile(DOUBLE64 speed, DOUBLE64 accel, DOUBLE64 decel)
{
    // This function can be used to set up motion profiles for the robot. For simplicity, we will use the same profile for all motions in this example.
    // In a more complex implementation, you could allow different profiles for different motions or groups.

    robot_motion_attributes.accelRatio = accel;
    robot_motion_attributes.decelRatio = decel;
    robot_speed = speed;

    return StatusInfo{ 0, "Motion profile set successfully" };
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

StatusInfo MotomanRobot::write_int(uint16_t index, int value)
{
    if (!connected) throw std::runtime_error("Not connected");

    IntegerVariableData data(value, index);

    return controller->Variables->IntegerVariable->Write(data);

 
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

StatusInfo MotomanRobot::write_position(uint16_t index, const Position& pos)
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

    return controller->Variables->RobotPositionVariable->Write(data);


}

// -------------------- MOTION --------------------

StatusInfo MotomanRobot::motion_start()
{
    if (!connected)
        throw std::runtime_error("Not connected");

    return controller->MotionManager->MotionStart();
}

StatusInfo MotomanRobot::motion_stop(bool clearTrajectory)
{
    if (!connected)
        throw std::runtime_error("Not connected");

    return controller->MotionManager->MotionStop(clearTrajectory);

}

StatusInfo MotomanRobot::clear_trajectory()
{
    if (!connected)
        //throw std::runtime_error("Not connected");

    return controller->MotionManager->ClearAllTrajectory();

}

// -------------------- TRAJECTORY --------------------

StatusInfo MotomanRobot::add_point_to_trajectory(const Position& pos)
{
    if (!connected)
        throw std::runtime_error("Not connected");

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
    return controller->MotionManager->AddPointToTrajectory(motion);

}


//-------------------- IO --------------------

StatusInfo MotomanRobot::write_io_bit(UINT32 index, bool value)
{
    if (!connected)
        throw std::runtime_error("Not connected");


    return controller->Io->WriteBit(index,value);

}

StatusInfo MotomanRobot::write_general_output_bit(UINT16 group, UINT8 bitPosition, bool ioBit)
{
    if (!connected)
        throw std::runtime_error("Not connected");

    return controller->Io->WriteBit(
        IOType::GeneralOutput,
        group,
        bitPosition,
        ioBit);
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




