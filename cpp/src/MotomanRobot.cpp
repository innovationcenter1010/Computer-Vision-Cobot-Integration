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


    SystemInfoData systemInfo;
    StatusInfo st = controller->Status->ReadSystemInformation(
        SystemInfoId::R1,
        systemInfo
    );

    if(st.IsOk())
        software_version = systemInfo.softwareVersion;
   
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
        bool ok = std::any_cast<bool>(ping_robot().payload);

        connected = ok;

        std::this_thread::sleep_for(500ms);
    }
}

robot_result MotomanRobot::ping_robot()
{
    if (!controller)
    {
        robot_result result{};
        result.status = StatusInfo{ 1, "Controller not initialized" };
        result.payload = false;
        return result;
    }

    SystemInfoData systemInfo;

    StatusInfo st = controller->Status->ReadSystemInformation(
        SystemInfoId::R1,
        systemInfo
    );

    // Example field name — adjust to actual API structure
    std::string checked_software_version = systemInfo.softwareVersion;


    robot_result result{};
    result.status = st;
    result.payload = checked_software_version == software_version;

    return result;
}

// -------------------- SETUP --------------------

robot_result MotomanRobot::set_motion_profile(DOUBLE64 speed, DOUBLE64 accel, DOUBLE64 decel)
{
    // This function can be used to set up motion profiles for the robot. For simplicity, we will use the same profile for all motions in this example.
    // In a more complex implementation, you could allow different profiles for different motions or groups.

    robot_motion_attributes.accelRatio = accel;
    robot_motion_attributes.decelRatio = decel;
    robot_speed = speed;

    robot_result result{};
    result.status = StatusInfo{ 0, "Motion profile set successfully" };
    result.payload = true;

    return result;
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

robot_result MotomanRobot::write_int(uint16_t index, int value)
{
    if (!connected) 
        return robot_result{ StatusInfo{ 1, "Not connected" }, false };

    IntegerVariableData data(value, index);

    robot_result result{};
    result.status = controller->Variables->IntegerVariable->Write(data);
    result.payload = result.status.IsOk();

    return result;

}

// -------------------- POSITION --------------------

robot_result MotomanRobot::read_position(uint16_t index)
{
    if (!connected)
        return robot_result{ StatusInfo{ 1, "Not connected" }, Position{} };

    RobotPositionVariableData data{};

    StatusInfo st =
        controller->Variables->RobotPositionVariable->Read(index, data);

    if (!st.IsOk())
        return robot_result{ st, Position{} };

    Position p;

    // YMConnect mapping: axisData[0]=X, [1]=Y, [2]=Z
    p.x = data.positionData.axisData[0];
    p.y = data.positionData.axisData[1];
    p.z = data.positionData.axisData[2];
    p.rx = data.positionData.axisData[3];
    p.ry = data.positionData.axisData[4];   
    p.rz = data.positionData.axisData[5];

    robot_result result{};
    result.status = st;
    result.payload = p;

    return result;
}

robot_result MotomanRobot::write_position(uint16_t index, const Position& pos)
{
    if (!connected) 
        return robot_result{ StatusInfo{ 1, "Not connected" }, false };

    RobotPositionVariableData data{};
    data.variableIndex = index;

    data.positionData.axisData[0] = pos.x;
    data.positionData.axisData[1] = pos.y;
    data.positionData.axisData[2] = pos.z;
    data.positionData.axisData[3] = pos.rx;
    data.positionData.axisData[4] = pos.ry;
    data.positionData.axisData[5] = pos.rz;

    robot_result result{};
    result.status = controller->Variables->RobotPositionVariable->Write(data);
    result.payload = result.status.IsOk();

    return result;

}

// -------------------- MOTION --------------------

robot_result MotomanRobot::motion_start()
{
    if (!connected)
        return robot_result{ StatusInfo{ 1, "Not connected" }, false };

    robot_result result{};
    result.status = controller->MotionManager->MotionStart();
    result.payload = result.status.IsOk();

    return result;
}

robot_result MotomanRobot::motion_stop(bool clearTrajectory)
{
    if (!connected)
        return robot_result{ StatusInfo{ 1, "Not connected" }, false };

    robot_result result{};
    result.status = controller->MotionManager->MotionStop(clearTrajectory);
    result.payload = result.status.IsOk();
    return result; 

}

robot_result MotomanRobot::clear_trajectory()
{
    if (!connected)
        return robot_result{ StatusInfo{ 1, "Not connected" }, false };

    robot_result result{};
    result.status = controller->MotionManager->ClearAllTrajectory();
    result.payload = result.status.IsOk();

    return result;

}

// -------------------- TRAJECTORY --------------------

robot_result MotomanRobot::add_point_to_trajectory(const Position& pos)
{
    if (!connected)
        return robot_result{ StatusInfo{ 1, "Not connected" }, false };

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

    robot_result result{};
    result.status = controller->MotionManager->AddPointToTrajectory(motion);
    result.payload = result.status.IsOk();
    return result;

}


//-------------------- IO --------------------

robot_result MotomanRobot::write_io_bit(UINT32 index, bool value)
{
    if (!connected)
        return robot_result{ StatusInfo{ 1, "Not connected" }, false };


    robot_result result{};
    result.status = controller->Io->WriteBit(index,value);
    result.payload = result.status.IsOk();

    return result;

}

robot_result MotomanRobot::write_general_output_bit(UINT16 group, UINT8 bitPosition, bool ioBit)
{
    if (!connected)
        return robot_result{ StatusInfo{ 1, "Not connected" }, false };

    robot_result result{};

    result.status = controller->Io->WriteBit(
        IOType::GeneralOutput,
        group,
        bitPosition,
        ioBit);
    result.payload = result.status.IsOk();

    return result;
}

robot_result MotomanRobot::read_general_output_bit(UINT16 group, UINT8 bitPosition)
{
    if (!connected)
        return robot_result{ StatusInfo{ 1, "Not connected" }, false };

    bool ioBit;

    StatusInfo st = controller->Io->ReadBit(
        IOType::GeneralOutput,
        group,
        bitPosition,
        ioBit);

    return robot_result{ st, ioBit };
}




