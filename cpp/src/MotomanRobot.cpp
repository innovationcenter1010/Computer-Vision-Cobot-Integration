#include "MotomanRobot.hpp"
#include <stdexcept>

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
        throw std::runtime_error("Not connected");

    StatusInfo st =
        controller->MotionManager->ClearAllTrajectory();

    if (!st.IsOk())
        throw std::runtime_error("ClearAllTrajectory failed: " + st.Message);
}


