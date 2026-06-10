#include "MotomanRobot.hpp"
#include "YMConnect.h"
#include <thread>
#include <chrono>
#include <future>
#include <stdexcept>
//#include <pybind11/pybind11.h>
//#include <pybind11/stl.h>


using namespace std::chrono_literals;

const std::chrono::milliseconds MOTION_POLLING_INTERVAL = 50ms;
const std::chrono::milliseconds CONNECTION_POLLING_INTERVAL = 100ms; 

UINT32 INTERPOLATION_INVALID_ALARM_CODE = 4684;



MotomanRobot::MotomanRobot(const std::string& ip)
    : ip(ip), controller(nullptr), connected(false)
{
}

MotomanRobot::~MotomanRobot()
{
    disconnect();
}

//connects the robot and arms the servos 
robot_result MotomanRobot::connect()
{
    StatusInfo status{};

    std::cout << "Connecting to robot at " << ip << "..." << std::endl;

    controller = YMConnect::OpenConnection(ip, status);

    StatusInfo arming_status = controller->ControlCommands->SetServos(SignalStatus::ON);

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

    std::cout << "connection successful";

    return result;
}

//disconnects from the robot and arms the servos 
robot_result MotomanRobot::disconnect()
{
    
    StatusInfo disarm_status;

    if (controller)
    {
        disarm_status = controller->ControlCommands->SetServos(SignalStatus::OFF);
        YMConnect::CloseConnection(controller);
        controller = nullptr;
    }

    else

    {
        robot_result result{};
        result.status = StatusInfo{ 1, "Controller not initialized" };
        result.payload = false;
        return result;
    }

    connected = false;
    running = false; 

    robot_result result{};
    result.status = disarm_status;
    result.payload = true;

    return result;
}

//the watchdog loop that polls the robot for connection status 
void MotomanRobot::connection_watchdog_loop()
{
    while (running)
    {
        bool ok = std::any_cast<bool>(ping_robot().payload);

        connected = ok;

        std::this_thread::sleep_for(CONNECTION_POLLING_INTERVAL);
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

    //read the software version as a verification that the cobot is still connected 
    StatusInfo st = controller->Status->ReadSystemInformation(
        SystemInfoId::R1,
        systemInfo
    );
    std::string checked_software_version = systemInfo.softwareVersion;


    robot_result result{};
    result.status = st;
    //compare the checnked software version with the one stored at connecion
    result.payload = checked_software_version == software_version;

    return result;
}

// -------------------- SETUP --------------------

// This function can be used to set up motion profiles for the robot. For simplicity, we will use the same profile for all motions in this example.
robot_result MotomanRobot::set_motion_profile(DOUBLE64 speed, DOUBLE64 accel, DOUBLE64 decel)
{

    robot_motion_attributes.accelRatio = accel;
    robot_motion_attributes.decelRatio = decel;
    robot_speed = speed;

    robot_result result{};
    result.status = StatusInfo{ 0, "Motion profile set successfully" };
    result.payload = true;

    return result;
}

// -------------------- INTEGER --------------------

//reads an integer from a register in the cobots memory
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

    //PrintActiveAlarms();

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

    std::cout << "Adding point to trajectory: (" << pos.x << ", " << pos.y << ", " << pos.z << ", "
              << pos.rx << ", " << pos.ry << ", " << pos.rz << ")" << std::endl;

    if(abs(pos.x) > 1500 || abs(pos.y) > 1500){

        return robot_result{StatusInfo{1, "Out of Bounds"}};

    }


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

    if(!IsPositionValid(motion.position)){
        result.status.StatusCode = INTERPOLATION_INVALID_ALARM_CODE;
        result.status.Message = "INTERPOLATION INAVALID";
    }

    else{

        result.status = controller->MotionManager->AddPointToTrajectory(motion);
        result.payload = result.status.IsOk();
    }

    //PrintActiveAlarms();

    return result;

}

robot_result MotomanRobot::HasReachedTarget()
{
    StatusInfo status;

    robot_result result{};

    result.payload = false;


    DOUBLE64 progress = controller->MotionManager->GetMotionTargetProgress(ControlGroupId::R1, status);
    result.status = status;
    if (!status.IsOk())
    {
        return result;
    }

    INT32 stepsRemaining = 0;
    status = controller->MotionManager->GetRemainingStepsInTrajectory(stepsRemaining);
    result.status = status;
    if (!status.IsOk())
    {
        return result;
    }

    std::cout << "Motion progress: " << progress << "%, Steps remaining: " << stepsRemaining << std::endl;

    result.payload = (progress >= 99.0 && stepsRemaining <=1) || stepsRemaining == 0; // Consider target reached if progress is 95% or more and 1 or fewer steps remain, or if progress is 99% or more regardless of steps remaining (to account for any potential discrepancies in step counting)

    StatusInfo active_alarms_status;
    ActiveAlarms active_alarms = get_active_alarms(active_alarms_status);

    //easy cath for wrong interpolation account
    if(active_alarms.numberOfActiveAlarms == 1 && active_alarms.alarms[0].code == INTERPOLATION_INVALID_ALARM_CODE){

        result.status.StatusCode = INTERPOLATION_INVALID_ALARM_CODE;
        result.status.Message = "INTERPOLATION INVALID";
        StatusInfo alarms_cleared = controller->Faults->ClearAllFaults();

    }

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
        IOType::GeneralInput,
        group,
        bitPosition,
        ioBit);

    return robot_result{ st, ioBit };
}

ActiveAlarms MotomanRobot::get_active_alarms(StatusInfo st)
{
    robot_result result{};

    ActiveAlarms activeAlarms;

    st = controller->Faults->GetActiveAlarms(activeAlarms);

    return activeAlarms;
}

bool MotomanRobot::IsPositionValid(PositionData& cartesianPos)
{
    PositionData converted;
    PositionData prevJointAngles;

    StatusInfo St = controller->ControlGroup->ReadPositionData(default_control_group,CoordinateType::Pulse,0,0,prevJointAngles);

    StatusInfo status = controller->Kinematics->ConvertPositionFromCartesian(
        default_control_group,
        cartesianPos,
        KinematicConversions::CartesianPosToJointAngle,
        KinematicType::Delta,
        prevJointAngles.axisData,
        converted
    );

    // If IK failed → invalid position
    if (status.IsOk() != 0){
    
        return false;
    }

    return true;
}





