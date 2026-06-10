#pragma once

#include <string>
#include <cstdint>
#include <thread>
#include <atomic>
#include <any>
#include "YMConnect.h"

struct robot_result
{
    StatusInfo status;
    std::any payload;
};

class Position
{
public:
    Position() = default;
    Position(double x, double y, double z, double rx, double ry, double rz)
        : x(x), y(y), z(z), rx(rx), ry(ry), rz(rz) {}

    double x = 0;
    double y = 0;
    double z = 0;
    double rx = 0;
    double ry = 0;
    double rz = 0;
};

class MotomanRobot
{
public:
    explicit MotomanRobot(const std::string& ip);
    ~MotomanRobot();

    robot_result connect();
    robot_result disconnect();
    void connection_watchdog_loop();
    robot_result ping_robot();

    // Integer variables
    int read_int(uint16_t index);
    robot_result write_int(uint16_t index, int value);

    // Position variables
    robot_result read_position(uint16_t index);
    robot_result write_position(uint16_t index, const Position& pos);

    // Motion control
    robot_result motion_start();
    robot_result motion_stop(bool clearTrajectory = false);
    robot_result clear_trajectory();

    //trajectory controll
    robot_result add_point_to_trajectory(const Position& pos);
    robot_result set_motion_profile(DOUBLE64 speed, DOUBLE64 accel, DOUBLE64 decel);
    double get_motion_progress();
    robot_result HasReachedTarget();

    bool is_connected() const { return connected; }

    robot_result write_io_bit(UINT32 index, bool value);
    robot_result write_general_output_bit(UINT16 group, UINT8 bitPosition, bool ioBit);
    robot_result read_general_output_bit(UINT16 group, UINT8 bitPosition);

    void PrintActiveAlarms();


private:
    std::string ip;
    MotomanController* controller = nullptr;
    std::string software_version;

    std::thread connectionwatchdog_thread;
    std::atomic<bool> running{false};
    std::atomic<bool> connected{false};

    MoveInterpolationType robot_interpolation_type = MoveInterpolationType::MoveJoint; // Default interpolation type for motions
    CoordinateType robot_coordinate_type = CoordinateType::BaseCoordinate; // Default coordinate type for position variables
    ControlGroupId default_control_group = ControlGroupId::R1; // Default control group for motions
    MotionAccelDecel robot_motion_attributes{ 25, 25 }; // Default to no acceleration/deceleration reduction
    DOUBLE64 robot_speed = 25; // Default speed for motions

};