#pragma once

#include <string>
#include <cstdint>
#include "YMConnect.h"

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

    void connect();
    void disconnect();

    // Integer variables
    int read_int(uint16_t index);
    void write_int(uint16_t index, int value);

    // Position variables
    Position read_position(uint16_t index);
    void write_position(uint16_t index, const Position& pos);

    // Motion control
    void motion_start();
    void motion_stop(bool clearTrajectory = false);
    void clear_trajectory();

    //trajectory controll
    void MotomanRobot::add_point_to_trajectory(const Position& pos);

    bool is_connected() const { return connected; }

private:
    std::string ip;
    MotomanController* controller = nullptr;
    bool connected = false;

    CoordinateType robot_coordinate_type = CoordinateType::BaseCoordinate; // Default coordinate type for position variables
    ControlGroupId default_control_group = ControlGroupId::R1; // Default control group for motions
    MotionAccelDecel robot_motion_attributes{ 50.0, 50.0 }; // Default to no acceleration/deceleration reduction
    
};