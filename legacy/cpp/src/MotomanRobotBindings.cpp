#include <pybind11/pybind11.h>

#include "MotomanRobot.hpp"
#include "YMConnect.h"

namespace py = pybind11;


PYBIND11_MODULE(motoman_cpp, m)
{

    // -----------------Status Info ----------------

    py::class_<StatusInfo>(m, "StatusInfo")
        .def(py::init<>())
        .def_readwrite("status_code", &StatusInfo::StatusCode)
        .def_readwrite("message", &StatusInfo::Message)
        .def("is_ok", &StatusInfo::IsOk);

    py::class_<robot_result>(m, "robot_result")
    .def(py::init<>())

    .def_readwrite("status", &robot_result::status)

    .def_property_readonly(
        "payload",
        [](const robot_result& r) -> py::object
        {
            if (!r.payload.has_value())
                return py::none();

            const auto& type = r.payload.type();

            if (type == typeid(std::string))
                return py::cast(std::any_cast<const std::string&>(r.payload));

            if (type == typeid(int))
                return py::cast(std::any_cast<int>(r.payload));

            if (type == typeid(double))
                return py::cast(std::any_cast<double>(r.payload));

            if (type == typeid(bool))
                return py::cast(std::any_cast<bool>(r.payload));

            if (type == typeid(Position))
                return py::cast(std::any_cast<const Position&>(r.payload));

            return py::none();
        });


    // ---------------- Position ----------------
    py::class_<Position>(m, "Position")
        .def(py::init<>())
        .def(py::init<double, double, double,
                  double, double, double>())
        .def_readwrite("x", &Position::x)
        .def_readwrite("y", &Position::y)
        .def_readwrite("z", &Position::z)
        .def_readwrite("rx", &Position::rx)
        .def_readwrite("ry", &Position::ry)
        .def_readwrite("rz", &Position::rz)

        .def("copy", [](const Position &p) {
        return Position(p.x, p.y, p.z, p.rx, p.ry, p.rz);
    });

    // ---------------- Robot ----------------
    py::class_<MotomanRobot>(m, "MotomanRobot")
        .def(py::init<const std::string&>())

        .def("connect", &MotomanRobot::connect)
        .def("disconnect", &MotomanRobot::disconnect)

        // integer API
        .def("read_int", &MotomanRobot::read_int)
        .def("write_int", &MotomanRobot::write_int)

        // position API
        .def("read_position", &MotomanRobot::read_position)
        .def("write_position", &MotomanRobot::write_position)

        // motion API
        .def("motion_start", &MotomanRobot::motion_start)

        .def("motion_stop",
             &MotomanRobot::motion_stop,
             py::arg("clear_trajectory") = false)

        .def("clear_trajectory",
             &MotomanRobot::clear_trajectory)

        // trajectory control API
        //.def("add_point_to_trajectory", &MotomanRobot::add_point_to_trajectory);
        .def("add_point_to_trajectory", &MotomanRobot::add_point_to_trajectory)

        .def("has_reached_target", &MotomanRobot::HasReachedTarget)

        .def("set_motion_profile",
            &MotomanRobot::set_motion_profile,
            py::arg("speed") = 100.0,
            py::arg("accel") = 100.0,
            py::arg("decel") = 100.0)
            
        .def("write_io_bit", &MotomanRobot::write_io_bit, 
            py::arg("index"), py::arg("value"))

        .def("write_general_output_bit",
            &MotomanRobot::write_general_output_bit,
            py::arg("group"),
            py::arg("bit_position"),
            py::arg("value"))

        .def("read_general_output_bit",
            &MotomanRobot::read_general_output_bit,
            py::arg("group"),
            py::arg("bit_position"));

}

