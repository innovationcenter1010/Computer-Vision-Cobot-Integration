#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "MotomanRobot.hpp"
#include "YMConnect.h"

namespace py = pybind11;

PYBIND11_MODULE(motoman_cpp, m)
{




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
        .def_readwrite("rz", &Position::rz);

    // ---------------- Robot ----------------
    py::class_<MotomanRobot>(m, "MotomanRobot")
        .def(py::init<const std::string&>())

        .def("connect", &MotomanRobot::connect)
        .def("disconnect", &MotomanRobot::disconnect)

        .def("is_connected", &MotomanRobot::is_connected)

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
        .def("add_point_to_trajectory", &MotomanRobot::add_point_to_trajectory, 
            py::arg("pos"))

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