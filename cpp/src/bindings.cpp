#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "MotomanRobot.hpp"

namespace py = pybind11;

PYBIND11_MODULE(motoman_cpp, m)
{
    // ---------------- Position ----------------
    py::class_<Position>(m, "Position")
        .def(py::init<>())
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
        .def("write_position", &MotomanRobot::write_position);
}