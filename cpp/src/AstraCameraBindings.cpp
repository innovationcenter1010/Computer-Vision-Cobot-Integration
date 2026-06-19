#include <pybind11/pybind11.h>
#include "AstraCamera.hpp"

namespace py = pybind11;


PYBIND11_MODULE(astra_cpp, m)
{

    // ---------------- RGBD Frame ----------------

    py::class_<RGBDFrame>(m, "RGBDFrame")
        .def(py::init<>())

        .def_readwrite(
            "timestamp",
            &RGBDFrame::timestamp)

        .def_readwrite(
            "color_frame_index",
            &RGBDFrame::colorFrameIndex)

        .def_readwrite(
            "depth_frame_index",
            &RGBDFrame::depthFrameIndex)

        .def_readwrite(
            "color_width",
            &RGBDFrame::colorWidth)

        .def_readwrite(
            "color_height",
            &RGBDFrame::colorHeight)

        .def_readwrite(
            "depth_width",
            &RGBDFrame::depthWidth)

        .def_readwrite(
            "depth_height",
            &RGBDFrame::depthHeight)

        .def_readwrite(
            "rgb",
            &RGBDFrame::rgb)

        .def_readwrite(
            "depth",
            &RGBDFrame::depth);



    // ---------------- Status ----------------

    py::class_<astra_status>(m, "AstraStatus")

        .def(py::init<>())

        .def_readonly(
            "connected",
            &astra_status::connected)

        .def_readonly(
            "streaming",
            &astra_status::streaming)

        .def_readonly(
            "frames_captured",
            &astra_status::framesCaptured)

        .def_readonly(
            "fps",
            &astra_status::fps)

        .def_readonly(
            "last_frame_timestamp",
            &astra_status::lastFrameTimestamp);



    // ---------------- Result ----------------

    py::class_<astra_result>(m, "AstraResult")

        .def(py::init<>())

        .def_readonly(
            "operation_status",
            &astra_result::operation_status)

        .def_readonly(
            "astra_health",
            &astra_result::astra_health)

        .def_readonly(
            "snapshot",
            &astra_result::snapshot);



    // ---------------- Camera ----------------

    py::class_<AstraCamera>(m, "AstraCamera")

        .def(
            py::init<>())

        .def(
            "start",
            &AstraCamera::start)

        .def(
            "stop",
            &AstraCamera::stop)

        .def(
            "get_frame",
            &AstraCamera::getFrame);
}