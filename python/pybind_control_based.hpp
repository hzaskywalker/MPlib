#pragma once

#include <vector>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/pybind11.h>
#include "../src/control_based_planner.h"

namespace py = pybind11;

#ifdef USE_SINGLE
using DATATYPE = float;
#else
using DATATYPE = double;
#endif


using ControlBasedPlanner = ControlBasedPlannerTpl<DATATYPE>;
using PlannerStatus = ob::PlannerStatus;
using Path = ob::Path;
using PathGeometric = og::PathGeometric;


void build_pycontrol(py::module &m_all) {
    auto m = m_all.def_submodule("control_based");

    auto PyControlBasedPlanner = py::class_<ControlBasedPlanner, std::shared_ptr<ControlBasedPlanner>>(m, "ControlBasedPlanner");
    PyControlBasedPlanner.def(py::init<PlanningWorldTpl_ptr<DATATYPE> const &>(), py::arg("world"))
            .def("plan", &ControlBasedPlanner::plan, 
                py::arg("start_state"), 
                py::arg("goal_states"),
                py::arg("planner_name") = "RRTConnect",  
                py::arg("time") = 1.0, 
                py::arg("range") = 0.0, 
                py::arg("verbose") = false,
                py::arg("integration_step") = 0.01
            );

}