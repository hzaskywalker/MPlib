#pragma once

#include <vector>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/pybind11.h>

#include "../src/dubins_planner.h"

namespace py = pybind11;
//using namespace dubins;

#ifdef USE_SINGLE
using DATATYPE = float;
#else
using DATATYPE = double;
#endif


using DubinsPlanner = dubins::OMPLPlannerTpl<DATATYPE>;
using PlannerStatus = ob::PlannerStatus;
using Path = ob::Path;
using PathGeometric = og::PathGeometric;


void build_dubins(py::module &m_all) {
    auto m = m_all.def_submodule("dubins");

    auto PyDubinsPlanner = py::class_<DubinsPlanner, std::shared_ptr<DubinsPlanner>>(m, "DubinsPlanner");
    PyDubinsPlanner.def(
                py::init<PlanningWorldTpl_ptr<DATATYPE> const &>(), py::arg("world")
            ).def("plan", &DubinsPlanner::plan, 
                py::arg("start_state"), 
                py::arg("goal_states"),
                py::arg("planner_name") = "RRTConnect",  
                py::arg("time") = 1.0, 
                py::arg("range") = 0.0, 
                py::arg("verbose") = false
            );
}