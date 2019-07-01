#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include "Environment.hpp"
#include "VectorizedEnvironment.hpp"

#define __RSG_MAKE_STR(x) #x
#define _RSG_MAKE_STR(x) __RSG_MAKE_STR(x)
#define RSG_MAKE_STR(x) _RSG_MAKE_STR(x)

namespace py = pybind11;
using namespace raisim;

#ifndef ENVIRONMENT_NAME
  #define ENVIRONMENT_NAME RaisimGymEnv
#endif

PYBIND11_MODULE(_raisim_gym, m) {
  py::class_<VectorizedEnvironment<ENVIRONMENT>>(m, RSG_MAKE_STR(ENVIRONMENT_NAME))
    .def(py::init<std::string, std::string>())
    .def("init", &VectorizedEnvironment<ENVIRONMENT>::init)
    .def("getExtraInfoNames", &VectorizedEnvironment<ENVIRONMENT>::getExtraInfoNames)
    .def("reset", &VectorizedEnvironment<ENVIRONMENT>::reset)
    .def("observe", &VectorizedEnvironment<ENVIRONMENT>::observe)
    .def("step", &VectorizedEnvironment<ENVIRONMENT>::step)
    .def("setSeed", &VectorizedEnvironment<ENVIRONMENT>::setSeed)
    .def("step", &VectorizedEnvironment<ENVIRONMENT>::step)
    .def("testStep", &VectorizedEnvironment<ENVIRONMENT>::testStep)
    .def("close", &VectorizedEnvironment<ENVIRONMENT>::close)
    .def("isTerminalState", &VectorizedEnvironment<ENVIRONMENT>::isTerminalState)
    .def("setSimulationTimeStep", &VectorizedEnvironment<ENVIRONMENT>::setSimulationTimeStep)
    .def("setControlTimeStep", &VectorizedEnvironment<ENVIRONMENT>::setControlTimeStep)
    .def("getObDim", &VectorizedEnvironment<ENVIRONMENT>::getObDim)
    .def("getActionDim", &VectorizedEnvironment<ENVIRONMENT>::getActionDim)
    .def("getExtraInfoDim", &VectorizedEnvironment<ENVIRONMENT>::getExtraInfoDim)
    .def("getNumOfEnvs", &VectorizedEnvironment<ENVIRONMENT>::getNumOfEnvs)
    .def("startRecordingVideo", &VectorizedEnvironment<ENVIRONMENT>::startRecordingVideo)
    .def("stopRecordingVideo", &VectorizedEnvironment<ENVIRONMENT>::stopRecordingVideo)
    .def("curriculumUpdate", &VectorizedEnvironment<ENVIRONMENT>::curriculumUpdate);

}
