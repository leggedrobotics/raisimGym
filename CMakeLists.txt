cmake_minimum_required(VERSION 3.10)
project(_raisim_gym)
set(CMAKE_CXX_STANDARD 14)

find_package(pybind11 2.3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(raisimOgre 0.6.0 CONFIG REQUIRED)
find_package(OpenMP REQUIRED)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

message("[RAISIM_GYM] including directory ${RSG_ENVIRONMENT_INCLUDE_PATH}")

pybind11_add_module(_raisim_gym raisim_gym/env/raisim_gym.cpp)
target_link_libraries(_raisim_gym PRIVATE raisim::raisim raisim::raisimOgre yaml-cpp)
target_include_directories(_raisim_gym PUBLIC raisim_gym/env ${RSG_ENVIRONMENT_INCLUDE_PATH} ${EIGEN3_INCLUDE_DIRS})
target_compile_options(_raisim_gym PRIVATE -mtune=native -fPIC -O3)
