project(UnscentedKF)

cmake_minimum_required(VERSION 3.5)
add_definitions(-std=c++0x)

include_directories(headers)

set(SOURCES src/tools.cpp
            src/usagecheck.cpp
            src/measurementpredictor.cpp
            src/statepredictor.cpp
            src/stateupdater.cpp
            src/datapoint.cpp
            src/fusionukf.cpp
            src/main.cpp)

add_executable(UnscentedKF ${SOURCES})
