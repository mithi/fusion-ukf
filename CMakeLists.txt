project(UnscentedKF)

cmake_minimum_required(VERSION 3.5)
add_definitions(-std=c++0x)

include_directories(headers)

set(SOURCES src/main.cpp
            src/tools.cpp
            src/radarpredictor.cpp
            src/statepredictor.cpp
            src/stateupdater.cpp
            src/tests.cpp )

add_executable(UnscentedKF ${SOURCES})
