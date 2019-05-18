cmake_minimum_required(VERSION 3.13)
project(Test_car)

find_package(OpenCV REQUIRED) # after project()
set(CMAKE_CXX_STANDARD 11)

add_executable(smart_car main.cpp cmake-build-debug/DCMotor.h dc_motor.cpp lane_detector.cpp ir_tracer.cpp ir_detector.cpp lane.h lane.h lane.cpp)

include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(opencv ${OpenCV_LIBS})