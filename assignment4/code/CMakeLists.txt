cmake_minimum_required(VERSION 3.10)
project(BezierCurve CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV REQUIRED core imgproc highgui)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories("/usr/local/include/eigen3")

add_executable(BezierCurve main.cpp)

target_link_libraries(BezierCurve PRIVATE ${OpenCV_LIBS} Eigen3::Eigen)
