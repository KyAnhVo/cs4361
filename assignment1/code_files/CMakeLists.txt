cmake_minimum_required(VERSION 3.10)
project(Rasterizer CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Archlinux was screaming at me because some GTK, so
# I look at the code and saw no need for it. Thus I
# limit it down to the core packages.
find_package(OpenCV REQUIRED core imgproc highgui)
# On Ubuntu, Eigen ships a CMake config that defines the imported target Eigen3::Eigen
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(Rasterizer
    main.cpp
    rasterizer.hpp rasterizer.cpp
    Triangle.hpp  Triangle.cpp
)

# Link libraries (OpenCV provides include dirs via the target, Eigen is header-only)
target_link_libraries(Rasterizer PRIVATE ${OpenCV_LIBS} Eigen3::Eigen)
