cmake_minimum_required(VERSION 3.5)

project(Screw_Angle_Detector_Embedded LANGUAGES CXX)
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(Screw_Angle_Detector_Embedded main.cpp)
target_link_libraries( Screw_Angle_Detector_Embedded ${OpenCV_LIBS} )
