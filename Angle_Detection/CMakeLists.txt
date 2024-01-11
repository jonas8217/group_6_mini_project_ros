cmake_minimum_required(VERSION 3.5)

project(Angle_Detection LANGUAGES CXX)
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(Angle_Detection  main.cpp)
target_link_libraries( Angle_Detection ${OpenCV_LIBS} )
