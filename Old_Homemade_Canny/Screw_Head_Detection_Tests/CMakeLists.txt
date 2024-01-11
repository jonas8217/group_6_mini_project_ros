cmake_minimum_required(VERSION 3.5)

project(Screw_Head_Detection_Tests LANGUAGES CXX)
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(Screw_Head_Detection_Tests  main.cpp)
target_link_libraries( Screw_Head_Detection_Tests ${OpenCV_LIBS} )
