cmake_minimum_required(VERSION 3.5)
project(screw_it)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(dynamixel_sdk_custom_interfaces REQUIRED) # you can include other packages as long as they are also in the package.xml

# Include seperate headers
include_directories(${CMAKE_SOURCE_DIR})
include_directories(include) # remeber to include the .h files


add_executable(motor_control
  src/motor_control.cpp
)

add_executable(CNN_interface
  src/CNN_interface.cpp
)

add_executable(Angle_Node
  src/Angle_Node.cpp
)


add_executable(CNN_interface_test
  src/CNN_interface_test.cpp
)

add_executable(CNN_interface_test_axilite
  src/CNN_axi_interface_test.cpp
)

ament_target_dependencies(motor_control
	rclcpp
	sensor_msgs
  std_msgs
  dynamixel_sdk_custom_interfaces # for vscode?
)

ament_target_dependencies(CNN_interface
	rclcpp
  OpenCV
	cv_bridge
  std_msgs
)

ament_target_dependencies(Angle_Node
	rclcpp
	sensor_msgs
	OpenCV
	cv_bridge
)

ament_target_dependencies(CNN_interface_test
	rclcpp
  OpenCV
	cv_bridge
  std_msgs
  )

ament_target_dependencies(CNN_interface_test_axilite
  rclcpp
  OpenCV
  cv_bridge
  std_msgs
)

install(TARGETS 
	motor_control 
	CNN_interface 
	Angle_Node 
	CNN_interface_test 
  CNN_interface_test_axilite
	DESTINATION lib/${PROJECT_NAME}
	)

install(DIRECTORY
	launch
	config
	DESTINATION share/${PROJECT_NAME}
	)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
