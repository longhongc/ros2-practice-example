cmake_minimum_required(VERSION 3.5)
project(sensors)

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
find_package(sensor_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(rcpputils REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_compression REQUIRED)
find_package(rosbag2_storage_default_plugins REQUIRED)

add_executable(temp_pub src/temp.cpp)
ament_target_dependencies(temp_pub 
    rclcpp 
    sensor_interfaces
)

add_executable(speed_pub src/speed.cpp)
ament_target_dependencies(speed_pub 
    rclcpp 
    geometry_msgs
)

add_executable(laser_pub src/laser.cpp)
ament_target_dependencies(laser_pub 
    rclcpp 
    sensor_msgs
    rosbag2_cpp 
    rosbag2_storage
    rosbag2_storage_default_plugins
)

install(TARGETS
    temp_pub
    speed_pub
    laser_pub
    DESTINATION lib/${PROJECT_NAME}
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
