cmake_minimum_required(VERSION 3.5)
project(landing_zone_system)

# Use C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(OpenCV REQUIRED)

# include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# gather all source files
file(GLOB_RECURSE SRC_FILES
  src/*.cpp
)

# executable
add_executable(landing_zone_node ${SRC_FILES})

# link dependencies
ament_target_dependencies(landing_zone_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  gazebo_ros
  OpenCV
)

# install the executable
install(TARGETS landing_zone_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# install public headers
install(DIRECTORY include/ DESTINATION include)

# install launch files and worlds
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY worlds/ DESTINATION share/${PROJECT_NAME}/worlds)

ament_package()
