cmake_minimum_required(VERSION 3.8)
project(arduino_publihser)

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(motor_data_msgs REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  message_filters
  nodelet
  pluginlib
  roscpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros)
  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
  )
ament_auto_add_executable(arduino_publihser_node src/arduino_publihser.cpp)
target_link_libraries(arduino_publihser_node ${Boost_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})
ament_target_dependencies(arduino_publihser_node
rclcpp
std_msgs
std_srvs
sensor_msgs
geometry_msgs
motor_data_msgs)


ament_auto_package()
