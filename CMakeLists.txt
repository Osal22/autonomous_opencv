cmake_minimum_required(VERSION 3.8)
project(laserscan_localization)





find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED )
find_package(nav_msgs REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  control_msgs
  moveit_msgs
  geometry_msgs
  nav_msgs
  moveit_ros_planning_interface
)
add_executable(autonomous_opencv src/main.cpp src/Laserscan_localization.cpp  src/Map.cpp src/MapSize.cpp src/MapNode.cpp)
target_link_libraries(autonomous_opencv ${Boost_LIBRARIES} ${OpenCV_LIBS})
ament_target_dependencies(autonomous_opencv
rclcpp
std_msgs
std_srvs
sensor_msgs
control_msgs
moveit_msgs
geometry_msgs
nav_msgs
moveit_ros_planning_interface
)
add_executable(scan_test src/scan_test.cpp)
target_link_libraries(scan_test ${Boost_LIBRARIES} ${OpenCV_LIBS})
ament_target_dependencies(scan_test
rclcpp
std_msgs
std_srvs
sensor_msgs
control_msgs
moveit_msgs
geometry_msgs
nav_msgs
moveit_ros_planning_interface
)

install(TARGETS autonomous_opencv DESTINATION lib/${PROJECT_NAME})
install(TARGETS scan_test DESTINATION lib/${PROJECT_NAME})

ament_package()
