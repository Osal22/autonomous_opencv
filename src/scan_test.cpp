#include <math.h>  
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <limits>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <memory>
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <ostream>
#include <sstream>
#include <bits/stdc++.h>
#include <time.h>
#include <ctime>
#include <stdexcept>
#include <numeric>
#include <algorithm>
#include <chrono>
#include <cmath>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <thread>
#include <nav_msgs/msg/odometry.hpp>
  int resolution=70;
  double angle_offset_value=0;
  double pi = 3.14159265359;
  float inf = std::numeric_limits<float>::infinity();
  cv::Vec3b  color={0,0,0};

  struct map_size
{
  int width;
  int height;
};

map_size _map_size={900,900};
nav_msgs::msg::Odometry::SharedPtr _odom_data;
struct point_coor
{
   float x;
   float y;
};
std::vector <point_coor> _coor_data;


int get_angle_offest(){

  return angle_offset_value;
}

void set_delta_pose(nav_msgs::msg::Odometry::SharedPtr odom_data)
{
_odom_data=odom_data;
}

nav_msgs::msg::Odometry::SharedPtr get_delta_pose()
{
  return _odom_data;
}

void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_data){
  tf2::Quaternion q(
    odom_data->pose.pose.orientation.x,
    odom_data->pose.pose.orientation.y,
    odom_data->pose.pose.orientation.z,
    odom_data->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
   auto degree = yaw * (180/pi);
    if( degree < 0 )
    {
         degree += 360.0; // convert negative to positive angles
      
    }
    angle_offset_value=int(degree);

    set_delta_pose(odom_data);
    // std::cout<<"odom degree:= "<<int(degree)<<" rad:= "<<(yaw)<<std::endl;
}


std::vector <point_coor> compute_coor(const sensor_msgs::msg::LaserScan::SharedPtr _scan)
  {
    _coor_data.clear();
    float angle_ff=abs(get_angle_offest());
    point_coor _points;
    double angle_min = _scan->angle_min;
    double angle_increment = _scan->angle_increment;
    double range_min = _scan->range_min;
    for (size_t i = 0; i < _scan->ranges.size(); ++i) {
        if(_scan->ranges[i]==inf)
        {
        //   std::cout<<"got inf:= "<<std::endl;
        }
        else
        {
        double range = _scan->ranges[i];
        double angle = angle_min + i * angle_increment;
        auto angle_raians=angle_ff * (pi / 180);
        double x = range * std::cos(angle+angle_raians);
        double y = range * std::sin(angle+angle_raians);
        _points.x=x;
        _points.y=y;
        _coor_data.push_back(_points);
        }
  }
    return _coor_data;
  }


cv::Point convert_to_opencv_points(float x, float y)
{
  
   return cv::Point((-y*resolution)+int(_map_size.height/2),(-x*resolution)+int(_map_size.width/2));

}
void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    auto map = cv::Mat(900,900, CV_8UC3, cv::Scalar(255,255,255));
    auto points=compute_coor(msg);
    auto _odom_current_position =get_delta_pose();

    for (auto itr:points){
      map.at<cv::Vec3b>(convert_to_opencv_points(itr.x,itr.y)) = color;
    //   map.at<cv::Vec3b>(convert_to_opencv_points(itr.x+_odom_current_position->pose.pose.position.x,itr.y+_odom_current_position->pose.pose.position.y)) = color;
      
      }
    cv::namedWindow("map", cv::WINDOW_NORMAL);
    cv::imshow("map",map);
    auto key_vlaue=cv::waitKey(1);




}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("laser_scan_subscriber");
  auto odom_subscription_=node->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, odom_callback);
  auto sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", 10, laserScanCallback);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}