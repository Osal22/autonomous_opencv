#ifndef LASERSCAN_LOCALIZATION_HPP
#define LASERSCAN_LOCALIZATION_HPP

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
#include "MapNode.hpp"
#include "MapSize.hpp"
#include "Map.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <ctime>
#include <unistd.h>
#include <sys/time.h>


using std::placeholders::_1;



struct map_size
{
  int width;
  int height;
};

struct point_coor
{
   float x;
   float y;
   void print_p()
   {
    std::cout<<" x:= "<<x<<" y:= "<<y;
   }
     
};

struct velocity_vector
{
  float x;
  float y;
  float angle;

  void print()
  {
    std::cout<<"x:= "<<x<<" y:= "<<y<<" angle:= "<<angle<<std::endl;
  }


};

class Laserscan_localization : public rclcpp::Node
{
  public:
    Laserscan_localization();
   

private:
  Map* _map_planner =new Map;

  velocity_vector velocity_normal;
  // velocity_normal.x=1;
  // velocity_normal.y=0;
  // velocity_normal.angle=0;       
  map_size _map_size={900,900};
  int resolution=50,make_origin;
  cv::Point convert_to_opencv_points(float x,float y);
  std::deque<point_coor> planned_path;
  std::deque<point_coor> plan_path( cv::Mat &_map,const cv::Point &_start,const cv::Point &_end);
  cv::Point origin={250,250};
  double pi = 3.14159265359;
  bool ref_coor_flag=true,is_rotating=false;
  cv::Vec3b  color={0,0,0};
  cv::Vec3b  path_color={15,255,0};
  cv::Vec3b  target_color={15,255,255};


  point_coor diff_point;
  cv::Point current_position,prev_position;
  // void CallBackFunc(int event, int x, int y, int flags, void* userdata);

  std::vector<cv::Point>position_points_holder;
  std::vector <point_coor> ref_coor;
  std::vector <point_coor> compute_coor(const sensor_msgs::msg::LaserScan::SharedPtr _scan);
  point_coor p0,p1,p2,p3,zero_coor;
  std::vector <float> _scan_data;
  std::vector <point_coor> _coor_data;
  int image_save_no=0;
  float _dist(point_coor _start,point_coor _end);
   
  void calulate_points(const sensor_msgs::msg::LaserScan::SharedPtr _scan);
  float inf = std::numeric_limits<float>::infinity();
  void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void cmd_vel_callback( geometry_msgs::msg::Twist::SharedPtr _cmd_del);
  void set_velocity(geometry_msgs::msg::Twist _cmd_vel);
  geometry_msgs::msg::Twist get_velocity();


  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_data) ;
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_data) ;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vle_subscription_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_map;
  void save_image(const cv::Mat &_mat);
  std::string image_save_path="/home/goalbytes/_dev/laserscan_localization/saved_images/";

  void save_csv_data(const sensor_msgs::msg::LaserScan::SharedPtr &_scan);
  std::string return_current_time_and_date();
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_scan;
  nav_msgs::msg::Odometry::SharedPtr _odom_data;
  // std::vector<MapNode *> path_calculated;
  void set_delta_pose(nav_msgs::msg::Odometry::SharedPtr odom_data);
  nav_msgs::msg::Odometry::SharedPtr get_delta_pose();
  // void set_path(std::vector<MapNode *>_path);
  int angle_offset_value;
  int get_angle_offest();
  // driver section
  geometry_msgs::msg::Twist controller;
  void timer_callback();
  void set_path(std::vector<MapNode *>_path);
  double angleBetweenLines(double x1a, double y1a, double x1b, double y1b, double x2a, double y2a, double x2b, double y2b);

  nav_msgs::msg::OccupancyGrid matToOccupancyGrid(const cv::Mat& mat);
  std::deque<point_coor> update_path;

  // imu section
  tf2::Quaternion quat_tf;
  std::vector<cv::Point> points_holder;
  bool flag_points_holder=true;
  nav_msgs::msg::Odometry::SharedPtr prev_odom_data;
  void compute_velocty_from_odom(nav_msgs::msg::Odometry::SharedPtr odom_data);
  uint64_t time_prev,time_now;
  uint64_t return_time_milli();
  bool prev_odom_flag=false;
  double delta_t=0.0;
  

  std::deque<point_coor> smooth_path();
  point_coor smooth( point_coor x0,point_coor x1 ,float t);

};
  

#endif  //LASERSCAN_LOCALIZATION_HPP