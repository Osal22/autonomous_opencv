#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <memory>
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include "Laserscan_localization.hpp"
#include <iostream>
#include <string>

#include <thread>
#include <chrono>
#include <vector>

using std::placeholders::_1;
using namespace std::chrono_literals;

float target_x=0,target_y=0,target_angle=0;


// velocity_vector velocity_normal;
// velocity_normal.x=1;
// velocity_normal.y=0;
// velocity_normal.angle=0;

cv::Vec3b  end_color={0,0,255};
cv::Vec3b  start_color={255,0,0};
bool start_planning=false,start_robot=false,first_image=true;
bool done_planning=false;
cv::Mat map = cv::Mat(900,900, CV_8UC3, cv::Scalar(255,255,255));
cv::Mat map_for_planning = cv::Mat(900,900, CV_8UC3, cv::Scalar(255,255,255));

geometry_msgs::msg::Twist velocity;
cv::Point target_point;

// laser start here //////////////////////////////////////////////////////////////////////////////  
std::ofstream all_data_csv ("/home/goalbytes/_dev/laserscan_localization/saved_images/data.csv");


void set_target(float x,float y,float angle)
{
  target_x=x;
  target_y=y;
  target_angle=angle;

}
velocity_vector get_target()
{
  velocity_vector _target_vel_vector;
  _target_vel_vector.x=target_x;
  _target_vel_vector.y=target_y;
  _target_vel_vector.angle=target_angle;
  return _target_vel_vector;

}

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
     if  ( event == cv::EVENT_LBUTTONDOWN )
     {
      std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
      target_point={x,y};
      start_planning=true;

	 }
     else if  ( event == cv::EVENT_RBUTTONDOWN )
     {

        start_robot=!start_robot;
     }
     else if  ( event == cv::EVENT_MBUTTONDOWN )
     {
          
     }
     else if ( event == cv::EVENT_MOUSEMOVE )
     {
      set_target(x,y,0);
      // std::cout<<"x:= "<<x<<" y:= "<<y<<std::endl;


     }
}


Laserscan_localization::Laserscan_localization(): Node("laserscan_localization")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 1, std::bind(&Laserscan_localization::topic_callback, this, _1));


      cmd_vle_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 1, std::bind(&Laserscan_localization::cmd_vel_callback, this, _1));
      
      imu_subscription_=this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", 1, std::bind(&Laserscan_localization::imu_callback, this, _1));

      odom_subscription_=this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 1, std::bind(&Laserscan_localization::odom_callback, this, _1));
      
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
      timer_ = this->create_wall_timer(
      10ms, std::bind(&Laserscan_localization::timer_callback, this));

      timer_scan = this->create_wall_timer(
      10ms, std::bind(&Laserscan_localization::timer_callback, this));

      publisher_map = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);



    }

void Laserscan_localization::timer_callback()
    {
      // publisher_->publish(controller);
    }

float Laserscan_localization::_dist(point_coor _start,point_coor _end)
   {
      if(_start.x==_end.x &&_start.y==_end.y)
	{
		return 1;
	}
	else
	{
	float diff_x=abs(_end.x-_start.x);
	float diff_y=abs(_end.y-_start.y);
	float dist= sqrt(diff_x*diff_x+diff_y*diff_y);
	return dist;
	}

   }
std::deque<point_coor> drawPath(cv::Mat &map, std::vector<MapNode *> path) {
    std::deque<point_coor> calc_path;
    point_coor _clac_points;
    // cvtColor(map, map, cv::COLOR_BGR2HSV);
    for (int i = 0; i < path.size() - 1; i++) {
        MapNode *node = path[i];
        // map.at<cv::Vec3b>(node->y, node->x) = cv::Vec3b(20 + (1.0 - ((double) i / path.size())) * 80, 200, 255);
        // std::cout << "->(" << node->x << "," << node->y << ")";
        _clac_points.y=node->y;
        _clac_points.x=node->x;

        calc_path.push_back(_clac_points);

    }
    // std::cout << std::endl;

    // cvtColor(map, map, cv::COLOR_HSV2BGR);
    // resize(map, map, cv::Size(500, 500), 0, 0, cv::INTER_NEAREST);
    return calc_path;
}

std::deque<point_coor> Laserscan_localization::plan_path( cv::Mat &_map,const cv::Point &_start,const cv::Point &_end)
  {

    std::cout<<"planner started"<<std::endl;
    // std::deque<point_coor> path_vector;


    _map_planner->map_1=_map;
    cv::Vec3b  start_color={255,0,0};
    cv::Vec3b  end_color={0,0,255};

    _map_planner->map_1.at<cv::Vec3b>(_start )= start_color;
    _map_planner->map_1.at<cv::Vec3b>(_end )= end_color;
    _map_planner->mapSize = MapSize(_map_planner->map_1.cols, _map_planner->map_1.rows);
    _map_planner->mapData = std::vector<MapNode>( _map_planner->mapSize.size);
    std::cout << "MapSize(" <<  _map_planner->mapSize.width << ", " <<  _map_planner->mapSize.height << ", " <<  _map_planner->mapSize.size << ")" << std::endl;
    _map_planner->make_nodes();
    std::cout << "startNode=(" << _map_planner->startNode->x << ", " << _map_planner->startNode->y << ")" << std::endl;
    std::cout << "endNode=(" << _map_planner->targetNode->x << ", " << _map_planner->targetNode->y << ")" << std::endl;

    _map_planner->openList.push_back(_map_planner->startNode);
    std::cout<<_map_planner->openList.size()<<std::endl;
    std::vector<MapNode *> path = _map_planner->find();
    std::cout<<"done planning"<<std::endl;

    auto path_vector=drawPath(_map_planner->map_1, path);
    set_path(path);
    cv::cvtColor(_map_planner->map_1, _map_planner->map_1, cv::COLOR_BGR2HSV);
    for (int i = 0; i < path.size() - 1; i++) {
        MapNode *node = path[i];
        _map_planner->map_1.at<cv::Vec3b>(node->y, node->x) = cv::Vec3b(20 + (1.0 - ((double) i / path.size())) * 80, 200, 255);
    }
    
    cv::cvtColor(_map_planner->map_1, _map_planner->map_1, cv::COLOR_HSV2BGR);
    // path.clear();
    _map_planner->mapData.clear();
    _map_planner->openList.clear();
    return path_vector;

  }
void Laserscan_localization::calulate_points(const sensor_msgs::msg::LaserScan::SharedPtr _scan)
{

    cv::Mat resized;
    cv::resize(map_for_planning, resized, cv::Size(500, 500), 0, 0, cv::INTER_NEAREST);
    // mapSize = MapSize(map_for_planning.cols, map_for_planning.rows);
    // mapData = std::vector<MapNode>(mapSize.size);


    ///////////////////

  _scan_data.clear();
      for(auto itr:_scan->ranges)
      {
        if(itr==inf)
        {

        }
        else
        {
        _scan_data.push_back(itr);
        }
      }

      auto points= compute_coor(_scan);
      if(ref_coor_flag)
      {
        ref_coor=points;
        save_csv_data(_scan);
        ref_coor_flag=false;
      }
      if(!ref_coor_flag)
      {   


          try
          {
          auto _vel=get_velocity();
          if(_vel.angular.z !=0)
          {
            // std::cout<<"rotating"<<std::endl;
            is_rotating=true;
          }
          else{
            // std::cout<<"stoped or moving straight"<<std::endl;
            is_rotating=false;
          }      
          // std::cout<<_vel.angular.z<<std::endl;
          }
          catch(...)
          {
            std::cout <<"failed to get cmd_vel"<<std::endl;;
          }



        if(is_rotating) 
        {
          // diff_point.x=float(ref_coor[0].x-points[0].x);
          diff_point.x=0.00;
          // diff_point.y=float(ref_coor[0].y-points[0].y);
          diff_point.y=0.00;

        }
        else
        {
          int count_values=0;
          for(auto itr:points)
          {
          count_values++;
          diff_point.x+=float(ref_coor[count_values].x-points[count_values].x);
          diff_point.y+=float(ref_coor[count_values].y-points[count_values].y);
          }
          diff_point.x=diff_point.x/count_values;
          diff_point.y=diff_point.y/count_values;


        }

      }

      auto _odom_current_position =get_delta_pose();
      for (auto itr:points)
      {
      map.at<cv::Vec3b>(convert_to_opencv_points(itr.x+_odom_current_position->pose.pose.position.x,itr.y+_odom_current_position->pose.pose.position.y)) = color;
      cv::circle(map_for_planning,convert_to_opencv_points(itr.x+_odom_current_position->pose.pose.position.x,itr.y+_odom_current_position->pose.pose.position.y),10,cv::Scalar(0, 0, 0),6);
      }


      
      


      
      // current_position=cv::Point(int((-diff_point.y)*resolution)+450,int((-diff_point.x)*resolution)+450);
      current_position=cv::Point(int((-_odom_current_position->pose.pose.position.y)*resolution)+450,int((-_odom_current_position->pose.pose.position.x)*resolution)+450);

      auto angle_offset=get_angle_offest();

      // std::cout<<"angle:= "<<angle_offset<<" cur:= "<<current_position.x<<" "<<current_position.y<<std::endl;

    // x component = _dist* cos(theta);
    // y component = _dist * sin(theta);
        auto angle_radians=angle_offset * (pi / 180);

        auto x_component_f= 11*cos(angle_radians);
        auto y_component_f= 11*sin(angle_radians);

        auto x_component_l= 7*cos(angle_radians);
        auto y_component_l= 11*sin(angle_radians);

        auto x_component_r= 7*cos(angle_radians);
        auto y_component_r= 11*sin(angle_radians);


        auto current_poi_f=cv::Point{current_position.x-int(y_component_f),(current_position.y-int(x_component_f))};
        auto current_poi_l=cv::Point{current_position.x-int(x_component_l),(current_position.y+int(y_component_l))};
        auto current_poi_r=cv::Point{current_position.x+int(x_component_r),(current_position.y-int(y_component_r))};

      cv::Point zero_point=cv::Point(int((-points[0].y)*resolution)+450,(int(-points[0].x)*resolution)+450);
      position_points_holder.push_back(current_position);


      // cv::line(map,current_position,cv::Point(target_vel_vec.x,target_vel_vec.y),cv::Scalar(0, 255, 0),1,cv::LINE_8);
      cv::line(map,current_position,current_poi_f,cv::Scalar(255, 0, 0),1,cv::LINE_8);
      cv::line(map,current_position,current_poi_l,cv::Scalar(255, 0, 0),1,cv::LINE_8);
      cv::line(map,current_position,current_poi_r,cv::Scalar(255, 0, 0),1,cv::LINE_8);
      cv::line(map,current_poi_f,current_poi_l,cv::Scalar(255, 0, 0),1,cv::LINE_8);
      cv::line(map,current_poi_f,current_poi_r,cv::Scalar(255, 0, 0),1,cv::LINE_8);




      if(position_points_holder.size()>10)
      {
        position_points_holder.clear();
      }
      else
      {
        for (auto itr:position_points_holder)
        {
            cv::circle(map,(itr),1,cv::Scalar(0, 255, 0),1);
        }
      }
        point_coor origin={450.0,450.0};
        zero_coor.x=zero_point.x;
        zero_coor.y=zero_point.y;


    auto drive_dist=_dist(origin,zero_coor);      
    geometry_msgs::msg::Twist _twist;

    if(start_planning)
    {
    const auto now = std::chrono::system_clock::now();
    const std::time_t t_c = std::chrono::system_clock::to_time_t(now);
    // std::cout << "The system clock start " << std::ctime(&t_c)<<std::endl;
    
    update_path=plan_path(map_for_planning,current_poi_f,target_point);
    auto smoothed_path_vec=smooth_path();
    if(_map_planner->draw==true)
    {
    planned_path=update_path;
    }
    
    // auto last_elem=planned_path[planned_path.size()-10];
    // cv::circle(map,cv::Point(last_elem.x,last_elem.y),3,cv::Scalar(0, 0, 255),1);

    // std::cout<<"planning done "<<planned_path.size()<<std::endl;
    const auto now1 = std::chrono::system_clock::now();
    const std::time_t t_c1  = std::chrono::system_clock::to_time_t(now1);
    // std::cout << "The system clock end" << std::ctime(&t_c1)<<std::endl;
    start_planning=false;
    }
    else
    {


    if(planned_path.size()>20)
    {
        for(auto itr:planned_path)
        {
         map.at<cv::Vec3b>(cv::Point(int(itr.x),int(itr.y))) = path_color;
        }
      auto target_goal=planned_path[10];

      // map.at<cv::Vec3b>(cv::Point(int(target_goal.x),int(target_goal.y))) = target_color;
      cv::circle(map,cv::Point(int(target_goal.x),int(target_goal.y)),1,target_color,2);
      point_coor near_point_start={-((_odom_current_position->pose.pose.position.y)*resolution)+450,-((_odom_current_position->pose.pose.position.x)*resolution)+450};
      point_coor near_point_end={target_goal.x,target_goal.y};
      auto near_point_dist= _dist(near_point_start,near_point_end);
      // std::cout<<near_point_dist<<std::endl;
      
      
      auto target_vel_vec=get_target();
      target_vel_vec.x=target_goal.x;
      target_vel_vec.y=target_goal.y;
      // target_vel_vec.y=sqrt(target_goal.y*target_goal.y-target_goal.x*target_goal.x);

  
      // point_coor target_vel_vec_coor={target_vel_vec.x,target_vel_vec,y}; 
      // _dist(target_vel_vec_coor,)
      velocity_normal.x=1;
      velocity_normal.y=0;
      velocity_normal.angle=get_angle_offest();  


      velocity_vector current_vel_vec;
     

      // change frame eof reference 
      // current_position,cv::Point(angle_vec_x,angle_vec_y)
      double angle = atan2(current_position.y - target_vel_vec.y, target_vel_vec.x - current_position.x);
      double angle_sec=angleBetweenLines(current_position.y,current_position.x,current_poi_f.y,current_poi_f.x,current_position.y,current_position.x,target_vel_vec.y,target_vel_vec.x);

      // double angle_sec = angleBetweenLines(x1, y1, x2, y2, x3, y3, x4, y4);      
      // double angle = atan2((int((-_odom_current_position->pose.pose.position.x)*resolution)) - target_vel_vec.y, target_vel_vec.x - (int((-_odom_current_position->pose.pose.position.y)*resolution))) * 180 / pi;

      
      current_vel_vec.x=(target_vel_vec.x-velocity_normal.x)*0.01-((-_odom_current_position->pose.pose.position.y)*resolution)*0.01-(5.22);
      current_vel_vec.y=(target_vel_vec.y-velocity_normal.y)*0.01-((-_odom_current_position->pose.pose.position.x)*resolution)*0.01-4.49;
      current_vel_vec.angle=angle_sec;
      
      auto angle_vec_x=0.1*cos(angle_sec);
      auto angle_vec_y=0.1*sin(angle_sec);




      cv::circle(map,cv::Point(target_vel_vec.x,target_vel_vec.y),3,cv::Scalar(0, 0, 255),1);
      // std::cout<<current_vel_vec.x<<" "<<current_vel_vec.y<<" "<<current_vel_vec.angle<<std::endl;
      // current_vel_vec.print();
      _twist.linear.x=abs(-current_vel_vec.y);
      _twist.linear.y=-current_vel_vec.x;
      _twist.angular.z=(current_vel_vec.angle);
      // std::cout<<"x:= "<<_twist.linear.x<<" z:= "<<_twist.angular.z<<std::endl;


    if(start_robot)
    {
      _twist.linear.y=0;
      if(_twist.linear.x<0.1)
      {
        _twist.linear.x=0.1;
      }
      if(_twist.linear.x>0.3)
      {
        _twist.linear.x=0.3;
      }
      publisher_->publish(_twist);

    }
    else
    {
      _twist.linear.x=0;
      _twist.linear.y=0;
      _twist.angular.z=0;
      publisher_->publish(_twist);
    }
      
      
      if(near_point_dist<3.5)
      {
        // std::remove(planned_path.begin(),planned_path.begin()+10,target_goal);
        for(int i=0;i<10;i++)
        {
        planned_path.pop_front();
        }
        start_planning=true;

      }
    }
  else
    {
      start_robot=false;
      _twist.linear.x=0;
      _twist.linear.y=0;
      _twist.angular.z=0;
      publisher_->publish(_twist);

    }
}

    
    
    /**/ 
      // cv::namedWindow("map_planning", cv::WINDOW_NORMAL);
      cv::namedWindow("map", cv::WINDOW_NORMAL);

      // cv::setMouseCallback("map_planning", CallBackFunc, NULL);
      cv::setMouseCallback("map", CallBackFunc, NULL);

      // cv::imshow("map_planning",map_for_planning);
      cv::imshow("map",map);

      auto key_vlaue=cv::waitKey(1);
      if(key_vlaue=='s')
	    {
        save_image(map);
        save_csv_data(_scan);

      }
    // cv::cvtColor(map, map, cv::COLOR_BGR2GRAY);

    // matToOccupancyGrid(map_for_planning);
    map_for_planning = cv::Mat(900,900, CV_8UC3, cv::Scalar(255,255,255));
    map = cv::Mat(900,900, CV_8UC3, cv::Scalar(255,255,255));



}
void Laserscan_localization::topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) 
{   
try
{
        calulate_points(msg);

}
catch(...)
{
  std::cout<<"some issue here and there"<<std::endl;
}
}
void Laserscan_localization::imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_data) 
{
    tf2::Quaternion q(
    imu_data->orientation.x,
    imu_data->orientation.y,
    imu_data->orientation.z,
    imu_data->orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    auto degree = yaw * (180/pi);
    if( degree < 0 ){
         degree += 360.0; // convert negative to positive angles
    }
}
void Laserscan_localization::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_data){
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


void Laserscan_localization::cmd_vel_callback( geometry_msgs::msg::Twist::SharedPtr _cmd_vel)
{

    set_velocity(*_cmd_vel);


}
void Laserscan_localization::set_velocity(geometry_msgs::msg::Twist _cmd_vel)
{

    velocity=_cmd_vel;

}

geometry_msgs::msg::Twist Laserscan_localization::get_velocity()
{

    return velocity;

}


std::vector <point_coor> Laserscan_localization::compute_coor(const sensor_msgs::msg::LaserScan::SharedPtr _scan)
  {
    _coor_data.clear();
  //   int angle=0;
  //   angle=abs(get_angle_offest());

  //   point_coor _points;
  //   // x component = _dist* cos(theta);
  //   // y component = _dist * sin(theta);
  //   auto angle_increment=_scan->angle_increment;
  // for(auto itr:_scan->ranges)
  //     {
  //       // std::cout<<angle<<std::endl;
  //       if(itr==inf)
  //       {
  //         std::cout<<"got inf:= "<<angle<<std::endl;
  //       }
  //       else
  //       {
  //       auto angle_raians=angle * (pi / 180);
  //       _points.x=itr* cos(angle_raians);
  //       _points.y=itr* sin(angle_raians);
  //       _coor_data.push_back(_points);
        
  //       }
  //       angle++;
        
  //     }
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

  void Laserscan_localization::save_image(const cv::Mat &_mat)
  {
    image_save_no++;
    std::string _path=image_save_path+std::to_string(image_save_no)+".jpg";
		cv::imwrite(_path, _mat); 
		std::cout<<"image_saved"<<std::endl;


  }

  void Laserscan_localization::save_csv_data(const sensor_msgs::msg::LaserScan::SharedPtr &_scan)
  {
    int angle_track=0;
    std::string _scan_str;
    std::string _angle_str;
    auto start_itr=_scan->ranges.begin();
    for (auto  itr:_scan->ranges)
    {
      angle_track++;
      _scan_str=_scan_str+" , "+std::to_string(itr);
      _angle_str=_angle_str+", " +std::to_string(angle_track);
    }

    if(all_data_csv.is_open()  )
    {
      all_data_csv<<"angle,"<<_angle_str<<std::endl;
      all_data_csv<<"time_stamp:=,"<<return_current_time_and_date()<<_scan_str<<std::endl;
      

    }
  }


std::string Laserscan_localization::return_current_time_and_date()
{
	auto now = std::chrono::system_clock::now();
	auto in_time_t = std::chrono::system_clock::to_time_t(now);

	std::stringstream ss;
	ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X");
	return ss.str();
}


int Laserscan_localization::get_angle_offest(){

  return angle_offset_value;
}


cv::Point Laserscan_localization::convert_to_opencv_points(float x, float y)
{
  
   return cv::Point((-y*resolution)+int(_map_size.height/2),(-x*resolution)+int(_map_size.width/2));

}

void Laserscan_localization::set_delta_pose(nav_msgs::msg::Odometry::SharedPtr odom_data)
{
_odom_data=odom_data;
// compute_velocty_from_odom(odom_data);
}

nav_msgs::msg::Odometry::SharedPtr Laserscan_localization::get_delta_pose()
{
  return _odom_data;
}



void Laserscan_localization::set_path(std::vector<MapNode *>_path)
{

  // path_calculated=_path;
}

double Laserscan_localization::angleBetweenLines(double x1a, double y1a, double x1b, double y1b, double x2a, double y2a, double x2b, double y2b) {    
    // calculate the angles of the two lines
    double angle1 = atan2(y1b - y1a, x1b - x1a);
    double angle2 = atan2(y2b - y2a, x2b - x2a);
    
    // adjust angles based on quadrants
    if (angle1 < 0) {
        angle1 += 2 * M_PI;
    }
    if (angle2 < 0) {
        angle2 += 2 * M_PI;
    }
    
    // calculate the angle between the two lines
    double angle = angle2 - angle1;
    
    // adjust angle to be between -180 and 180 degrees
    if (angle > M_PI) {
        angle -= 2 * M_PI;
    }
    if (angle < -M_PI) {
        angle += 2 * M_PI;
    }
    
    // convert angle to degrees
    double angle_deg = angle * 180 / M_PI;
    return angle;
}


nav_msgs::msg::OccupancyGrid Laserscan_localization::matToOccupancyGrid(const cv::Mat& mat)
{
    nav_msgs::msg::OccupancyGrid occupancy_grid;
    occupancy_grid.header.frame_id = "base_footprint";
    occupancy_grid.info.resolution = 0.05; // Map resolution in meters/pixel
    occupancy_grid.info.width = mat.cols; // Map width in pixels
    occupancy_grid.info.height = mat.rows; // Map height in pixels;

    
    cv::Mat image_gray = mat;   

   // convert gray image to binary image 
   // After threshold, all values are either (0 or 200)
    cv::Mat imgage_bw;
    cv::threshold(image_gray, imgage_bw, 200, 255.0, cv::THRESH_BINARY);

   // if you really want images with 0 for blocked cell and 1 for free cell
    cv::Mat image_grid = imgage_bw/255;  


    std::vector<int8_t> occupancy_data;
    for(int i = 0; i < mat.rows; i++) {
        for(int j = 0; j < mat.cols; j++) {
            occupancy_data.push_back(image_gray.at<int8_t>(i,j));
            // std::cout<<mat.at<uchar>(i, j)<<std::endl;
        }
    }

    occupancy_grid.data = occupancy_data;
    publisher_map->publish(occupancy_grid);

    return occupancy_grid;
}



void Laserscan_localization::compute_velocty_from_odom(nav_msgs::msg::Odometry::SharedPtr odom_data)
{
 
  try
  {
  time_now=return_time_milli();
  delta_t=(time_now-time_prev);
  if(prev_odom_flag==false)
  {
    prev_odom_data=odom_data;
    prev_odom_flag=true;
  }
  double delta_dist=(odom_data->pose.pose.position.x-prev_odom_data->pose.pose.position.x)*1.0001f;
  // std::cout<<"odom_data:= "<<odom_data->pose.pose.position.x<<" prev_odom_data:= "<<prev_odom_data->pose.pose.position.x<<" delta:= "<< delta_dist<<std::endl;

  auto vel_x=delta_dist/delta_t;


  std::cout<<"delta_dist:= "<<delta_dist<<" delta t:= "<<delta_t<<" vel in x dir:= "<<vel_x<<std::endl;
  // std::cout<<"time now:= "<<time_now<<"delta t:= "<<delta_t<<std::endl;
  }
  catch(...)
  {

  }

  time_prev=time_now;
  prev_odom_data=odom_data;
}


uint64_t Laserscan_localization::return_time_milli()
{

	uint64_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	return ms;

}


std::deque<point_coor> Laserscan_localization::smooth_path()
{
  int i=0;
  point_coor start_coor,end_coor;
  std::deque<point_coor> smoothed_path;
  // for(int i=0;i<update_path.size();i++)
  // {
    
    
  //   auto x0= update_path[i];
  //   auto x1= update_path[i+2];
  //   auto x2= update_path[i+4];
  //   auto x3= update_path[i+10];

  //   for(float i=0;i<1.001;i=i+0.01)
  //   {
  //   auto bezie_a=smooth(x0,x1,i);
  //   auto bezie_b=smooth(x1,x2,i);
  //   auto bezie_c=smooth(x2,x3,i);

  //   auto bezie_quad_a=smooth(bezie_a,bezie_b,i);
  //   auto bezie_quad_b=smooth(bezie_b,bezie_c,i);
  //   smoothed_path.push_back(smooth(bezie_quad_a,bezie_quad_b,i));

  //   }

  // } 


    
    int _size=update_path.size();
    auto x0= update_path[0];
    auto x1= update_path[(abs(_size/2))];
    auto x2= update_path[abs(_size/1.5)];
    auto x3= update_path[update_path.size()];

    for(float i=0;i<1.001;i=i+0.01)
    {
    auto bezie_a=smooth(x0,x1,i);
    auto bezie_b=smooth(x1,x2,i);
    auto bezie_c=smooth(x2,x3,i);

    auto bezie_quad_a=smooth(bezie_a,bezie_b,i);
    auto bezie_quad_b=smooth(bezie_b,bezie_c,i);
    smoothed_path.push_back(smooth(bezie_quad_a,bezie_quad_b,i));

    }

  
  return smoothed_path;
}

point_coor Laserscan_localization::smooth( point_coor x0,point_coor x1 ,float t)
{
  point_coor _point;
  _point.x=x0.x+(x1.x-x0.x)*t;
  _point.y=x0.y+(x1.y-x0.y)*t;

return _point;
}


