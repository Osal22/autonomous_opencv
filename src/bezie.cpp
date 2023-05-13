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


auto map = cv::Mat(900,900, CV_8UC3, cv::Scalar(255,255,255));
cv::Vec3b  color={0,0,0};
    float t=0.1;
    std::vector<cv::Point2f>path_bezie;
    cv::Point x0={0,450};
    cv::Point x1={450,700};
    cv::Point x2={900,300};
    cv::Point x3={900,450};

cv::Point2f smooth( cv::Point2f x0,cv::Point2f x1 ,float t)
{
return x0+(x1-x0)*t;
}

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
     if  ( event == cv::EVENT_LBUTTONDOWN )
     {
     
        x0={x,y};
	 }
     else if  ( event == cv::EVENT_RBUTTONDOWN )
     {
        x1={x,y};
        path_bezie.clear();
        map = cv::Mat(900,900, CV_8UC3, cv::Scalar(255,255,255));


     }
     else if  ( event == cv::EVENT_MBUTTONDOWN )
     {
          
     }
     else if ( event == cv::EVENT_MOUSEMOVE )
     {
       
    for(float i=0;i<1.001;i=i+0.01)
    {
        x2={x,y};
        auto bezie_a=smooth(x0,x1,i);
        auto bezie_b=smooth(x1,x2,i);
        auto bezie_c=smooth(x2,x3,i);

        auto bezie_quad_a=smooth(bezie_a,bezie_b,i);
        auto bezie_quad_b=smooth(bezie_b,bezie_c,i);
        path_bezie.push_back(smooth(bezie_quad_a,bezie_quad_b,i));
    }
    for(auto itr:path_bezie)
    {
        // map.at<cv::Vec3b>(itr) = color;
        
        cv::circle(map,itr,1,cv::Scalar(0, 0, 0),1);
    }

     }
}


int main()
{
    cv::namedWindow("bezie", cv::WINDOW_NORMAL);

    while(true)
    {
    cv::line(map,x0,x1,cv::Scalar(0,0,255),1,8,0);
    cv::line(map,x1,x2,cv::Scalar(255,0,255),1,8,0);
    cv::line(map,x2,x3,cv::Scalar(255,50,255),1,8,0);
    cv::line(map,x0,x3,cv::Scalar(20,255,50),1,8,0);


    cv::setMouseCallback("bezie", CallBackFunc, NULL);      
    cv::imshow("bezie",map);
    cv::waitKey(2);

    


    }


    return 0;
}

