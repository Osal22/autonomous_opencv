#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "String.h"
#include "Motors.h"

#include <motor_data_msgs/msg/motors.hpp>
#include <ros/ros.h>
#include <geometry_msgs/msg/twist.hpp>
using namespace std::chrono_literals;

/*TODO create cmd_vel subscriber in ros2 and than do inverse kinematics create motor data msg publihser 
  in ros and publish all those commands to ros topic which is than subscribed by arduino*/
class ArduinoPublisher : public rclcpp::Node
{
public:
  ArduinoPublisher() : Node("arduino_publihser") {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&ArduinoPublisher::topic_callback, this, std::placeholders::_1));
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "arduino_publihser");
    ros::NodeHandle nh;
    chatter_pub = nh.advertise<std_msgs::String>("chatter_ros1", 1000);
    motors_pub = nh.advertise<motors_data_msgs_ros1::Motors>("arduino_pub", 1000);

  }

private:

  void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
    motors_data_msgs_ros1::Motors _slam_bot_motors;
    // RCLCPP_INFO(this->get_logger(), "linear x: '%f', linear y: '%f', linear z: '%f'", msg->linear.x, msg->linear.y, msg->linear.z);
    // auto angle_c  = msg->angular.z;
    // auto x_c      = msg->linear.x;
    // auto y_c      = msg->linear.y;
    // auto  wsum=angle_c*(lx+ly);
    float radius=0.02f;
    auto  omega = msg->angular.z*3.0;
    // auto V= sqrt(msg->linear.x*msg->linear.x+msg->linear.y*msg->linear.y);
    auto V= msg->linear.x;
    float V1 = V - omega * radius;  // Front-right wheel
    float V2 = V + omega * radius;  // Front-left wheel
    float V3 = V + omega * radius;  // Rear-left wheel
    float V4 = V - omega * radius;  // Rear-right wheel
    // w1=int(1/rad*(x_c-y_c-wsum)*20.0f);
    // w2=int(1/rad*(x_c+y_c+wsum)*20.0f);
    // w3=int(1/rad*(x_c+y_c-wsum)*20.0f);
    // w4=int(1/rad*(x_c-y_c+wsum)*20.0f);
    w1=int(V1*20.0f);
    w2=int(V2*20.0f);
    w3=int(V3*20.0f);
    w4=int(V4*20.0f);
    RCLCPP_INFO(this->get_logger(), "w1: '%i', w2: '%i', w3: '%i',w4: '%i' ", w1, w2, w3, w4);
    // direction set of all motors
    if(w1<0) {
    _slam_bot_motors.dir1=0;}
    else{
    _slam_bot_motors.dir1=1;}

    if(w2<0) {
    _slam_bot_motors.dir2=0;}
    else {
    _slam_bot_motors.dir2=1;}
    if(w3<0) {
    _slam_bot_motors.dir3=0;}
    else {
    _slam_bot_motors.dir3=1;}

    if(w4<0) {
    _slam_bot_motors.dir4=0;}
    else{
    _slam_bot_motors.dir4=1;}
      RCLCPP_INFO(this->get_logger(), "w1: '%i', w2: '%i', w3: '%i',w4: '%i' ", _slam_bot_motors.dir1, _slam_bot_motors.dir2, _slam_bot_motors.dir3, _slam_bot_motors.dir4);
    upper_limit(w1);
    upper_limit(w2);
    upper_limit(w3);
    upper_limit(w4);
    _str_msg.data=str;
    _slam_bot_motors.motor1=abs(w1);
    _slam_bot_motors.motor2=abs(w2);
    _slam_bot_motors.motor3=abs(w3);
    _slam_bot_motors.motor4=abs(w4);

    motors_pub.publish(_slam_bot_motors);
    chatter_pub.publish(_str_msg);
  }
  void upper_limit(int& val) {
    if(val>250) {
      val=250;
    }
  }
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
  ros::Publisher chatter_pub;
  ros::Publisher motors_pub;
  double w_r = 0, w_l = 0;
  double wheel_rad = 0.0325, wheel_sep = 0.295;
  std_msgs::String _str_msg;
  std::string str="get ready motors";
  int w1=0,w2=0,w3=0,w4=0;
  float wsum=0.0,lx=0.15,ly=0.25,rad=1.0;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArduinoPublisher>());
  rclcpp::shutdown();
  return 0;
}