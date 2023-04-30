#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <memory>
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include "Laserscan_localization.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
using std::placeholders::_1;


int main(int argc, char * argv[])
{
  
  
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<Laserscan_localization>());

  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
    rclcpp::spin(std::make_shared<Laserscan_localization>());

  }
  
  rclcpp::shutdown();
  return 0;
}