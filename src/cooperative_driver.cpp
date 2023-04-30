#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <iostream>
#include <iostream>
#include <atomic>
#include <thread>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp> //for basic rclcpp functionalites




using namespace std::chrono_literals;
void planner_exectutor(moveit::planning_interface::MoveGroupInterface &_interface_grp,moveit::planning_interface::MoveGroupInterface::Plan _plan)
{
//got plan now execute time 

moveit::planning_interface::MoveItErrorCode err=_interface_grp.asyncExecute(_plan);
std::cout<<"done executing"<<std::endl;
// return err;
}

int main(int argc, char **argv)
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  // auto const node = std::make_shared<rclcpp::Node>("controller", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto const node = std::make_shared<rclcpp::Node>("controller");


  // Create a ROS logger
  auto const logger = rclcpp::get_logger("controller");
  RCLCPP_INFO(logger,"Started");





  moveit::planning_interface::MoveGroupInterface dual_arm_group(node,"co_manipulator");
  moveit::planning_interface::MoveGroupInterface ur_1_group(node,"ur1/ur_manipulator");
  moveit::planning_interface::MoveGroupInterface ur_2_group(node,"ur2/ur_manipulator");



  dual_arm_group.setStartStateToCurrentState();
  geometry_msgs::msg::PoseStamped ur_1_pose_goal;
  geometry_msgs::msg::PoseStamped ur_2_pose_goal;

  auto ur_1_pose= ur_1_pose_goal.pose;
  auto ur_2_pose= ur_2_pose_goal.pose;


  ur_1_pose_goal.header.frame_id = "base_link";
  ur_2_pose_goal.header.frame_id = "base_link_1";


  ur_1_pose.position.x = 0.212;
  ur_1_pose.position.y = 0.015;
  ur_1_pose.position.z = 0.014;

  ur_1_pose.orientation.x = 0;
  ur_1_pose.orientation.y = 0;
  ur_1_pose.orientation.z = 0;
  ur_1_pose.orientation.w = 1;


  ur_2_pose.position.x = 0.28;
  ur_2_pose.position.y = -0.12;
  ur_2_pose.position.z = 0.2;

  ur_2_pose.orientation.x = 0;
  ur_2_pose.orientation.y = 0;
  ur_2_pose.orientation.z = 0;
  ur_2_pose.orientation.w = 1.0;





  moveit::planning_interface::MoveGroupInterface::Plan my_plan_1;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_2;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_co;

  std::string ur_1_eff = "tool0";
  std::string ur_2_eff = "tool1";






  // Plan with first joint group
  ur_1_group.clearPoseTargets();
  ur_1_group.setPoseReferenceFrame("base_link");
  ur_1_group.setPoseTarget(ur_1_pose, ur_1_eff);
  // ur_1_group.asyncMove();

  ur_2_group.clearPoseTargets();
  ur_2_group.setPoseReferenceFrame("base_link_1");
  ur_2_group.setPoseTarget(ur_2_pose, ur_2_eff);
  // ur_2_group.asyncMove();

  auto res = ur_1_group.plan(my_plan_1);
  bool success = (res == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (!success) {
         RCLCPP_INFO(logger, "failed: Move 1 action failed");
         return 0;
  }

   res = ur_2_group.plan(my_plan_2);
   success = (res == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (!success) {
         RCLCPP_INFO(logger, "failed: Move 2 action failed");
         return 0;
  }



  moveit_msgs::msg::RobotTrajectory joint_names_1= my_plan_1.trajectory_;
  auto string_1=joint_names_1.joint_trajectory.joint_names;

  moveit_msgs::msg::RobotTrajectory joint_names_co= my_plan_co.trajectory_;
  auto string_co=joint_names_co.joint_trajectory.joint_names;

  // my_plan_co.trajectory_.joint_trajectory.joint_names.push_back(string_1);

  std::vector<std::string> joint_names_holder;

 for(int i=0;i<string_1.size();i++)
 {
      // std::cout<<"planner 1:= "<<string_1[i]<<std::endl;
      // my_plan_co.trajectory_.joint_trajectory.joint_names[i]=string_1[i];
      joint_names_holder.push_back(string_1[i]);
 }

  moveit_msgs::msg::RobotTrajectory joint_names_2= my_plan_2.trajectory_;
  auto string_2=joint_names_2.joint_trajectory.joint_names;
  my_plan_co.trajectory_.joint_trajectory.joint_names;
 
 for(int i=0;i<string_2.size();i++)
 {
      // std::cout<<"planner 2:= "<<string_2[i]<<std::endl;
      joint_names_holder.push_back(string_2[i]);

 }

  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points_holder;
  std::vector <std::pair <std::string,double>> ponits_mapper;
  std::vector <std::pair <std::string,double>>::iterator ponits_mapper_itr;
  auto joint_points_1= my_plan_1.trajectory_.joint_trajectory.points;

  auto joint_points_2= my_plan_2.trajectory_.joint_trajectory.points;

for(int l=0;l<joint_points_1.size();l++)
 {
    points_holder.push_back(joint_points_1[l]);
    my_plan_co.trajectory_.joint_trajectory.points.push_back(joint_points_1[l]);
    
 }

 for(int j=0;j<joint_points_2.size();j++)
 {
    points_holder.push_back(joint_points_2[j]);
    // my_plan_1.trajectory_.joint_trajectory.points.push_back(joint_points_2[j]);
    my_plan_co.trajectory_.joint_trajectory.points.push_back(joint_points_2[j]);

 }
std::cout<<"points copied "<<points_holder.size()<<std::endl;
std::vector<double >points_with_sequence;
for(int points_index=0;points_index<my_plan_1.trajectory_.joint_trajectory.points.size();points_index++)
{
 std::cout<< my_plan_1.trajectory_.joint_trajectory.points.size()<<std::endl;
 std::cout<< my_plan_1.trajectory_.joint_trajectory.points[points_index].positions.size()<<std::endl;
 for(int angle_itr=0;angle_itr<my_plan_1.trajectory_.joint_trajectory.points[points_index].positions.size();angle_itr++)
 {
  // std::cout<<"joint name:= "<<my_plan_1.trajectory_.joint_trajectory.joint_names[angle_itr]<<std::endl;
  // std::cout<<"angle value:= "<<my_plan_1.trajectory_.joint_trajectory.points[points_index].positions[angle_itr]<<std::endl;
  points_with_sequence.push_back(my_plan_1.trajectory_.joint_trajectory.points[points_index].positions[angle_itr]);
  ponits_mapper.push_back(std::make_pair(my_plan_1.trajectory_.joint_trajectory.joint_names[angle_itr],my_plan_1.trajectory_.joint_trajectory.points[points_index].positions[angle_itr]));
 }
}

for(int points_index=0;points_index<my_plan_2.trajectory_.joint_trajectory.points.size();points_index++)
{
 std::cout<< my_plan_2.trajectory_.joint_trajectory.points.size()<<std::endl;
 std::cout<< my_plan_2.trajectory_.joint_trajectory.points[points_index].positions.size()<<std::endl;
 for(int angle_itr=0;angle_itr<my_plan_2.trajectory_.joint_trajectory.points[points_index].positions.size();angle_itr++)
 {
  // std::cout<<"joint name:= "<<my_plan_2.trajectory_.joint_trajectory.joint_names[angle_itr]<<std::endl;
  // std::cout<<"angle value:= "<<my_plan_2.trajectory_.joint_trajectory.points[points_index].positions[angle_itr]<<std::endl;
  points_with_sequence.push_back(my_plan_2.trajectory_.joint_trajectory.points[points_index].positions[angle_itr]);
  ponits_mapper.push_back(std::make_pair(my_plan_2.trajectory_.joint_trajectory.joint_names[angle_itr],my_plan_2.trajectory_.joint_trajectory.points[points_index].positions[angle_itr]));

 }
}


for(ponits_mapper_itr=ponits_mapper.begin();ponits_mapper_itr!=ponits_mapper.end();ponits_mapper_itr++)
{
  std::cout<<"joint name:= "<<ponits_mapper_itr->first<<" angle:= "<<ponits_mapper_itr->second<<std::endl;
  dual_arm_group.setJointValueTarget(ponits_mapper_itr->first,ponits_mapper_itr->second);

}
auto err=dual_arm_group.move();
switch (err.val) // there are various move failures, some are covered here. Rest can be added in future
      {
      case moveit::planning_interface::MoveItErrorCode::SUCCESS:
         RCLCPP_INFO(logger, "Success: Move action Successful");
         rclcpp::shutdown();
         return true;
         break;

      case moveit::planning_interface::MoveItErrorCode::FAILURE:
         RCLCPP_INFO(logger, "Failure: Move action failed");
         break;
      case moveit::planning_interface::MoveItErrorCode::GOAL_IN_COLLISION:
         RCLCPP_INFO(logger, " *** Warning: Cant move, goal pose in collision ****");
         break;
      case moveit::planning_interface::MoveItErrorCode::INVALID_GROUP_NAME:
         RCLCPP_INFO(logger, "Failure: Invalid group name supplied! ");
         break;
      default:
         RCLCPP_INFO(logger, "Failure: Can't move, unknown failures ");
         break;
      }
 
rclcpp::spin(node);

}









