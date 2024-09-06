#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <math.h>

int main(int argc, char * argv[])
{
  
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "move_program", 
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto const logger = rclcpp::get_logger("move_program");

  moveit::planning_interface::MoveGroupInterface move_group_interface(node, "panda_arm");

  geometry_msgs::msg::Pose goal_pose;

  //Quaternion

  geometry_msgs::msg::Quaternion msg_quat;
  msg_quat.x = 0.0;
  msg_quat.y = 0.0;
  msg_quat.z = sin(-M_PI / 4);  // half-angle for pitch (yaw rotation of -M_PI / 2)
  msg_quat.w = cos(-M_PI / 4);  // half-angle for pitch (yaw rotation of -M_PI / 2)

  goal_pose.orientation = msg_quat;
  goal_pose.position.x = 0.3;
  goal_pose.position.y = 0.0;
  goal_pose.position.z = 0.3;

  move_group_interface.setPoseTarget(goal_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan1;
  auto const success = static_cast<bool>(move_group_interface.plan(plan1));

  // Execute the plan if planning was successful
  if (success)
  {
    move_group_interface.execute(plan1);
    RCLCPP_INFO(logger, "Motion plan successfully executed!");
  }
  else
  {
    RCLCPP_ERROR(logger, "Motion planning failed!");
  }

  // Shutdown the ROS2 node
  rclcpp::shutdown();
  return 0;
}
