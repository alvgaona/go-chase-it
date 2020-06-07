#include "ball_chaser/DriveToTarget.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

// ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;

// This function should publish the requested linear x and angular velocities to the robot wheel joints
// After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities
bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res) {
  // Create a motor_command object of type geometry_msgs::Twist
  geometry_msgs::Twist motor_command;
  // Set wheel velocities
  motor_command.linear.x = req.linear_x;
  motor_command.angular.z = req.angular_z;
  // Publish angles to drive the robot
  motor_command_publisher.publish(motor_command);

  std::stringstream ss;
  ss << "Published velocities - linear_x: " << std::to_string(static_cast<double>(motor_command.linear.x))
     << ", angular_z: " << std::to_string(static_cast<double>(motor_command.angular.z));

  res.msg_feedback = ss.str();
  ROS_INFO_STREAM(res.msg_feedback);
}

int main(int argc, char** argv) {
  // Initialize a ROS node
  ros::init(argc, argv, "drive_bot");

  // Create a ROS NodeHandle object
  ros::NodeHandle n;

  // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic
  // with a publishing queue size of 10
  motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  // Subscribe to /ball_chaser/command_robot topic to read the requested wheel velocities
  ros::ServiceServer srv1 = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);
  ros::spin();

  return 0;
}
