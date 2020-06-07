#include <sensor_msgs/Image.h>

#include "ball_chaser/DriveToTarget.h"
#include "ros/ros.h"

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float linear_x, float angular_z) {
  ROS_INFO_STREAM("Driving robot to the ball.");
  ball_chaser::DriveToTarget srv;
  srv.request.linear_x = linear_x;
  srv.request.angular_z = angular_z;

  if (!client.call(srv)) {
    ROS_ERROR("Failed to call service DriveToTarget.");
  }
}

void stop_robot() {
  float linear_x = 0.0;
  float angular_z = 0.0;
  drive_robot(linear_x, angular_z);
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img) {
  int white_pixel = 255;
  int height = img.height;
  int step = img.step;

  float offset_accumulated = 0;
  int count_total = 0;

  for (int i = 0; i < height; i++) {
    for (int j = 0; j < step; j++) {
      if (img.data[i * step + j] == white_pixel) {
        offset_accumulated += j - step / 2.0;
        count_total++;
      }
    }
  }

  if (count_total == 0) {
    stop_robot();
  } else {
    float linear_x = 0.1;
    float angular_z = -4.0 * offset_accumulated / count_total / (step / 2.0);
    drive_robot(linear_x, angular_z);
  }
}

int main(int argc, char** argv) {
  // Initialize the process_image node and create a handle to it
  ros::init(argc, argv, "process_image");
  ros::NodeHandle n;

  // Define a client service capable of requesting services from command_robot
  client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

  // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
  ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

  // Handle ROS communication events
  ros::spin();

  return 0;
}
