#include <sensor_msgs/Image.h>

#include "ball_chaser/DriveToTarget.h"
#include "ros/ros.h"

struct Pixel {
  int r;
  int g;
  int b;

  bool is_white() {
    int white = 255;
    return r == white && g == white && b == white;
  }
};

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
  Pixel pixel;
  int count_total = 0;
  int x_position = 0;

  for (int i = 0; i + 2 < img.data.size(); i++) {
    pixel.r = img.data[i];
    pixel.g = img.data[i + 1];
    pixel.b = img.data[i + 2];

    if (pixel.is_white()) {
      x_position += (i % (img.width * 3)) / 3;
      count_total++;
    }
  }

  if (count_total == 0) {
    stop_robot();
    return;
  }

  int mean_x_position = x_position / count_total;

  float linear_x;
  float angular_z;

  if (mean_x_position < img.width / 3) {
    linear_x = 0.5;
    angular_z = 0.5;
  } else if (mean_x_position > img.width * 2 / 3) {
    linear_x = 0.5;
    angular_z = -0.5;
  } else {
    linear_x = 0.5;
    angular_z = 0.0;
  }

  drive_robot(linear_x, angular_z);
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
