# Project Specification
## Go Chase It!

### Basic Requirements

| CRITERIA | MEETS SPECIFICATIONS |
|----------|----------------------|
| Does the submission include the my_robot and the ball_chaser ROS packages? | The student submitted all required files specified in the criteria. |
| Do these packages follow the directory structure detailed in the project description section? | The student followed the same directory structure detailed in the project description section. |

### Robot Design

| CRITERIA | MEETS SPECIFICATIONS |
|----------|----------------------|
| Does the submission include a design for a differential drive robot, using the Unified Robot Description Format? | Robot design requirements: Lidar and camera sensors, Gazebo plugins for the robot’s differential drive, lidar, and camera, housed inside the world, Significant changes from the sample taught in the project lesson,  robot is stable when moving. |
                                                                                                                                                                                                                                                                                                                                                                                                                                     
### Gazebo World

| CRITERIA | MEETS SPECIFICATIONS |
|----------|----------------------|
| Does the my_robot ROS package contain the Gazebo world? | Gazebo world requirements: Same as the world designed in the Build My World project or a new world that you design on the building editor for this project, Includes a white-colored ball |

### Ball Chasing

| CRITERIA | MEETS SPECIFICATIONS |
|----------|----------------------|
| Does the ball_chaser ROS package contain two C++ ROS nodes to interact with the robot and make it chase a white-colored ball? | `drive_bot` requirements: A `ball_chaser/command_robot` service, service accepts linear x and angular z velocities, service publishes to the the wheel joints, service returns the requested velocities. `process_image` requirements: Subscribes to the robot’s camera image, a function to analyze the image and determine the presence and position of a white ball, requests a service to drive the robot towards a white ball (when present). | 

### Launch Files                
                                      
| CRITERIA | MEETS SPECIFICATIONS |   
|----------|----------------------|   
| Does the submission include `world.launch` and `ball_chaser.launch` files that launch all the nodes in this project? | `world.launch` requirements: Launch the world (which includes a white ball), launch the robot. `ball_chaser.launch` requirements: Run the `drive_bot` C++ node, run the `process_image` C++ node. |
