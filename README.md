# A Simulated Implementation of the IGVC 2018 Auto Nav Course


Contributors: 
- Sachidanand VS ep19b010@smail.iitm.ac.in
- Suraj Rathi me19b177@smail.iitm.ac.in

## OVERVIEW:

The main steps in our project were the following:

1.  Select and place sensors on the robot.
2.  Detect obstacles.
3.  Detect Lane markings
4.  Path through the course to various gps waypoints

The idea is to turn the lanes into a set of points which the path planning can treat like an obstacle. This white pixels in the lane are converted to a point cloud by computing the coordinates of point and angle from the camera and projecting it on the ground then the point cloud is sent as a ROS PointCloud2 message to the move\_base. Path Planning is done by ROS Navigation Stack’s “move\_base” node. The map is created from sensor reading from laser sensor and camera which detects lane. The global and local maps are layered so the vehicle is simultaneously aware of its location on each. The sensor’s used are 1 laser scanner, 2 camera’s for each lane ,1 IMU sensor ,1 GPS sensor.

## SENSORS USED:
1. 2-D Lidar : Mounted at the front of the robot to detect solid obstacles.
2. RGB Camera: Two such cameras are mounted in an elevated position on the front of the robot.
Required for lane detection
3. GPS Unit: Mounted near the core of the robot
4. IMU with 6 DOF: For more accurate odometry

#### 2-D lidar sensor:
* Range : 5-8 meters
* Angle : 360 degree
* Quantity : 1
* Min_Scan_frequency: 10 Hz
* Model: Sick LD-LRS3600. no price shown

#### CAMERA :
* Rate: 30 fps
* Type: RGB image
* Angle : 120 degree and above is ok
* Model : AUKEY DR01
* Price : Rs . 6000

#### GPS SENSOR:
* Price : Rs.1925.00
* Model: Radiolink M8N GPS Module UBX-M8030

#### IMU SENSOR:
* DOF : 6
* Model : 6-axis Rate Gyro BMI160 Gravity Accelerometer Sensor Module IIC SPI Br
* Price : Rs.1290
we are not sure about the model but the specification mentioned are the requirement.

## LOCALISATION:

We were working with a gazebo simulation, so our odom source was perfectly precise.

However, as a real robot will require better odometry, we looked into the *Extended Kalaman Filter *based sensor fusion of GPS, IMU, and Encoder data.

## LANE DETECTION:

We pass the received image through a set of opencv based filters

1.  Gaussian Blur - smoothen out unnecessary details.
2.  In Range - select pixel from HSV colorspace.
3.  Flood Fill - this removes gazebos sky
4.  Rectangular Mask - removes the upper 60% of the image as it is not part of the ground and the resolution in meters on the ground per pixel is very low.
5.  Erosion - Sharpen the lanes and remove noise.
6.  MedianBlur - also for reducing noise

Then the points go through a set of geometric transforms:

1.  Projection through the camera - Each selected pixel is projected as a ray through a pinhole camera style model
2.  Transformation to ground frame - The rays are shifted and rotated to the “odom” frame which is continuous with time.
3.  Extension of the ray - The ray is then extended until it touches the ground (odom.z = 0)

These points are then published as a sensor\_msgs::PointCloud2 .

In the costmap, the lane appears as lethal obstacles.

We had used dynamic\_reconfigeration to tune the parameters of the different opencv filters.

Scope for Improvement:

1.  Down scale the image before processing.
2.  Use the laser based data to ignore sections of the image containing obstacles.
3.  Detect the lanes themself as a continuous series of line segments.

The third point was attempted with partial success; however, it was deemed unnecessary for the task at hand based on success with simpler methods and time constraints. [See Here:](https://github.com/surajRathi/abhiyaan-team-s/blob/line_lanes/src/lanes/lanes_mono.cpp)

## PATH PLANNING:

We use the ROS Navigation Stack’s “*move\_base*” node using the following planners:

-   *global\_planner/GlobalPlanner *- based on the global costmap.
-   *base\_local\_planner/TrajectoryPlannerROS *- based on the local costmap.

## COSTMAP GENERATION:

We used a layer costmap as provided through the costmap\_2d package.

We use a global costmap with the same size as the course.

The local costmap is a 10x10 meter moving window costmap.

The costmap is generated by combining the obstacle detection and an obstacle inflation layer, which discourages pathing closer to the obstacles.

The obstacles consist of the laser scan, lane detection, and a special ‘fake’ obstacle.

**‘Fake’ obstacle** - When following the given course, the robot doesn’t need to go backwards. To stop the global planner from pathing in that direction, we publish a laser scan which extends in an ~100° arc behind the robot. This is not published when the robot is executing recovery behaviors (has a forward speed less than a threshold).

## SENDING GOAL POSITIONS:

In the actual challenge, goal positions are provided to the robot as GPS locations. Converting gps to world frame coordinates is a trivial task.

So, for our project we found it simpler to directly provide map frame coordinates. They are provided to the robot one by one through the *move\_base *actionlib interface.

## RESULT:

Our robot is able to traverse most of the course and gets stuck near the last 5 meters.

Points of Failure:

-   Sometimes, parts of the lane are not detected
-   At the smaller gaps, the robot may get stuck (especially near (0,-22).
