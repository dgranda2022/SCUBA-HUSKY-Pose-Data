## README

### Overview

This code records position data from an OptiTrack camera system. Although it was developed for a Clearpath HUSKY A200, it is not specific to that platform and can be adapted for any system that publishes pose data in a similar  format. 

### Dependencies

- ROS Noetic (with Python3)

- ROS Packages: rospy, geometry_msgs, and tf

Make sure to install these and that your ROS workspace is properly set up.

### Setup

1. Create a Catkin Workspace:
    If you haven't yet, create and initialize your workspace:
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
```
2. Create/Place the Package:
    - Place your package (for me: husky_pose_rec) in the ~/catkin_ws/src folder/.
    - Ensure package has a valid package.xml and CMakeLists.txt. 
    
4. Build the Workspace:
   - From the workspace root:

   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash

   ```
### Usage

1. Modify the Topic Name:
    - In the code, replace **caleb_pose_topic** with the actual topic name that publishes pose data
3. Run the Node:
    - From the terminal run:
```bash
rosrun husky_pose_recorder pose_recorder.py

```
- (Ensure the package name matches your actual package folder name.)

### How it Works

- Subscription
    - The code subscribes to a ROS topic that publishes pose data as quaternions
  
- Conversion
    - It converts the quaternions (a 4-number representation of orientation) to Euler angles (roll, pitch yaw) using the ``` tf ``` library. We only use the yaw for the HUSKY
  
- Data Logging
    - The program writes the pose data (timestamp, x, y, z, yaw) to an output file (pose_data.txt) in the same directory. Data is appended continuously as new messages arrive. 
  
### Testing without Lab data
- If you don't have live pose data available, you can simulate data using:
```bash
rostopic pub /caleb_pose_topic geometry_msgs/PoseStamped "{header: {stamp: now, frame_id: 'base_link'}, pose: {position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}" -r 1

```
- This command publishes dummy pose data at a rate of one message per second, allowing you to verify that your node records the data correctly.  





   
