#!/usr/bin/env python3
# This ROS node subscribes to a pose topic and saves position and orientation data to a text file.

import rospy                       # Import the rospy library to interact with ROS.
from geometry_msgs.msg import PoseStamped   # Import the PoseStamped message type.
import tf                        # Import the tf package for converting quaternions to Euler angles.
import os                        # Import the os module for file path operations.

# Define the output file path; the file will be saved in the current working directory.
OUTPUT_FILE = os.path.join(os.getcwd(), "pose_data.txt")

def quaternion_to_yaw(orientation_q):
    """
    Convert a quaternion (orientation) into a yaw angle.
    :param orientation_q: A quaternion with fields x, y, z, w.
    :return: The yaw (rotation about the Z-axis) in radians.
    """
    # Create a tuple from the quaternion components.
    quaternion = (
        orientation_q.x,
        orientation_q.y,
        orientation_q.z,
        orientation_q.w
    )
    # Convert the quaternion into Euler angles (roll, pitch, yaw) using tf.
    euler = tf.transformations.euler_from_quaternion(quaternion)
    # Return the yaw angle (third element in the Euler tuple).
    return euler[2]

def pose_callback(msg):
    """
    Callback function that processes each incoming PoseStamped message.
    It extracts the position (x, y, z), computes the yaw angle (theta),
    and writes these values along with a timestamp to a text file.
    
    :param msg: The PoseStamped message received from the topic.
    """
    # Extract the x, y, z coordinates from the position part of the message.
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z
    
    # Convert the orientation quaternion to a yaw angle (theta).
    theta = quaternion_to_yaw(msg.pose.orientation)
    
    # Get the time stamp from the message header.
    # If the header stamp is not available, use the current ROS time.
    time_stamp = msg.header.stamp.to_sec() if msg.header.stamp else rospy.get_time()
    
    # Format the data as a comma-separated line: time, x, y, z, theta.
    data_line = "{:.3f},{:.4f},{:.4f},{:.4f},{:.4f}\n".format(time_stamp, x, y, z, theta)
    
    # Open the output file in append mode and write the data line.
    with open(OUTPUT_FILE, "a") as f:
        f.write(data_line)
    
    # Log the recorded data to the ROS console for debugging.
    rospy.loginfo("Recorded pose: " + data_line.strip())

def listener():
    """
    Initializes the ROS node, subscribes to the pose topic, and keeps the node running.
    """
    # Initialize the node with the name 'pose_recorder'. The 'anonymous=True'
    # option allows multiple nodes with the same name to run without conflict.
    rospy.init_node('pose_recorder', anonymous=True)
    
    # Subscribe to the topic that publishes PoseStamped messages.
    # Replace "caleb_pose_topic" with the actual topic name used by Caleb's node.
    rospy.Subscriber("caleb_pose_topic", PoseStamped, pose_callback)
    
    # Inform the user that the node has started and where the data will be saved.
    rospy.loginfo("Pose recorder node started. Writing data to: " + OUTPUT_FILE)
    
    # Keep the node active, waiting for incoming messages.
    rospy.spin()

if __name__ == '__main__':
    # When this script is executed, start the listener function.
    listener()
