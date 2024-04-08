#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

def send_velocity_command():
    
    rospy.init_node('franka_velocity_command_sender', anonymous=True)

    # Create a publisher object
    pub = rospy.Publisher('/joint_group_velocity_controller/command', Float64MultiArray, queue_size=10)

    # Wait for the publisher to connect to subscribers
    rospy.sleep(1.0)

    # Create a Float64MultiArray message
    velocity_array = Float64MultiArray()

   
    # Fill out the data for the velocities
    velocity_array.data = [0.0, 0.0, 0.0, -0.1, 0.0, 0.0, 0.0]  # Replace with your desired joint velocities

    # Publish the velocity_array
    pub.publish(velocity_array)

    # Sleep to allow for the message to be published.
    rospy.sleep(1.0)

if __name__ == "__main__":
    send_velocity_command()