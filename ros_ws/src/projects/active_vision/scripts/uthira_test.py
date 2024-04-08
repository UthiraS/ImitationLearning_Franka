#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def send_effort_joint_command():
    # Initialize the ROS node
    rospy.init_node('franka_effort_joint_command_sender', anonymous=True)

    # Create a publisher object
    pub = rospy.Publisher('/effort_joint_trajectory_controller/command', JointTrajectory, queue_size=10)

    # Wait for the publisher to connect to subscribers
    rospy.sleep(1.0)

    # Create a JointTrajectory message
    trajectory = JointTrajectory()
    trajectory.joint_names = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]

    point = JointTrajectoryPoint()
    #  <joint name="panda_joint1" value="0" />
    #   <joint name="panda_joint2" value="-0.785" />
    #   <joint name="panda_joint3" value="0" />
    #   <joint name="panda_joint4" value="-2.356" />
    #   <joint name="panda_joint5" value="0" />
    #   <joint name="panda_joint6" value="1.571" />
    #   <joint name="panda_joint7" value="0.785" />
    point.positions = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]  # Replace with your desired joint angles
    point.effort = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0]  # Replace with your desired joint effort values
    point.time_from_start = rospy.Duration(1.0)  # Move to the specified position in 1 second

    trajectory.points.append(point)

    # Publish the trajectory
    pub.publish(trajectory)

    # Sleep to allow for the message to be published.
    rospy.sleep(1.0)

if __name__ == "__main__":
    send_effort_joint_command()
