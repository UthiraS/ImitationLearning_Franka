#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Int32

bridge = CvBridge()

goalX = 0
goalY = 0

img_pub = rospy.Publisher("vsbot/vis", Image, queue_size=1)

r = rospy.get_param("vsbot/estimation/rate")

traj_pts_x = []
traj_pts_y = []

status = -2

def statusCallback(msg):
    global status
    status = msg.data


def eePosCallback(msg):
    global traj_pts_x, traj_pts_y, status

    if status == 2:
        traj_pts_x.append(int(msg.data[0]))
        traj_pts_y.append(int(msg.data[1]))


def visCallback(msg):
    # print("Received img")
    global bridge, goalX, goalY, img_pub, rate
    # rate = rospy.Rate(r)
    try:
        cv_img = bridge.imgmsg_to_cv2(msg,"bgr8")
    except CvBridgeError as e:
        print(e)
    
    
    
    if status == 2:
        # draw goal marker if servoing
        cv_img = cv2.circle(cv_img,(goalX, goalY), radius = 3, color=(0,255,0), thickness=2)

        # draw trajectory if servoing
        for i in range(len(traj_pts_x) - 1):
            start_pt = (traj_pts_x[i], traj_pts_y[i])
            end_pt = (traj_pts_x[i+1], traj_pts_y[i+1])
            cv_img = cv2.line(cv_img, start_pt, end_pt,(0,0,255), 2)

    # convert cv image to ROS msg
    try:
        ros_img = bridge.cv2_to_imgmsg(cv_img,"bgr8")
    except CvBridgeError as e:
        print(e)

    img_pub.publish(ros_img)
    # rate.sleep()






def main(args):
    global goalX, goalY, img_pub
    # Initialize ROS
    rospy.init_node('visualizer')
    print("Initialized vis")
    # Read goal posn from YAML
    goalX = rospy.get_param("vsbot/control/goal_pos_x")
    goalY = rospy.get_param("vsbot/control/goal_pos_y")

    # Initialize subscribers
    img_sub = rospy.Subscriber("vsbot/camera1/image_raw", Image, visCallback, queue_size = 1)
    ee_sub = rospy.Subscriber("marker_segmentation", Float64MultiArray, eePosCallback, queue_size=1)
    status_sub = rospy.Subscriber("vsbot/status", Int32, statusCallback, queue_size = 1)

    # publish robot trajectory
    # publish cartesian velocity vector
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == "__main__":
    main(sys.argv)
