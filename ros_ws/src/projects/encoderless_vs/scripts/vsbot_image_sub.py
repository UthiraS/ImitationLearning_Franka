#!/usr/bin/env python3
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

image_pub = rospy.Publisher('image_publisher', Image,  queue_size =1)

 # meaning 30 message published in 1 sec    
# Flag it for 
def image_callback(ros_image):
    r = rospy.get_param("vsbot/vs_baseline/pub_rate")
    rate = rospy.Rate(r)
    global bridge, image_pub
    #print(CallBack)
    '''try:
      #cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
      image_pub.publish(ros_image)
      rate.sleep()
    except CvBridgeError as e:
      print(e)'''
    
    '''while not rospy.is_shutdown():
    	  image_pub.publish(ros_image)
    	  rate.sleep()
    	  break'''
    if not rospy.is_shutdown():
        image_pub.publish(ros_image)
        rate.sleep()

def main(args):
  rospy.init_node('image_segmentation')
  image_sub = rospy.Subscriber("/vsbot/camera1/image_raw",Image,image_callback,  queue_size =1) 
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
    

