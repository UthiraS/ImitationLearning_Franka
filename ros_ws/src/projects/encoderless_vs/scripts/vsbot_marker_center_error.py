#!/usr/bin/env python3

import rospy
import signal
import sys
import cv2
from std_msgs.msg import String
from std_msgs.msg import Int64, Float64MultiArray, Int64MultiArray
from sensor_msgs.msg import Image
import matplotlib.pyplot as plt
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import sys, os

bridge = CvBridge()

points = []

pubCenter = rospy.Publisher('marker_center_error', Int64MultiArray, queue_size=1)
pubError = rospy.Publisher("publishError", Float64MultiArray, queue_size=1)
# Define a callback for the Image message
def image_callback(ros_image):
	  #print('got an image')
	  global bridge, points
	  r = rospy.get_param("vsbot/vs_baseline/pub_rate")
	  rate = rospy.Rate(r)
	  
	  try:
	    img = bridge.imgmsg_to_cv2(ros_image, "bgr8")
	  except CvBridgeError as e:
	      print(e) 
	  
	  hsvimg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	  
	  #taking binary masks of the end effector marker colored blue	  	  
	  blue_lower = np.array([94, 80, 2], np.uint8) 
	  blue_upper = np.array([120, 255, 255], np.uint8) 
	  blue_mask = cv2.inRange(hsvimg, blue_lower, blue_upper)
	    
	  '''kernel = np.ones((5, 5), "uint8")
	  blue_mask_contour = cv2.dilate(blue_mask, kernel)
	  res_blue = cv2.bitwise_and(img, img, mask = blue_mask)'''
	  
	  #taking binary mask of white pixels to complete binary convertion of the blue segment
	  white_lower = np.array([0, 0, 200], np.uint8)
	  white_upper = np.array([145, 60, 255], np.uint8)   
	  white_mask = cv2.inRange(hsvimg, white_lower, white_upper)
	  
	  orange_lower = np.array([10, 100, 20], np.uint8)
	  orange_upper = np.array([25, 255, 255], np.uint8)
	  orange_mask = cv2.inRange(hsvimg, orange_lower, orange_upper)
	  
	  # Blue segment converted to binary
	  mask1 = blue_mask + white_mask
	  
	  #mask2 = orange_mask + white_mask
	  
	  #First way of finding center of contour
	  pixels = np.argwhere(mask1 > 0)
	  
	  #print(pixels)
	  # find the first and last x components of binary pixels
	  x1 = pixels[0][1]
	  x2 = pixels[-1][1]
	  
	  #find the first and last y components of the binary pixels
	  y1 = pixels[0][0]
	  y2 = pixels[-1][0] 
	  
	  #calculating the center by averaging the pixels
	  cX = int((x1+x2)/2)
	  cY = int((y1+y2)/2)
	  	  
	  #print("Blue Center", cX, cY)
	  
	  pt = np.c_[cX, cY]
	  points = np.append(points, pt)
	  points = np.reshape(points, (-1, 2))
	  
	  print("Points Orig", points)

	  gx = rospy.get_param("vsbot/vs_baseline/goal_position_x") #goal x-coordinates
	  gy = rospy.get_param("vsbot/vs_baseline/goal_position_y") #goal y-coordinates
	  
	  img = cv2.circle(img, (gx, gy), radius=3, color=(0, 0, 255), thickness=-1)
	  
	  	  
	  cv2.imshow("Image window", img)
	  cv2.waitKey(3)
	  
	  
	  blue_center = Int64MultiArray()
	  blue_center.data = np.array([cX,cY])
	  
	  cXerror = gx - cX 
	  cYerror = gy - cY 
	  
	  errVect = Float64MultiArray()
	  errVect.data = np.array([cXerror,cYerror])
	  
	  #second way of finding center of marker
	  '''contours, hierarchy = cv2.findContours(blue_mask_contour, 
		                                   cv2.RETR_TREE, 
		                                   cv2.CHAIN_APPROX_SIMPLE) 
	  
	  
	  
	  for contour in contours:
	      M = cv2.moments(contour)
	      cX = int(M["m10"] / M["m00"])
	      cY = int(M["m01"] / M["m00"])
	      print("Second Centers", cX, cY)
	      
	  for contour in contours: 
		  blue_area = cv2.contourArea(contour)                     
		  if (blue_area > 300):
		      x, y, w, h = cv2.boundingRect(contour)
		      cX = int(x + (w/2))
		      cY = int(y + (h/2))
		      print("Third Centers", cX, cY)'''
		      
	  if not rospy.is_shutdown():	      
	      pubCenter.publish(blue_center)
	      pubError.publish(errVect)
	      rate.sleep()
	  
	  '''for i in range(len(points)):
	      cv2.circle(blank_img,                                                                                                                                                                                                                                                                                                                                                           (points[i][0], points[i][1]), radius=2, color=(0, 0, 255), thickness=-1)'''
	  
	  #cv2.imshow("Trajectory window", blank_img)
	  #cv2.waitKey(3)
	  
	      	      
	  if (cXerror == 0 and cYerror == 0):
	    rospy.loginfo("Shutdown")
	    input("Press Enter to continue...")
	    #rospy.signal_shutdown('need to stop')    
	  else:
	    rospy.loginfo("Nothing to see here")

         
	                 
	  


def main(args):
  rospy.init_node('marker_detection', anonymous=True)  
  image_sub = rospy.Subscriber("image_publisher",Image, image_callback)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)



    
