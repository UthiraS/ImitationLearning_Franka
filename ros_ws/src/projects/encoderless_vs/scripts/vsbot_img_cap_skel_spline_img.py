#!/usr/bin/env python3
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
from skimage.morphology import skeletonize
from skimage import data, filters
from skimage.util import invert
from std_msgs.msg import String, Float64MultiArray, Float64
from scipy import spatial
from sensor_msgs.msg import Image
from scipy.interpolate import splprep, splev, UnivariateSpline, LSQUnivariateSpline, CubicSpline, BSpline, InterpolatedUnivariateSpline, make_interp_spline, make_lsq_spline
import time
from datetime import datetime
from cv_bridge import CvBridge, CvBridgeError
from encoderless_vs.msg import plot_spline

bridge = CvBridge()
skel_pub = rospy.Publisher('skel_viewer', Image, queue_size = 1)
spline_pub = rospy.Publisher('spline_viewer', Image, queue_size = 1)
#pubSpline = rospy.Publisher('Spline_Plot_Topic', plot_spline, queue_size=1)

fig = None
ax = None
bgd = None
dataPts = None
splinePts = None

def image_callback(ros_image):      
    global bridge, captured_image, fig, ax, bgd, dataPts, splinePts
    rate = rospy.Rate(30)
    #capture image in gazebo
    try:
        cv_image_capture = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print(e)	   
    
    #converting it into masked image
    
    tstart = time.time()
    hsvimg = cv2.cvtColor(cv_image_capture, cv2.COLOR_BGR2HSV)
	  	  
    orange_lower = np.array([10, 100, 20], np.uint8) 
    orange_upper = np.array([25, 255, 255], np.uint8) 
      
    white_lower = np.array([0, 0, 200], np.uint8)
    white_upper = np.array([145, 60, 255], np.uint8) 
	  
    orange_mask = cv2.inRange(hsvimg, orange_lower, orange_upper)
    white_mask = cv2.inRange(hsvimg, white_lower, white_upper)
	    
    kernel = np.ones((5, 5), "uint8")
	
    cv_mask = orange_mask + white_mask
    masked_image = bridge.cv2_to_imgmsg(cv_mask, "mono8")
    #Convert binary to skeleton
    binary = np.where(cv_mask > 0, 1, cv_mask)  
    skeleton = skeletonize(binary, method='lee')
    skeleton = np.where(skeleton == 1, 255, skeleton)
        
    try:
        ros_skeleton = bridge.cv2_to_imgmsg(skeleton, "mono8")
    except CvBridgeError as e:
        print(e)    
    if not rospy.is_shutdown():          
          skel_pub.publish(ros_skeleton)
#          rate.sleep()

    tstop = time.time()
    timer = tstop - tstart
    print("Binary and Skeleton Timing", timer)
    # End Convert binary to skeleton
    
    #Start of Ordering pixels of skeleton using nearest neigbor search
    pixels = np.argwhere(skeleton > 0)
    x = pixels[:, 0]
    y = pixels[:, 1]
    
    points = np.c_[x,y]    
    points = np.flip(points)
    
    #points = points[0::2]
    
    '''tstart = time.time()
    x = points[:,0]
    y = points[:,1]
    order = np.argsort(x)
    xsort, ysort = x[order], y[order]
    
    df = pd.DataFrame()
    df['xsort'] = xsort
    df['ysort'] = ysort
    
    mean = df.groupby('xsort').mean()
    #print("Mean", mean)
    df_x = mean.index    
    df_y = mean['ysort'] 
    
    k = 2
    t = [0, 1, 2, 3, 4, 5]
    c = [-1, 2, 0, 1]    

    spline = CubicSpline(df_x, df_y)
    
    co_effs = spline.c
             
    newX = df_x
    newX = newX.to_numpy() #np.linspace(startPt, endPt, 20)
    #print(len(newX))
    newY = spline(newX)
    
    tstop = time.time()
    timer = tstop - tstart
    
    print("Spline Time", timer)   '''
        
    #End of Ordering pixels of skeleton using nearest neigbor search
    tstart = time.time()
    #blank_img = 255 * np.ones((300, 300, 3), dtype = "uint8")
    blank_img = np.zeros((300, 300, 3), dtype = "uint8")
    #black = [0, 0, 0]
    #new_img = cv2.copyMakeBorder(blank_img, 1, 1, 1, 1, cv2.BORDER_CONSTANT, value=black)
    
    #Start plotting
    blu = [0, 0, 255]
    
    for i in range(len(points)):
        cv2.circle(blank_img, (points[i][0], points[i][1]), 1, blu, -1)
    tstop = time.time()
    timer = tstop - tstart
    print("Plotting Time", timer)
    
    tstart = time.time()
    cv2.imshow("Spline_Image", blank_img)
    cv2.waitKey(1)    
    tstop = time.time()
    timer = tstop - tstart
    print("Display Time", timer)
         
    

def main(args):
  rospy.init_node('image_skel_spline')
  image_sub = rospy.Subscriber("image_publisher",Image,image_callback,  queue_size = 1)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
