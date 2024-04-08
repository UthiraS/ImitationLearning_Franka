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

points = []
fig = None
ax = None
dataPts = None

px = []
py = []

def callback_vsbot_center(data):
     global points, fig, ax, dataPts, px, py
     
     cX = np.int(data.data[0])
     cY = np.int(data.data[1])
     
     pt = np.c_[cX, cY]
     
     #print(pt)
     
#     print(points)
     
     #print("X is: ", x)
     #print("Y is:", y)
     
     #x = np.append(data.data[0])
     #y = np.append(data.data[1])
     
     points = np.append(points, pt)
     
     points = np.reshape(points, (-1,2))
     
     print("Points Mapping", points)
     
     print(len(points))
     
     blank_img = 255 * np.ones((500, 500, 3), dtype = "uint8")

     
     for i in range(len(points)):
         x = np.int(points[i][0])
         px = np.append(px, x)
         y = np.int(points[i][1])  
         py = np.append(py, y)       
         cv2.circle(blank_img, (x, y), radius =0, color = (0, 0, 255), thickness = -1)
     
     cv2.imshow("Trajectory window", blank_img)
     cv2.waitKey(3)
     
     
     
     '''if fig == None:
       fig = plt.figure()
       ax = fig.add_subplot(111)
       ax.set_xlim([-500, 0])
       ax.set_ylim([-500, 0])
       dataPts, = ax.plot([], [], 'b-', label ='Data')
       fig.canvas.draw()
       fig.show()
     
     dataPts.set_data(-py, -px)
     #ax.plot(dataPts)
     fig.canvas.draw()'''
	      
	                 
	  


def main(args):
  rospy.init_node('marker_center_display', anonymous=True)  
  image_sub = rospy.Subscriber("marker_center_error", Int64MultiArray, callback_vsbot_center)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)



    
