#!/usr/bin/env python3

import sys, csv, time, random, math, os, rospkg, rospy
import numpy as np
from os import path
from matplotlib import pyplot as plt
from active_vision.srv import trainedPolicySRV,trainedPolicySRVResponse
from active_vision.srv import controlSRV, controlSRVResponse, restartObjSRV, restartObjSRVResponse, optimalDistSRV, optimalDistSRVRequest
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
import csv 

optimalDist = rospy.ServiceProxy('/active_vision/optimalDistance', optimalDistSRV)
optimalDistmsg = optimalDistSRVRequest()
cYaw = -1

def helpDisp(text):
    print(text)
    print('\n-----Policy Training Help-----\n')
    print('Arguments : [StateVec]')
    print('StateVec : StateVec CSV file name')
    print('\n-----End Help-----\n')
    sys.exit()

#Takes a state vector input and returns a predicted direction
def predictionServer(req):
    global optimalDist
    global optimalDistmsg
    np.set_printoptions(precision=4, threshold=1200)
    stateVec = np.array(req.stateVec.data)
    cam_pose = Point()
    cam_pose.x = 1.0
    cam_pose.y = stateVec[-2]
    cam_pose.z = stateVec[-1]
    optimalDistmsg.points.append(cam_pose)
    optimalDistmsg.viz = req.viz
    # print(optimalDistmsg.points)
    retMsg = optimalDist(optimalDistmsg)
    # print(stateVec[-10:])
    predDir = retMsg.next_direction
    # print(predDir)
    rospy.loginfo("Service called. Direction -> "+str(predDir))
    return trainedPolicySRVResponse(direction=predDir)

#Kludge- reset the object when a new angle is published
def updateObj(data):
    global optimalDistmsg
    optimalDistmsg = optimalDistSRVRequest()

if __name__ == "__main__":
    
    rospy.init_node('PCA_Policy_Train_Server')

    # stateVec=List of all raw states
    # dirVec=List of the direction taken for each state
    # directions=N(1),NE(2),E(3),SE(4),S(5),SW(6),W(7),NW(8)
    

    feature_types = ["ESFH"] 
    #, "GRSD", "VFH", "CVFH", "ESF", "GASD"]
    
    s = rospy.Service('/active_vision/trained_policy', trainedPolicySRV, predictionServer)
    rospy.Subscriber('/active_vision/cRotation', Float64, updateObj)
    rospy.loginfo(" policy service ready.")
    rospy.spin()
    
        
