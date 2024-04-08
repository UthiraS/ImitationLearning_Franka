#!/usr/bin/env python3

import trainingPolicy as TP
import sys, csv, time, random, math, os, rospkg, rospy
import numpy as np
from os import path
from matplotlib import pyplot as plt
from active_vision.srv import trainedPolicySRV,trainedPolicySRVResponse
from active_vision.srv import getStatePCASRVRequest, getStatePCASRVResponse, getStatePCASRV
from load_data import loadDataJSON
from sklearn.model_selection import train_test_split
from sklearn.metrics import accuracy_score
import csv 

model = None

def helpDisp(text):
    print(text)
    print('\n-----Policy Training Help-----\n')
    print('Arguments : [StateVec]')
    print('StateVec : StateVec CSV file name')
    print('\n-----End Help-----\n')
    sys.exit()

#Takes a state vector input and returns a predicted direction
def predictionServer(req):
    global model
    stateVec = np.asarray(req.stateVec.data[0:50])
    stateVec = stateVec.reshape((1,stateVec.shape[0]))
    predDir = model.predict(stateVec)
    rospy.loginfo("Service called. Direction -> "+str(predDir))
    return trainedPolicySRVResponse(direction=predDir)

if __name__ == "__main__":
    # Load parameters from yaml
    rospy.init_node('PCA_Policy_Train_Server')
    print("Brick Policy")
    model = TP.Brick_Model()
    s = rospy.Service('/active_vision/trained_policy', trainedPolicySRV, predictionServer)
    rospy.loginfo(" policy service ready.")
    rospy.spin()
