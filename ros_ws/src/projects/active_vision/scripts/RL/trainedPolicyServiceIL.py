#!/usr/bin/env python3

import sys, csv, time, random, math, os, rospkg, rospy
import numpy as np
from os import path
from matplotlib import pyplot as plt
from active_vision.srv import trainedPolicySRV,trainedPolicySRVResponse
from active_vision.srv import getStatePCASRVRequest, getStatePCASRVResponse, getStatePCASRV
# from load_data import loadDataJSON
# from sklearn.model_selection import train_test_split
# from sklearn.metrics import accuracy_score
import Imitation, gazeboEnv
import csv 
import pdb 
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
    # pdb.set_trace()
    np.set_printoptions(precision=4, threshold=1200)
    stateVec = np.array(req.stateVec.data)
    print(stateVec[-10:])
    predDir = model.predict(stateVec, deterministic=True)
    print(predDir)
    predDir = int(360*(predDir[0][0]))
    rospy.loginfo("Service called. Direction -> "+str(predDir))
    return trainedPolicySRVResponse(direction=predDir)

if __name__ == "__main__":
    
    rospy.init_node('PCA_Policy_Train_Server')
    # pdb.set_trace()
    # stateVec=List of all raw states
    # dirVec=List of the direction taken for each state
    # directions=N(1),NE(2),E(3),SE(4),S(5),SW(6),W(7),NW(8)
    

   
    
    feature_type = sys.argv[1]
    print("Feature Type ", feature_type)

    modelType = sys.argv[2]

    print("Loading ", modelType)
     
    model = Imitation.bcSetup(None, "LOAD", gazeboEnv.emptyWrapper(feature_type), modelType).model.policy
    
    s = rospy.Service('/active_vision/trained_policy', trainedPolicySRV, predictionServer)
    rospy.loginfo(" policy service ready.")
    rospy.spin()
    
        
