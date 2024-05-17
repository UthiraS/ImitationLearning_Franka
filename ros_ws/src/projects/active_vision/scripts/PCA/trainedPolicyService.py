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
import glob

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
    stateVec = np.asarray(req.stateVec.data)
    stateVec = stateVec.reshape((1,stateVec.shape[0]))
    predDir = model.predict(stateVec)
    rospy.loginfo("Service called. Direction -> "+str(predDir))
    return trainedPolicySRVResponse(direction=predDir[0])

#change filename
#change feature_type
#change models


if __name__ == "__main__":
    feature_type = "GRSD" 
    #, "GRSD", "VFH", "CVFH", "ESF", "GASD"]

    # Load parameters from yaml
    rospy.init_node('PCA_Policy_Train_Server')
    
    # type = rospy.get_param("/active_vision/policyTester/policy")
    
    rospy.wait_for_service('/active_vision/getStateVectorPCA')
    getStateService = None
    try:
        getStateService = rospy.ServiceProxy('/active_vision/getStateVectorPCA', getStatePCASRV)
        print("Found state vector service")
    except:
        print("Service not found")
    
    filename = glob.glob("/home/rbe07/mer_lab/ros_ws/src/projects/active_vision/dataCollected/storage/Data_5/" + "*.txt")   

    print(feature_type)    
    stateVec, dirVec = loadDataJSON(filename, getStateService, feature_type)
    X_train, X_test, y_train, y_test = train_test_split(stateVec, dirVec, test_size = 0.10, random_state = 0)
        
    models = [TP.PCA_LDA_Model(0.9)]
    # models = [TP.PCA_LDA_LR_Model(0.9)]
    # models = [TP.PCA_LR_Model(0.9)]
        
    entry_row = [feature_type]
    print("using {} examples for training and {} examples for testing".format(X_train.shape[0], X_test.shape[0]))
    models[0].train(X_train, y_train)
    y_pred = models[0].predict(X_test)
    acc = accuracy_score(y_test, y_pred)
            
    model = models[0]
    s = rospy.Service('/active_vision/trained_policy', trainedPolicySRV, predictionServer)
    rospy.loginfo(" policy service ready.")
    rospy.spin()
    
"""
        if type == "PCA_LDA_LR":
            model = 
        elif type == "PCA_LDA":
            model = TP.PCA_LDA_Model(PCA_components)
        elif type == "PCA_LR":
            model = TP.PCA_LR_Model(PCA_components)
        elif type == "QLEARN":
            model = TP.Q_Model((pow(HAFstVecGridSize,2)*2)+2)
        elif type == "RANDOM":
            model = TP.Random_Model()
        elif type == "BRICK":
            model = TP.Brick_Model()
"""
        
