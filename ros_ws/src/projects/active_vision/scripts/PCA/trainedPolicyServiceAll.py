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
    stateVec = np.asarray(req.stateVec.data)
    stateVec = stateVec.reshape((1,stateVec.shape[0]))
    predDir = model.predict(stateVec)
    rospy.loginfo("Service called. Direction -> "+str(predDir))
    return trainedPolicySRVResponse(direction=predDir)

if __name__ == "__main__":
    log_file = "object_2.csv"
    fields = ["Feature", "PCA_LDA_LR_0.8", "PCA_LDA_LR_0.6", "PCA_LDA_LR_0.4", "PCA_LDA_LR_0.2", "PCA_SVM_0.8", "PCA_SVM_0.6", "PCA_SVM_0.4", "PCA_SVM_0.2"] 
    with open(log_file, 'w') as csvfile: 
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow(fields) 
        csvfile.close()
    
    #os.chdir(os.path.expanduser("~"))

    # Load parameters from yaml
    rospy.init_node('PCA_Policy_Train_Server')
    
    # type = rospy.get_param("/active_vision/policyTester/policy")
    # PCA_components = rospy.get_param("/active_vision/policyTester/PCAcomponents")
    # PCA_components = 0.85

    # stateVec=List of all raw states
    # dirVec=List of the direction taken for each state
    # directions=N(1),NE(2),E(3),SE(4),S(5),SW(6),W(7),NW(8)
    
    rospy.wait_for_service('/active_vision/getStateVectorPCA')
    getStateService = None
    try:
        getStateService = rospy.ServiceProxy('/active_vision/getStateVectorPCA', getStatePCASRV)
        print("Found state vector service")
    except:
        print("Service not found")
    
    filename = ["/home/rbe07/mer_lab/ros_ws/src/projects/active_vision/dataCollected/storage/Data_11/log_202203_14_15_27_1647286033.txt"]    

    feature_types = ["HAF"] 
    #, "GRSD", "VFH", "CVFH", "ESF", "GASD"]
     
    for feature_type in feature_types:
        print(feature_type)    
        stateVec, dirVec = loadDataJSON(filename, getStateService, feature_type)
        X_train, X_test, y_train, y_test = train_test_split(stateVec, dirVec, test_size = 0.10, random_state = 0)
        
        models = [TP.PCA_LDA_LR_Model(0.8)]
        #, TP.PCA_LDA_LR_Model(0.6), TP.PCA_LDA_LR_Model(0.4), TP.PCA_LDA_LR_Model(0.2), TP.PCA_SVM_Model(0.8), TP.PCA_SVM_Model(0.6), TP.PCA_SVM_Model(0.4), TP.PCA_SVM_Model(0.2)]
        model_names = ["PCA_LDA_LR_0.8"]
        #, "PCA_LDA_LR_0.6", "PCA_LDA_LR_0.4", "PCA_LDA_LR_0.2", "PCA_SVM_0.8", "PCA_SVM_0.6", "PCA_SVM_0.4", "PCA_SVM_0.2"]
        
        entry_row = [feature_type]
        for i in range(len(models)):
            print("using {} examples for training and {} examples for testing".format(X_train.shape[0], X_test.shape[0]))
            models[i].train(X_train, y_train)
            y_pred = models[i].predict(X_test)
            acc = accuracy_score(y_test, y_pred)
            print("Accuracy for {} feature using {} model is {}".format(feature_type, model_names[i], acc))
            entry_row.append(str(acc))
        
        with open(log_file, 'a', newline='') as csvfile: 
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow(entry_row) 
            csvfile.close()
            print(entry_row)
            
    global model
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
        
