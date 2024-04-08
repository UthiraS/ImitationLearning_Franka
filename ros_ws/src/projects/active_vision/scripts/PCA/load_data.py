#!/usr/bin/env python3

import numpy as np
import open3d
import json
import rospy
from std_msgs.msg import Header, Float32MultiArray
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from active_vision.srv import getStatePCASRVRequest, getStatePCASRVResponse, getStatePCASRV
from toolViewPointCalc import findDirection

FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
FIELDS_XYZRGB = FIELDS_XYZ + \
    [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]
    
class Entry:
    def __init__(self, r, theta, phi):
        self.r = r
        self.theta = theta
        self.phi = phi
        # self.print()
        
    def print(self):
        print("---{} {} {}".format(self.r, self.theta, self.phi))

def getPointCloud2msgFromPcd(pcd_path):
    open3d_cloud = open3d.io.read_point_cloud(pcd_path)
    
    # # Set "header"
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "camera"

    # # Set "fields" and "cloud_data"
    points = np.asarray(open3d_cloud.points)
    fields = FIELDS_XYZ
    cloud_data = points
    
    # # create ros_cloud
    return pc2.create_cloud(header, fields, cloud_data)

def visualize_pointcloud(path_obj, path_unexp):
    pcd_obj = open3d.io.read_point_cloud(path_obj)
    pcd_unexp = open3d.io.read_point_cloud(path_unexp)
    
    pcd_obj.paint_uniform_color([0.9, 0.1, 0.1])
    pcd_unexp.paint_uniform_color([0.1, 0.1, 0.9])
    
    open3d.visualization.draw_geometries([pcd_obj, pcd_unexp])

def loadDataJSON(filenames, getStateService, feature_type, visualize = False):
    states = []
    directions = []    
    for filename in filenames:
        file = open(filename)
        data = json.load(file)
        path = filename.split("log")[0]
    
        for run in data["data"]:
            # print(run["run_id"],": Num steps: ", len(run["steps"]))
            steps = run["steps"]
            for step_num in range(len(steps) - 1):
                curr_step = steps[step_num]
                next_step = steps[step_num + 1]
                start = Entry(curr_step["r"], 0, curr_step["phi"])
                end = Entry(next_step["r"], 0, next_step["phi"])
                            
                            
                # visualize
                if(visualize):
                    visualize_pointcloud(path + curr_step["pcdobj"], path + curr_step["pcdunexp"]) 

                # get states
                srv_msg = getStatePCASRVRequest()
                srv_msg.pcd_obj = getPointCloud2msgFromPcd(path + curr_step["pcdobj"])
                srv_msg.pcd_unexpobj = getPointCloud2msgFromPcd(path + curr_step["pcdunexp"])
                srv_msg.feature_type = feature_type
            
                response = getStateService(srv_msg)
                if(len(response.stateVec.data) == 0 or response.done == False): continue
                
                states.append(response.stateVec.data)
              
                direction = findDirection([0.5, start.phi, start.r],
                                      [0.5, end.phi, end.r])
                directions.append(direction)
            
    return np.asarray(states), np.asarray(directions)

if __name__ == "__main__":
    filename = '/home/rbe07/mer_lab/ros_ws/src/projects/active_vision/dataCollected/storage/Data_1/log_202202_03_12_11_1643908262.txt'
    loadDataJSON([filename], -1, -1)
