from cmath import isfinite
from time import sleep
import rospy
import numpy as np
from config import *
from active_vision.srv import controlSRV, controlSRVResponse, restartObjSRV, restartObjSRVResponse, optimalDistSRV, optimalDistSRVRequest
from geometry_msgs.msg import Point
from std_msgs.msg import Float64, String

class RosEnvironment():
    def __init__(self):
        super().__init__()
        rospy.init_node('Q_controller')
        print("Starting wait...")
        rospy.wait_for_service('/active_vision/restartEnv')
        print("Wait 1...")
        rospy.wait_for_service('/active_vision/moveKinect')
        print("Wait 2...")
        rospy.wait_for_service('/active_vision/optimalDistance')
        print("Wait done")
        self.restartEnv = rospy.ServiceProxy('/active_vision/restartEnv', restartObjSRV)
        self.moveCamera = rospy.ServiceProxy('/active_vision/moveKinect', controlSRV)
        self.optimalDist = rospy.ServiceProxy('/active_vision/optimalDistance', optimalDistSRV)
        self.srvObject = rospy.Publisher('/active_vision/cFileName', String)
        self.srvAngle = rospy.Publisher('/active_vision/cRotation', Float64)

        
        self.done = False
        
        #self.objs = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
        #self.poses = [1, 2, 3, 1, 5, 1, 1, 1, 1, 1]
        #self.yaws = [89, 179, 179, 179, 359, 359, 179, 89, 359, 359]
        
        #TODO: Load these parameters from ROS
        self.objs = [3]
        self.poses = [1] #Disabled temporarily, should=2
        self.yaws = [180]
        
        self.rMax = 360
        self.rMin = 0
        self.cReward = MAX_REWARD
        
        self.infoSpaceSize = 1027 #616 for CVFH and VFH_features, 1024 for GASD features
        self.actionSpaceSize = 2
        self.obj = 3
        self.discrete = False
        self.past_optimal_distance = 0.0
        self.optimalDistmsg = optimalDistSRVRequest()
        self.cheatDirection = -1
        objectMsg = String()
        #Hardcoded for now, should be based on object
        objectMsg.data = "prismAV20x6x5"
        self.srvObject.publish(objectMsg)

    # Choses random object + object position, then deletes + replaces current object
    def reset(self):
        c = np.random.choice(len(self.objs))
        pose = np.random.choice(self.poses[c])
        yaw = np.random.choice(self.yaws[c])

        self.cReward = MAX_REWARD
        
        print("Testing object %d, pose %d, yaw %d" % (self.obj, pose, yaw))
        
        observation = np.array(self.restartEnv(self.obj, pose, yaw).stateVec.data)
        
        angleMsg = Float64()
        
        angleMsg.data = yaw
        
        self.srvAngle.publish(angleMsg)
        sleep(.1)
        
        self.optimalDistmsg = optimalDistSRVRequest()
        cam_pose = Point()
        cam_pose.x = 1.0 #To account for viewsphere size diffs observation[-3]
        cam_pose.y = observation[-2]
        cam_pose.z = observation[-1]
        print("Adding (%f, %f, %f)" % (cam_pose.x, cam_pose.y, cam_pose.z))
        self.optimalDistmsg.points.append(cam_pose)
        retMsg = self.optimalDist(self.optimalDistmsg)
        self.past_optimal_distance = max(0, (180 - retMsg.optimal_distance)/180)
        self.cheatDirection = retMsg.next_direction

        return observation

    def cheat(self):
        # print("Looking up next best direction")
        if(3 == self.cheatDirection):
            action = [0, MAX_STEP_SIZE]
        else:
            action = [1, MAX_STEP_SIZE]
        return action

    # Runs the simulation one tick forwards having taken action
    def step(self, action):
        
        direction = int()
        
        if(action[0] < 0.5): direction = 3
        else: direction = 7
        
        ret = self.moveCamera(direction, np.clip(abs(action[1]) * MAX_STEP_SIZE, 0.01, MAX_STEP_SIZE))
        new_observation = np.array(ret.stateVec.data)
        done = ret.done
        cStep = ret.steps
        
        cam_pose = Point()
        cam_pose.x = 1.0 #To account for viewsphere size diffs new_observation[-3]
        cam_pose.y = new_observation[-2]
        cam_pose.z = new_observation[-1]
        # print("Adding (%f, %f, %f)" % (cam_pose.x, cam_pose.y, cam_pose.z))
        self.optimalDistmsg.points.append(cam_pose)
        self.optimalDistmsg.curr_direction = direction
        # "normalize" between 0 and 1. If the angle is less than 180,
        # it recieves a reward proportional to how close it is,
        # else it receives nothing
        retMsg = self.optimalDist(self.optimalDistmsg)
        curr_optimal_distance = max(0, (180 - retMsg.optimal_distance)/180)
        self.cheatDirection = retMsg.next_direction
        
        visReward = ret.visibilityReward
        if(not isfinite(visReward)):
            visReward = 0
        if(done):
            reward = self.cReward
        else:
            # Add reward term for optimal distance (curr_optimal_distance - self.past_optimal_distance)
            delta = curr_optimal_distance - self.past_optimal_distance
            # reward = (visReward) * HEURISTIC_COEF + (0.5 - cStep/10) * TIME_COEF + delta * OPT_COEF
            reward = 0
            self.cReward = self.cReward * TIME_COEF
            # print("Reward %.2f: vis=%.2f, time=%.2f, delta=%.2f-%.2f=%.2f" % (reward, (visReward) * HEURISTIC_COEF, (0.5 - cStep/10) * TIME_COEF, curr_optimal_distance, self.past_optimal_distance, delta*OPT_COEF))
            self.past_optimal_distance = curr_optimal_distance
        
        return new_observation, reward, done, cStep, direction

    def render(self):
        return

