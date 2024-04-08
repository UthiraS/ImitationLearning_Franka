import gym
import numpy as np
from gym import Env, spaces
import rospy
from active_vision.srv import controlSRV, controlSRVResponse, restartObjSRV, restartObjSRVResponse, optimalDistSRV, optimalDistSRVRequest
from geometry_msgs.msg import Point
from std_msgs.msg import Float64, String
from cmath import isfinite
from time import sleep
# import pdb 
REWARD_SCALING = 2.0

class Spec():
    """ a fake spec """

    def __init__(self, id_name):
        self.id = id_name

class environmentDetails():
    def __init__(self, name="base"):
        self.obj = []
        self.poses = []
        self.yaws = []
        self.objNames = []
        self.name = name

    def insert(self, id, number_poses, number_yaws, name):
        self.obj.append(id)
        self.poses.append(number_poses)
        self.yaws.append(number_yaws)
        self.objNames.append(name)

    def merge(self, e):
        for i in range(len(e.obj)):
            currentObj = e.obj[i]
            currentPose = e.poses[i]
            currentYaw = e.yaws[i]
            currentObjName = e.objNames[i]
            self.insert(currentObj, currentPose, currentYaw, currentObjName)

class defaultDetails(environmentDetails):
    def __init__(self, name="default"):
        super().__init__(name)
        self.insert(8, 1, 359, "009_gelatin_box")
        self.insert(41, 1, 359, "055_baseball")
        self.insert(51, 1, 359, "072-a_toy_airplane")
class easyDetails(environmentDetails):
    def __init__(self, name="easy"):
        super().__init__(name)
        self.insert(8, 1, 359, "009_gelatin_box")

class medDetails(environmentDetails):
    def __init__(self, name="med"):
        super().__init__(name)
        self.insert(41, 1, 359, "055_baseball")

class hardDetails(environmentDetails):
    def __init__(self, name="hard"):
        super().__init__(name)
        self.insert(51, 1, 359, "072-a_toy_airplane")

class emptyWrapper(Env):
    def __init__(self, random_poses=True, seed=None, details=environmentDetails()):
        super(emptyWrapper, self).__init__()
        np.random.seed(seed)
        self.observation_shape = (53,)
        self.observation_space = spaces.Box(low = np.zeros(self.observation_shape), 
                                            high = 255*np.ones(self.observation_shape),
                                            dtype = np.float16)
        self.action_space = spaces.Box(0, +1, (2,), dtype=np.float32)
        self.canvas = np.ones(self.observation_shape) * 1
        self.elements = []
        self.spec = Spec("dummy wrapper var")

class gazeboWrapper(Env):
    def __init__(self, random_poses=True, seed=None, details=environmentDetails()):
        super(gazeboWrapper, self).__init__()
        np.random.seed(seed)
        self.observation_shape = (53,)
        self.observation_space = spaces.Box(low = np.zeros(self.observation_shape), 
                                            high = 255*np.ones(self.observation_shape),
                                            dtype = np.float16)

        
        # Define an action space ranging from 0 to 4
        self.action_space = spaces.Box(0, +1, (2,), dtype=np.float32)
                        
        # Create a canvas to render the environment images upon 
        self.canvas = np.ones(self.observation_shape) * 1
        
        # Define elements present inside the environment
        self.elements = []
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
        self.strictLength = False
        self.maxPath = 40
        # pdb.set_trace()
        #self.details.obj = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
        #self.details.poses = [1, 2, 3, 1, 5, 1, 1, 1, 1, 1]
        #self.details.yaws = [89, 179, 179, 179, 359, 359, 179, 89, 359, 359]
        
        self.details = details
        self.random_poses = random_poses
        
        self.rMax = 360
        self.rMin = 0
        self.cReward = 2.0
        
        self.infoSpaceSize =  53 #1283 for  ESF ,619 for CVFH,VFH_features, 1027 for GASD features, 3 for FPFH, 53 for HAF, 45 for GRSD
        self.actionSpaceSize = 2
        self.obj = 3
        self.discrete = False
        self.past_optimal_distance = 0.0
        self.optimalDistmsg = optimalDistSRVRequest()
        # pdb.set_trace()
        self.cheatDirection = -1
        self.reset()
        self.spec = Spec("gazebo wrapper var")

    def reset(self):
        c = np.random.choice(len(self.details.obj))
        pose = np.random.choice(self.details.poses[c])
        yaw = np.random.choice(self.details.yaws[c])
        if(not self.random_poses):
            yaw = 43
        # yaw = 68
        self.obj = self.details.obj[c]

        self.cReward = 2.0
        
        print("Testing object %d, pose %d, yaw %d" % (self.obj, pose, yaw))
        
        restartData = self.restartEnv(self.obj, pose, yaw)

        observation = np.array(restartData.stateVec.data)
        
        
        angleMsg = Float64()
        objMsg = String()
        
        angleMsg.data = yaw
        objMsg = str(self.obj)
        # print(objMsg)
        
        self.srvObject.publish(objMsg)
        self.srvAngle.publish(angleMsg)

        sleep(.1)
        
        self.optimalDistmsg = optimalDistSRVRequest()
        # print(self.optimalDistmsg)
        cam_pose = Point()
        cam_pose.x = 1.0 #To account for viewsphere size diffs observation[-3]
        cam_pose.y = observation[-2]
        cam_pose.z = observation[-1]
        # pdb.set_trace()
        # print("Adding (%f, %f, %f)" % (cam_pose.x, cam_pose.y, cam_pose.z))
        self.optimalDistmsg.points.append(cam_pose)
        self.optimalDistmsg.viz.data = restartData.viz.data
        retMsg = self.optimalDist(self.optimalDistmsg)
        # pdb.set_trace()
        self.past_optimal_distance = max(0, (180 - retMsg.optimal_distance)/180)
        self.cheatDirection = retMsg.next_direction
        # print("Reseting, cheat = ", self.cheatDirection)
        # print(observation)
        return observation

    def cheat(self, input):
        # print("Looking up next best direction")
        action = (self.cheatDirection/360, 1)
        # print("C1 ", action)
        return [action]

    def cheat(self):
        action = [self.cheatDirection/360, 1]
        # print("C2 ", action)
        return action

    # Runs the simulation one tick forwards having taken action
    def step(self, action):
        # print("Taking action", action[0]*360)
        
        # direction = int()
        
        # if(action[0] < 0.5): direction = 3
        # else: direction = 7
        
        ret = self.moveCamera(int(360*action[0]), 0.05)
        np.set_printoptions(precision=4, threshold=1200)
        new_observation = np.array(ret.stateVec.data)
        # print(new_observation[-10:])
        # print(action)
        # print(int(360*action[0]))
        done = ret.done
        cStep = ret.steps
        
        cam_pose = Point()
        cam_pose.x = 1.0 #To account for viewsphere size diffs new_observation[-3]
        cam_pose.y = new_observation[-2]
        cam_pose.z = new_observation[-1]
        # print("Adding (%f, %f, %f)" % (cam_pose.x, cam_pose.y, cam_pose.z))
        self.optimalDistmsg.points.append(cam_pose)
        self.optimalDistmsg.viz.data = ret.viz.data
        self.optimalDistmsg.curr_direction = int(360*action[0])
        # "normalize" between 0 and 1. If the angle is less than 180,
        # it recieves a reward proportional to how close it is,
        # else it receives nothing
        retMsg = self.optimalDist(self.optimalDistmsg)
        curr_optimal_distance = max(0, (180 - retMsg.optimal_distance)/180)
        self.cheatDirection = retMsg.next_direction
        # print("Stepping, cheat = ", self.cheatDirection)
        
        visReward = ret.visibilityReward
        delta = curr_optimal_distance - self.past_optimal_distance
        # print("ret = ", (180 - retMsg.optimal_distance)/180, " c_opt = ", curr_optimal_distance, " old = ", self.past_optimal_distance)
        if(not isfinite(visReward)):
            visReward = 0
        if(done):
            reward = ((self.maxPath - cStep)/self.maxPath)
            # reward = self.cReward + (delta * REWARD_SCALING)
        else:
            # Add reward term for optimal distance (curr_optimal_distance - self.past_optimal_distance)
            # reward = (visReward) * HEURISTIC_COEF + (0.5 - cStep/10) * TIME_COEF + delta * OPT_COEF
            reward = delta * REWARD_SCALING
            # self.cReward = self.cReward * 0.9
            # print("Reward %.2f: vis=%.2f, time=%.2f, delta=%.2f-%.2f=%.2f" % (reward, (visReward) * HEURISTIC_COEF, (0.5 - cStep/10) * TIME_COEF, curr_optimal_distance, self.past_optimal_distance, delta*OPT_COEF))
            self.past_optimal_distance = curr_optimal_distance
            if(cStep > self.maxPath):
                done = True
        # print("reward: ", reward, " cStep: ", cStep)
        if(self.strictLength):
            reward = done * (100 - cStep)
            if(0 < reward):
                print(reward)
        return new_observation, reward, done, {"step":cStep, "direction": action[0]}

    def render(self, mode=0):
        return

def wrappedPrint(msg):
    print("-"*70)
    print(msg)
    print("-"*70)

if __name__ == '__main__':
    g = gazeboWrapper(random_poses=True, seed=None)
    wrappedPrint("Randomness test- should return different objs/positions")
    for i in range(10):
        g.reset()
    wrappedPrint("repeating....")
    for i in range(10):
        g.reset()
    g = gazeboWrapper(random_poses=True, seed=0)
    wrappedPrint("Seeded test- should return same objs/positions")
    for i in range(10):
        g.reset()
    g = gazeboWrapper(random_poses=True, seed=0)
    wrappedPrint("repeating....")
    for i in range(10):
        g.reset()