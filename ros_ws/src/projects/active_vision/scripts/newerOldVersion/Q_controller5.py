#!/usr/bin/env python3
from tensorflow import keras
from tensorflow.keras.layers import Input, Dense, Activation, LeakyReLU, Concatenate
from tensorflow.keras.models import Sequential, load_model
import tensorflow as tf
from tensorflow.keras import optimizers as opt
import numpy as np
import gym
import pickle
import matplotlib
from tqdm import tqdm
import wandb
import time
#matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
import rospkg
import rospy
from config import *
from replay_buffer import *
from networks import *
from agent import AgentGB
from active_vision.srv import controlSRV, controlSRVResponse, restartObjSRV, restartObjSRVResponse
from baselines.common.schedules import LinearSchedule
import gc
import psutil

np.random.seed(0)

class generalEnvironment(object):
    def __init__(self):
        self.env = gym.make('LunarLander-v2')
        self.done = False
        self.infoSpaceSize = 8
        self.actionSpaceSize = 4
        self.rMin = 0.0
        self.rMax = 1.0
        self.discrete = True

    def reset(self):
        return self.env.reset()

    def step(self, action):
        return self.env.step(action)

    def render(self):
        self.env.render()

    def updateStopCondition(self, new_observation, reward, done, info):
        self.done = done

    def queryStopCondition(self):
        if(self.done):
            self.done = False
            return True
        else:
            return False

class floatEnvironment(generalEnvironment):
    def __init__(self):
        super().__init__()
        self.env = gym.make('LunarLanderContinuous-v2')
        self.done = False
        self.infoSpaceSize = 8
        self.actionSpaceSize = 2
        self.rMin = -1.0
        self.rMax = 1.0
        self.discrete = False

class floatRosEnvironment(floatEnvironment):
    def __init__(self):
        super().__init__()
        rospy.init_node('Q_controller')
        rospy.wait_for_service('/active_vision/restartEnv')
        rospy.wait_for_service('/active_vision/moveKinect')
        self.restartEnv = rospy.ServiceProxy('/active_vision/restartEnv', restartObjSRV)
        self.nextMove = rospy.ServiceProxy('/active_vision/moveKinect', controlSRV)
        self.rMin = 0.0
        self.rMax = 360.0
        #self.objs = [2]
        #self.poses = [2]
        #self.yaws = [179]
        self.objs = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
        self.poses = [1, 2, 3, 1, 5, 1, 1, 1, 1, 1]
        self.yaws = [89, 179, 179, 179, 359, 359, 179, 89, 359, 359]
        self.gridSize = 5
        self.infoSpaceSize = 308 #616 for CVFH and VFH_features, 1024 for GASD features
        print(self.infoSpaceSize)
        self.actionSpaceSize = 1
        self.obj = 0

    def reset(self):
        while(True):
            c = np.random.choice(len(self.objs))
            if(not (self.obj == self.objs[c])):
                self.obj = self.objs[c]
                break

        pose = np.random.choice(self.poses[c])
        yaw = np.random.choice(self.yaws[c])
        print("Testing object %d, pose %d, yaw %d" % (self.obj, pose, yaw))
        observation = np.array(self.restartEnv(self.obj, pose, yaw).stateVec.data)
        return observation

    def step(self, action):
        ret = self.nextMove(int(action))
        new_observation = np.array(ret.stateVec.data)
        done = ret.done
        cStep = ret.steps
        if(done):
            reward = 5
        else:
            reward = -1
        return new_observation, reward, done, cStep

    def render(self):
        return

if __name__ == '__main__':
    config = dict(
        learning_rate_actor = ACTOR_LR,
        learning_rate_critic = CRITIC_LR,
        batch_size = BATCH_SIZE,
        architecture = "DDPG",
        infra = "Ubuntu",
        env = ENV_NAME
    )

    wandb.init(
        project="tensorflow2_ddpg",
        tags=["DDPG", "FCL", "RL"],
        config=config,
    )
    wandb.define_metric("total_steps")
    wandb.define_metric("current_direction", step_metric="total_steps")
    wandb.define_metric("critic_loss", step_metric="total_steps")

    base_dir = rospkg.RosPack().get_path('active_vision')
    path = base_dir + "/model/"
    # env = generalEnvironment()
    #env = floatEnvironment()
    env = floatRosEnvironment()
    n_games = 1200
    grid_size = 5  # Size of the state vector
    num_inputs = env.infoSpaceSize
    num_actions = env.actionSpaceSize
    convergence_target = 0.7

    agent = AgentGB(env)
    scores = []
    std_devs = []
    avg_scores = []
    eps_history = []
    #agent.load()
    best = 0
    beta_annealing = LinearSchedule(n_games, initial_p=PRIORITY_BETA, final_p=1.0)

    evaluation = False
    total_steps = 0
    for i in range(n_games):
        start_time = time.time()
        done = False
        score = 0
        cStep = 0
        ramUsage = psutil.virtual_memory()[2]
        if(ramUsage > 94):
            print("Uh oh, too much RAM used!")
            print("saving...")
            agent.save()
            print("saved")
            break
        #print("Testing object %d, pose %d, yaw %d" % (obj, pose, yaw))
        ##observation = np.array(restartEnv(obj, pose, yaw).stateVec.data)
        observation = env.reset()
        while not done:
            total_steps += 1
            print(cStep)
            # env.render()
            action = agent.choose_action(observation, evaluation)
            print(int(action))
            new_observation, reward, done, cStep = env.step(action)
            
            agent.remember(observation, action, reward, new_observation, done)
            observation = new_observation
            #Update beta for prioritized replay
            agent.setPRBeta(beta_annealing.value(i))
            for _ in range(RELEARN_RATE):
                agent.learn()
            score += reward
            #print(LENGTH_THRESHOLD, cStep)
            if LENGTH_THRESHOLD <= cStep:
                done = True
            wandb.log({'total_steps': total_steps, 'critic_loss': agent.getCriticLoss(), 'current_direction': action}, commit=False)
        scores.append(score)
        avg_score = np.mean(scores[max(0, i-100):(i+1)])
        if avg_score > best and i >= 100:
            best = avg_score
        avg_scores.append(avg_score)
        std = np.std(scores[max(0, i-100):(i+1)])
        std_devs.append(std)
        agent.memory.update_n_games()
        wandb.log({'Game number': i, '# Episodes': agent.memory.mem_cntr, 
            "Average reward": round(np.mean(scores[-10:]), 2), \
                    "Time taken": round(time.time() - start_time, 2)})
        if (i + 1) % EVALUATION_FREQUENCY == 0:
            evaluation = True
            states = env.reset()
            done = False
            score = 0
            while not done:
                env.render()
                action = agent.choose_action(states, evaluation)
                new_states, reward, done, info = env.step(action)
                score += reward
                states = new_states
            wandb.log({'Game number': agent.memory.n_games, 
                    '# Episodes': agent.memory.mem_cntr, 
                    'Evaluation score': score})
            evaluation = False
        #if (i + 1) % SAVE_FREQUENCY == 0:
        #   print("saving...")
        #   agent.save()
        #   print("saved")
        #   gc.collect()
    print("saving...")
    agent.save()
    print("saved")
