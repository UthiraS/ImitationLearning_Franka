#!/usr/bin/python
import os
from platform import architecture

#******************************
#******** Enviroment **********
#******************************

ENV_NAME = 'ROS_WS'
OS_NAME  = 'Ubuntu'
NETWORK_TYPE = "DDPG"
RECORD = "disabled" #disables recording
# RECORD = 'online' #enables recording
PATH_SAVE = "model/"

PATH_LOAD = "model/save_agent_31500"
LOAD_PRETRAINED = False

HEURISTIC_COEF = 0.0
TIME_COEF = .9
OPT_COEF = 0.0
MAX_REWARD = 2.0

#******************************
#****** Replay Buffer *********
#******************************

BATCH_SIZE = 64
# Minimum size of the buffer to start learning, until then random actions
MIN_SIZE_BUFFER = 500
BUFFER_CAPACITY = 100000

#******************************
#******** Networks ************
#******************************

ACTOR_HIDDEN_0 = 512
ACTOR_HIDDEN_1 = 256
INIT_MINVAL = -0.5
INIT_MAXVAL = 0.5

CRITIC_HIDDEN_0 = 512
CRITIC_HIDDEN_1 = 256

#******************************
#********** Agent *************
#******************************

GAMMA = 0.99
ACTOR_LR = 0.001
CRITIC_LR = 0.005

# For soft update the target network
TAU = 0.1

# Parameters for Ornstein–Uhlenbeck process
THETA = 0.45
DT = 1e-1

# Use prioritized replay
PRIORITIZED_REPLAY = False
PRIORITY_ALPHA = 0.6
PRIORITY_BETA = 0.4

#******************************
#********** Main **************
#******************************
# in episode number
MAX_EPISODES = 10000
EVALUATION_FREQUENCY = 100
PRETRAIN_LENGTH = 100
MAX_STEP_SIZE = 0.15

# in step number
LENGTH_THRESHOLD = 50
LEARN_RATE = 20
TARGET_UPDATE_RATE = 200
SAVE_FREQUENCY = 500
