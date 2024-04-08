#!/usr/bin/python
import os

#******************************
#******** Enviroment **********
#******************************

#ENV_NAME = 'BipedalWalkerHardcore-v3'
# ENV_NAME = 'LunarLanderContinuous-v2'
#ENV_NAME = 'Pendulum-v0'
ENV_NAME = 'ROS_WS'

PATH_SAVE = "../model/"
#PATH_LOAD = None
PATH_LOAD = "../model/160p/"
#PATH_LOAD = "../model/save_agent_202105130824/"

#******************************
#****** Replay Buffer *********
#******************************

BATCH_SIZE = 64
MIN_SIZE_BUFFER = 100 # Minimum size of the buffer to start learning, until then random actions
BUFFER_CAPACITY = 1000000

#******************************
#******** Networks ************
#******************************

ACTOR_HIDDEN_0 = 512
ACTOR_HIDDEN_1 = 256
INIT_MINVAL = -0.5#-0.05
INIT_MAXVAL = 0.5#0.05

CRITIC_HIDDEN_0 = 512
CRITIC_HIDDEN_1 = 256

#******************************
#********** Agent *************
#******************************

GAMMA = 0.99
ACTOR_LR = 0.001#.001
CRITIC_LR = 0.005#0.001
#ACTOR_LR = 0.01
#CRITIC_LR = 0.005

TAU = 0.05 # For soft update the target network

# Parameters for Ornstein–Uhlenbeck process
THETA=0.45#0.15
DT=1e-1

# Use prioritized replay
PRIORITIZED_REPLAY = False
PRIORITY_ALPHA = 0.6
PRIORITY_BETA = 0.4

#******************************
#********** Main **************
#******************************

MAX_GAMES = 1200
EVALUATION_FREQUENCY = 100
SAVE_FREQUENCY = 200
LENGTH_THRESHOLD = 5

# Number of times to repeat learning per step.
# 20 is conservative, 40 what the paper recommended.
RELEARN_RATE = 20
