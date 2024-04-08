import numpy as np
import json
import sys
sys.path.append("../src")
from config import *
#from baselines.common.segment_tree import SumSegmentTree, MinSegmentTree
import random
from collections import deque

class ReplayBuffer(object):
    def __init__(self, env):
        self.mem_size = BUFFER_CAPACITY
        self.batch_size = BATCH_SIZE
        self.min_size_buffer = MIN_SIZE_BUFFER
        self.discrete = env.discrete
        if self.discrete:
            dtype = np.int8
        else:
            dtype = np.float32
        self.replay_memory = deque([], self.mem_size)
        self.n_games = 0

    def __len__(self):
        return len(self.replay_memory)

    def store_transition(self, state, action, reward, next_state, done):
        transition_tuple = [state, action, reward, next_state, done]
        self.replay_memory.append(transition_tuple)

    def check_buffer_size(self):
        return len(self.replay_memory) >= self.batch_size and len(self.replay_memory) >= self.min_size_buffer

    def update_n_games(self):
        self.n_games += 1

    def sample_buffer(self):
        minibatch = random.sample(list(self.replay_memory), self.batch_size)
        states = np.asarray(list(list(zip(*minibatch))[0]))
        actions = np.asarray(list(list(zip(*minibatch))[1]))
        rewards = np.asarray(list(list(zip(*minibatch))[2]))
        next_states = np.asarray(list(list(zip(*minibatch))[3]))
        dones = np.asarray(list(list(zip(*minibatch))[4]))
        return states, actions, rewards, next_states, dones
