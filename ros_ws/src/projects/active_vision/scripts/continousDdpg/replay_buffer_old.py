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
        transition_tuple = (state, action, reward, next_state, done)
        self.replay_memory.appendleft(transition_tuple)

    def check_buffer_size(self):
        return len(self.replay_memory) >= self.batch_size and len(self.replay_memory) >= self.min_size_buffer

    def update_n_games(self):
        self.n_games += 1

    def sample_buffer(self):
        minibatch = np.random.choice(np.asarray(self.replay_memory), self.batch_size, replace=False)
        states = minibatch[:][0]
        actions = minibatch[:][1]
        rewards = minibatch[:][2]
        next_states = minibatch[:][3]
        dones = minibatch[:][4]
        return states, actions, rewards, next_states, dones
"""
class ReplayBufferOld(object):
    def __init__(self, env):
        self.mem_size = BATCH_SIZE
        self.batch_size = BATCH_SIZE
        self.min_size_buffer = MIN_SIZE_BUFFER
        self.discrete = env.discrete
        if self.discrete:
            dtype = np.int8
        else:
            dtype = np.float32
        self.states = np.zeros((self.mem_size, env.infoSpaceSize))
        self.next_states = np.zeros((self.mem_size, env.infoSpaceSize))
        self.actions = np.zeros((self.mem_size, env.actionSpaceSize), dtype=dtype)
        self.rewards = np.zeros(self.mem_size)
        self.dones = np.zeros(self.mem_size, dtype=bool)
        self.mem_cntr = 0
        self.n_games = 0
        self.index = 0

    def __len__(self):
        return min(self.mem_cntr, self.mem_size)

    def store_transition(self, state, action, reward, next_state, done):
        self.index = self.mem_cntr % self.mem_size
        self.states[self.index] = state
        self.next_states[self.index] = next_state
        self.rewards[self.index] = reward
        self.dones[self.index] = done
        if self.discrete:
            actions = np.zeros(self.actions.shape[1])
            actions[action] = 1.0
            self.actions[self.index] = actions
        else:
            self.actions[self.index] = action
        self.mem_cntr += 1

    def check_buffer_size(self):
        return self.mem_cntr >= self.batch_size and self.mem_cntr >= self.min_size_buffer

    def update_n_games(self):
        self.n_games += 1

    def sample_buffer(self):
        max_mem = min(self.mem_cntr, self.mem_size)
        batch = np.random.choice(max_mem, self.batch_size, replace=False)
        states = self.states[batch]
        next_states = self.next_states[batch]
        rewards = self.rewards[batch]
        actions = self.actions[batch]
        dones = self.dones[batch]
        return states, actions, rewards, next_states, dones
    
    def save(self, folder_name):
        """
        """
        Save the replay buffer
        """
        """
        if not os.path.isdir(folder_name):
            os.mkdir(folder_name)

        np.save(folder_name + '/states.npy', self.states)
        np.save(folder_name + '/actions.npy', self.actions)
        np.save(folder_name + '/rewards.npy', self.rewards)
        np.save(folder_name + '/next_states.npy', self.next_states)
        np.save(folder_name + '/dones.npy', self.dones)
        
        dict_info = {"buffer_counter": self.mem_cntr, "n_games": self.n_games}
        
        with open(folder_name + '/dict_info.json', 'w') as f:
            json.dump(dict_info, f)

    def load(self, folder_name):
        """
        """
        Load the replay buffer
        """
        """
        self.states = np.load(folder_name + '/states.npy')
        self.actions = np.load(folder_name + '/actions.npy')
        self.rewards = np.load(folder_name + '/rewards.npy')
        self.next_states = np.load(folder_name + '/next_states.npy')
        self.dones = np.load(folder_name + '/dones.npy')
        
        with open(folder_name + '/dict_info.json', 'r') as f:
            dict_info = json.load(f)
        self.mem_cntr = dict_info["buffer_counter"]
        self.n_games = dict_info["n_games"]
        print(self.buffer_counter)

class PrioritizedReplayBuffer(ReplayBuffer):
    def __init__(self, env, max_size=BUFFER_CAPACITY, batch_size=BATCH_SIZE, 
    min_size_buffer=MIN_SIZE_BUFFER, p_alpha=PRIORITY_ALPHA):
        super(PrioritizedReplayBuffer, self).__init__(env, max_size, batch_size, min_size_buffer)
        self.alpha = p_alpha
        self.max_priority = 1.0
        self.tree_size = 1
        while (self.tree_size) < self.mem_size:
            self.tree_size *= 2
        self.sum_tree = SumSegmentTree(self.tree_size)
        self.min_tree = MinSegmentTree(self.tree_size)

    def store_transition(self, state, action, reward, next_state, done):
        super().store_transition(state, action, reward, next_state, done)
        self.sum_tree[self.index] = self.max_priority ** self.alpha
        self.min_tree[self.index] = self.max_priority ** self.alpha

    def sample_proportional(self):
        ret = []
        # priority_sum = self.sum_tree.sum(0, len(self)-1)
        priority_sum = self.sum_tree.sum()
        range_len = priority_sum / self.batch_size
        for i in range(self.batch_size):
            mass = random.random() * range_len * (i + 1)
            index = self.sum_tree.find_prefixsum_idx(mass)
            ret.append(index)
        return ret

    def sample_buffer(self, beta=PRIORITY_BETA):
        indexes = self.sample_proportional()
        priority_sum = self.sum_tree.sum()
        priority_min = self.min_tree.min() / priority_sum
        max_weight = (priority_min * len(self)) ** (-beta)
        weights = []

        for i in indexes:
            priority = self.sum_tree[i] / priority_sum
            weight = (priority * len(self)) ** (-beta)
            weights.append(weight / max_weight)

        states = self.states[indexes]
        next_states = self.next_states[indexes]
        rewards = self.rewards[indexes]
        actions = self.actions[indexes]
        dones = self.dones[indexes]
        return states, actions, rewards, next_states, dones, indexes, weights

    def update_priorities(self, indexes, priorities):
        for i in range(len(indexes)):
            self.sum_tree[indexes[i]] = priorities[i] ** self.alpha
            self.min_tree[indexes[i]] = priorities[i] ** self.alpha
            self.max_priority = max(self.max_priority, priorities[i])

    def save(self, folder_name):
        super().save(folder_name)
        # Coming soon!

    def load(self, folder_name):
        super().load(folder_name)
        # Coming soon!
"""
