from os import stat
import sys

from tensorflow.python.ops.gen_array_ops import lower_bound
sys.path.append("../src")
from replay_buffer import *
from config import *
import tensorflow as tf
import numpy as np
from tensorflow.keras.layers import Dense, LeakyReLU
from tensorflow.keras.initializers import random_uniform
from tensorflow.keras.layers.experimental.preprocessing import Normalization

class Critic(tf.keras.Model):
    def __init__(self, name, hidden_0=CRITIC_HIDDEN_0, hidden_1=CRITIC_HIDDEN_1):
        super(Critic, self).__init__()
        
        self.hidden_0 = hidden_0
        self.hidden_1 = hidden_1

        self.net_name = name

        self.dense_0 = Dense(self.hidden_0)
        self.dense_1 = Dense(self.hidden_1)
        self.q_value = Dense(1, activation=None)

    def call(self, input):
        state = tf.linalg.normalize(input[0])[0]
        action = input[1]
        #print(state, action)
        #print(len(input))
        state_action_value = self.dense_0(tf.concat([state, action], axis=1))
        state_action_value = LeakyReLU()(state_action_value)
        state_action_value = self.dense_1(state_action_value)
        state_action_value = LeakyReLU()(state_action_value)

        q_value = self.q_value(state_action_value)

        return q_value

class Actor(tf.keras.Model):
    def __init__(self, name, actions_dim, upper_bound, lower_bound, hidden_0=CRITIC_HIDDEN_0, hidden_1=CRITIC_HIDDEN_1, init_minval=INIT_MINVAL, init_maxval=INIT_MAXVAL):
        super(Actor, self).__init__()
        self.hidden_0 = hidden_0
        self.hidden_1 = hidden_1
        self.actions_dim = actions_dim
        self.init_minval = init_minval
        self.init_maxval = init_maxval
        self.upper_bound = upper_bound
        self.lower_bound = lower_bound
        
        self.net_name = name

        self.dense_0 = Dense(self.hidden_0, activation='relu')
        self.dense_1 = Dense(self.hidden_1, activation='relu')
        self.policy = Dense(self.actions_dim, kernel_initializer=random_uniform(minval=self.init_minval, maxval=self.init_maxval), activation='tanh')

    def call(self, state):
        state = tf.linalg.normalize(state)[0]
        # print(state)
        x = self.dense_0(state)
        # x = LeakyReLU()(x)
        policy = self.dense_1(x)
        # policy = LeakyReLU()(policy)
        policy = self.policy(policy)
        #policy = LeakyReLU()(policy)
        #Rescale outputs 0-2
        policy += 1.0
        #Rescale outputs 0-1
        policy /= 2.0

        return (policy * (self.upper_bound-self.lower_bound)) + self.lower_bound
        #return policy
