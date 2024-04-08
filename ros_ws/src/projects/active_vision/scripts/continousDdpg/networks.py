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
    def __init__(self, name):
        super(Critic, self).__init__()
        
        self.hidden_0 = CRITIC_HIDDEN_0
        self.hidden_1 = CRITIC_HIDDEN_1

        self.net_name = name

        self.dense_0 = Dense(self.hidden_0, activation = 'relu')
        self.dense_1 = Dense(self.hidden_1, activation = 'relu')
        self.q_value = Dense(1, activation = None)

    def call(self, input):
        state = tf.linalg.normalize(input[0])[0]
        action = input[1]
        state_action_value = self.dense_0(tf.concat([state, action], axis=1))
        state_action_value = self.dense_1(state_action_value)
        q_value = self.q_value(state_action_value)
        return q_value

class Actor(tf.keras.Model):
    def __init__(self, name, actions_dim):
        super(Actor, self).__init__()
        self.hidden_0 = ACTOR_HIDDEN_0
        self.hidden_1 = ACTOR_HIDDEN_1
        self.actions_dim = actions_dim
        self.init_minval = INIT_MINVAL
        self.init_maxval = INIT_MAXVAL
        
        self.net_name = name

        self.dense_0 = Dense(self.hidden_0, activation = 'relu')
        self.dense_1 = Dense(self.hidden_1, activation = 'relu')
        self.policy = Dense(self.actions_dim, activation = 'sigmoid')

    def call(self, state):
        state = tf.linalg.normalize(state)[0]
        x = self.dense_0(state)
        x = self.dense_1(x)
        x = self.policy(x)
        return (x)
