import sys
sys.path.append("../src")
import tensorflow as tf
from tensorflow.keras import optimizers as opt
import numpy as np
import random
import time
from config import *
from replay_buffer import *
from networks import *

class Agent(object):

    def __init__(self, env):
        self.gamma = GAMMA
        self.tau = TAU
        self.pr = PRIORITIZED_REPLAY
        
        if(self.pr):
            self.memory = PrioritizedReplayBuffer(env)
            self.cBeta = PRIORITY_BETA
        else:
            self.memory = ReplayBuffer(env)
            
        self.action_space = env.actionSpaceSize
        self.upper_bound = env.rMax
        self.lower_bound = env.rMin
        
        self.actor_lr = ACTOR_LR
        self.critic_lr = CRITIC_LR
        self.path_save = PATH_SAVE
        self.path_load = PATH_LOAD
        
        self.discrete = env.discrete
        
        self.critic_loss = -1
        
        self.actor = Actor(name='actor', actions_dim = self.action_space)
        self.critic = Critic(name='critic')
        self.target_actor = Actor(name='target_actor', actions_dim = self.action_space)
        self.target_critic = Critic(name='target_critic')

        self.actor.compile(optimizer = opt.Adam(clipnorm = 1e-8, learning_rate = self.actor_lr))
        self.critic.compile(optimizer = opt.Adam(clipnorm = 1e-3, learning_rate = self.critic_lr))
        self.target_actor.compile(optimizer = opt.Adam(clipnorm = 1e-8, learning_rate = self.actor_lr))
        self.target_critic.compile(optimizer = opt.Adam(clipnorm = 1e-3, learning_rate = self.critic_lr))

        self.noise = np.zeros(self.action_space)

        if LOAD_PRETRAINED:
            self.load()
            print("-----------------------LOADED PRETRAINED WEIGHTS --------------------------------")
        else:
            actor_weights = self.actor.get_weights()
            critic_weights = self.critic.get_weights()
        
            self.target_actor.set_weights(actor_weights)
            self.target_critic.set_weights(critic_weights)
        
            

    def _ornstein_uhlenbeck_process(self, x, theta = THETA, mu = 0, dt = DT, std = 0.25):
        """
        Ornstein–Uhlenbeck process
        """
        return x + theta * (mu-x) * dt + std * np.sqrt(dt) * np.random.normal(size=self.action_space)

    def remember(self, state, action, reward, new_state, done):
        self.memory.store_transition(state, action, reward, new_state, done)

    def choose_action(self, observation, evaluation=False):
        state = tf.convert_to_tensor([observation], dtype=tf.float32)
        action = self.actor(state)
        if not evaluation:
            self.noise = self._ornstein_uhlenbeck_process(self.noise)
            action += self.noise
        else:
            action += np.zeros(shape=(self.action_space,))
        
        # action= tf.clip_by_value(action, self.lower_bound, self.upper_bound)        
        return action[0]

    def setPRBeta(self, beta):
        self.cBeta = beta

    def getCriticLoss(self):
        return self.critic_loss

    def learn(self):
        if self.memory.check_buffer_size() == False:
            return
        if(self.pr):
            state, action, reward, new_state, done, indexes, weights = self.memory.sample_buffer(self.cBeta)
        else:
            state, action, reward, new_state, done = self.memory.sample_buffer()

        states = tf.convert_to_tensor(state, dtype=tf.float32)
        new_states = tf.convert_to_tensor(new_state, dtype=tf.float32)
        rewards = tf.convert_to_tensor(reward, dtype=tf.float32)
        actions = tf.convert_to_tensor(action, dtype=tf.float32)

        with tf.GradientTape() as tape:
            target_actions = self.target_actor(new_states)
            target_critic_values = tf.squeeze(self.target_critic([new_states, target_actions]), 1)
            critic_value = tf.squeeze(self.critic([states, actions]), 1)
            target = reward + self.gamma * target_critic_values * (1-done)
            critic_loss = tf.keras.losses.MSE(target, critic_value)
            self.critic_loss = critic_loss.numpy()

        if(self.pr):
            new_weights = np.abs(target-critic_value) + 1e-6
            self.memory.update_priorities(indexes, new_weights)

        critic_gradient = tape.gradient(critic_loss, self.critic.trainable_variables)
        self.critic.optimizer.apply_gradients(zip(critic_gradient, self.critic.trainable_variables))
        avg = 0
        for g in critic_gradient:
            avg += tf.reduce_mean(g)
        print("gradient", avg)

        with tf.GradientTape() as tape:
            policy_actions = self.actor(states)
            actor_loss = self.critic([states, policy_actions])
            actor_loss = -tf.math.reduce_mean(actor_loss)

        actor_gradient = tape.gradient(actor_loss, self.actor.trainable_variables)
        self.actor.optimizer.apply_gradients(zip(actor_gradient, self.actor.trainable_variables))

    def update_target_networks(self):
        actor_weights = self.actor.weights
        target_actor_weights = self.target_actor.weights
        for index in range(len(actor_weights)):
            target_actor_weights[index] = self.tau * actor_weights[index] + (1 - self.tau) * target_actor_weights[index]

        self.target_actor.set_weights(target_actor_weights)
        
        critic_weights = self.critic.weights
        target_critic_weights = self.target_critic.weights
    
        for index in range(len(critic_weights)):
            target_critic_weights[index] = self.tau * critic_weights[index] + (1 - self.tau) * target_critic_weights[index]

        self.target_critic.set_weights(target_critic_weights)

    def save(self, steps):
        #date_now = time.strftime("%Y%m%d%H%M")
        if not os.path.isdir(f"{self.path_save}/save_agent_{steps}"):
            os.makedirs(f"{self.path_save}/save_agent_{steps}")
        self.actor.save_weights(f"{self.path_save}/save_agent_{steps}/{self.actor.net_name}.h5")
        self.target_actor.save_weights(f"{self.path_save}/save_agent_{steps}/{self.target_actor.net_name}.h5")
        self.critic.save_weights(f"{self.path_save}/save_agent_{steps}/{self.critic.net_name}.h5")
        self.target_critic.save_weights(f"{self.path_save}/save_agent_{steps}/{self.target_critic.net_name}.h5")
        
        np.save(f"{self.path_save}/save_agent_{steps}/noise.npy", self.noise)
        # self.memory.save(f"{self.path_save}/save_agent_{date_now}")

    def load(self):
        self.actor.build(input_shape=(1,1027))
        self.actor.load_weights(f"{self.path_load}/{self.actor.net_name}.h5")
        self.target_actor.build(input_shape=(1,1027))
        self.target_actor.load_weights(f"{self.path_load}/{self.target_actor.net_name}.h5")
        self.critic.build(input_shape=[(1,1027),(1,2)])
        self.critic.load_weights(f"{self.path_load}/{self.critic.net_name}.h5")
        self.target_critic.build(input_shape=[(1,1027),(1,2)])
        self.target_critic.load_weights(f"{self.path_load}/{self.target_critic.net_name}.h5")
        
        # self.noise = np.load(f"{self.path_load}/noise.npy")
        
        # self.memory.load(f"{self.path_load}")
