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

class AgentGB(object):
    def __init__(self, env, actor_lr=ACTOR_LR, critic_lr=CRITIC_LR, gamma=GAMMA, max_size=BUFFER_CAPACITY, 
        tau=TAU, path_save=PATH_SAVE, path_load=PATH_LOAD, prioritized_replay=PRIORITIZED_REPLAY):
        
        self.gamma = gamma
        self.tau = tau
        self.pr = prioritized_replay
        if(self.pr):
            self.memory = PrioritizedReplayBuffer(env)
            self.cBeta = PRIORITY_BETA
        else:
            self.memory = ReplayBuffer(env)
        self.action_space = env.actionSpaceSize
        self.upper_bound = env.rMax
        self.lower_bound = env.rMin
        self.actor_lr = actor_lr
        self.critic_lr = critic_lr
        self.path_save = path_save
        self.path_load = path_load
        self.discrete = env.discrete
        self.critic_loss = -1
        offset=0.0
        
        self.actor = Actor(name='actor', actions_dim=self.action_space, upper_bound=self.upper_bound+(2*offset), lower_bound=self.lower_bound-offset)
        self.critic = Critic(name='critic')
        self.target_actor = Actor(name='target_actor', actions_dim=self.action_space, upper_bound=self.upper_bound+(2*offset), lower_bound=self.lower_bound-offset)
        self.target_critic = Critic(name='target_critic')

        self.actor.compile(optimizer=opt.Adam(clipnorm=1e-8, learning_rate=actor_lr))
        self.critic.compile(optimizer=opt.Adam(clipnorm=1e-3, learning_rate=critic_lr))
        self.target_actor.compile(optimizer=opt.Adam(clipnorm=1e-8, learning_rate=actor_lr))
        self.target_critic.compile(optimizer=opt.Adam(clipnorm=1e-3, learning_rate=critic_lr))

        actor_weights = self.actor.get_weights()
        critic_weights = self.critic.get_weights()
        
        self.target_actor.set_weights(actor_weights)
        self.target_critic.set_weights(critic_weights)
        
        self.noise = np.zeros(self.action_space)

    def _ornstein_uhlenbeck_process(self, x, theta=THETA, mu=0, dt=DT, std=5.0):
        """
        Ornstein–Uhlenbeck process
        """
        return x + theta * (mu-x) * dt + std * np.sqrt(dt) * np.random.normal(size=self.action_space)

    def remember(self, state, action, reward, new_state, done):
        self.memory.store_transition(state, action, reward, new_state, done)

    def choose_action(self, observation, evaluation=False):
        state = tf.convert_to_tensor([observation], dtype=tf.float32)
        actions = self.actor(state)
        if not evaluation:
            print(actions)
            self.noise = self._ornstein_uhlenbeck_process(self.noise)
            actions += self.noise
            print(actions)
        else:
            actions += np.zeros(shape=(self.action_space,))
        actions = tf.clip_by_value(actions, self.lower_bound, self.upper_bound)
        print("action_taken: ", actions)
        return actions[0]

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
            target_critic_values = tf.squeeze(self.target_critic(
                                [new_states, target_actions]), 1)
            critic_value = tf.squeeze(self.critic([states, actions]), 1)
            # target = reward + self.gamma * target_critic_values
            target = reward + self.gamma * target_critic_values * (1-done)
            critic_loss = tf.keras.losses.MSE(target, critic_value)
            self.critic_loss = critic_loss.numpy()

        if(self.pr):
            new_weights = np.abs(target-critic_value) + 1e-6
            self.memory.update_priorities(indexes, new_weights)

        critic_gradient = tape.gradient(critic_loss,
                                            self.critic.trainable_variables)
        self.critic.optimizer.apply_gradients(zip(
            critic_gradient, self.critic.trainable_variables))

        with tf.GradientTape() as tape:
            policy_actions = self.actor(states)
            actor_loss = self.critic([states, policy_actions])
            actor_loss = -tf.math.reduce_mean(actor_loss)

        actor_gradient = tape.gradient(actor_loss, 
                                    self.actor.trainable_variables)
        self.actor.optimizer.apply_gradients(zip(
            actor_gradient, self.actor.trainable_variables))

        self.update_target_networks(self.tau)

    def update_target_networks(self, tau):
        actor_weights = self.actor.weights
        target_actor_weights = self.target_actor.weights
        for index in range(len(actor_weights)):
            target_actor_weights[index] = tau * actor_weights[index] + (1 - tau) * target_actor_weights[index]

        self.target_actor.set_weights(target_actor_weights)
        
        critic_weights = self.critic.weights
        target_critic_weights = self.target_critic.weights
    
        for index in range(len(critic_weights)):
            target_critic_weights[index] = tau * critic_weights[index] + (1 - tau) * target_critic_weights[index]

        self.target_critic.set_weights(target_critic_weights)

    def save(self):
        date_now = time.strftime("%Y%m%d%H%M")
        if not os.path.isdir(f"{self.path_save}/save_agent_{date_now}"):
            os.makedirs(f"{self.path_save}/save_agent_{date_now}")
        self.actor.save_weights(f"{self.path_save}/save_agent_{date_now}/{self.actor.net_name}.h5")
        self.target_actor.save_weights(f"{self.path_save}/save_agent_{date_now}/{self.target_actor.net_name}.h5")
        self.critic.save_weights(f"{self.path_save}/save_agent_{date_now}/{self.critic.net_name}.h5")
        self.target_critic.save_weights(f"{self.path_save}/save_agent_{date_now}/{self.target_critic.net_name}.h5")
        
        np.save(f"{self.path_save}/save_agent_{date_now}/noise.npy", self.noise)
        
        self.memory.save(f"{self.path_save}/save_agent_{date_now}")

    def load(self):
        self.actor.build(input_shape=(1,52))
        self.actor.load_weights(f"{self.path_load}/{self.actor.net_name}.h5")
        self.target_actor.build(input_shape=(1,52))
        self.target_actor.load_weights(f"{self.path_load}/{self.target_actor.net_name}.h5")
        self.critic.build(input_shape=(1,52))
        self.critic.load_weights(f"{self.path_load}/{self.critic.net_name}.h5")
        self.target_critic.build(input_shape=(1,52))
        self.target_critic.load_weights(f"{self.path_load}/{self.target_critic.net_name}.h5")
        
        self.noise = np.load(f"{self.path_load}/noise.npy")
        
        self.memory.load(f"{self.path_load}")

    # def learn(self):
    #     if self.memory.mem_cntr < self.batch_size:
    #         return
    #     state, action, reward, new_state, done = self.memory.sample_buffer()
    #     if(self.discrete):
    #         action_values = np.array(self.action_space, dtype=np.int8)
    #         action_indices = np.dot(action, action_values)
    #         q_action = self.q_act(state)
    #         q_next = self.q_act(new_state)
    #         q_target = tf.zeros(q_action.shape, dtype=tf.float32)
    #         tf.compat.v1.assign(q_target, q_action, validate_shape=False, name='clone')

    #         batch_index = np.arange(self.batch_size, dtype=np.int32)
    #         q_target[batch_index, action_indices] = reward + self.gamma*np.max(q_next, axis=1)*done
    #         # print(self.gamma*np.max(q_next, axis=1)*done)
    #         _ = self.q_act.fit(state, q_target, verbose=0)
    #     else:
    #         with tf.GradientTape() as tape:
    #             target_actions = self.q_act_target(new_state, training=True)
    #             y = tf.cast(reward, dtype=tf.float32) + \
    #                 tf.cast(self.gamma * self.q_crit_target(new_state, target_actions, training=True) * (1-done), dtype=tf.float32)
    #             critic_value = self.q_crit(state, action, training=True)
    #             critic_loss = tf.math.reduce_mean(tf.math.square(y - critic_value))
    #         critic_grad = tape.gradient(critic_loss, self.q_crit.trainable_variables)
    #         self.q_crit.optimizer.apply_gradients(
    #             zip(critic_grad, self.q_crit.trainable_variables)
    #         )
    #         with tf.GradientTape() as tape:
    #             pred_actions = self.q_act(state, training=True)
    #             critic_value = self.q_crit(state, pred_actions, training=True)
    #             actor_loss = -tf.math.reduce_mean(critic_value)
    #         actor_grad = tape.gradient(actor_loss, self.q_act.trainable_variables)
    #         self.q_act.optimizer.apply_gradients(
    #             zip(actor_grad, self.q_act.trainable_variables)
    #         )

    #     actor_weights = self.q_act.weights
    #     target_actor_weights = self.q_act_target.weights
    #     for index in range(len(actor_weights)):
    #         target_actor_weights[index] = self.tau * actor_weights[index] + (1 - self.tau) * target_actor_weights[index]

    #     self.q_act_target.set_weights(target_actor_weights)
        
    #     critic_weights = self.q_crit.weights
    #     target_critic_weights = self.q_crit_target.weights
    
    #     for index in range(len(critic_weights)):
    #         target_critic_weights[index] = self.tau * critic_weights[index] + (1 - self.tau) * target_critic_weights[index]

    #     self.q_crit_target.set_weights(target_critic_weights)

    #     if(self.epsilon > self.epsilon_min):
    #         self.epsilon = self.epsilon*self.epsilon_decount
    #     else:
    #         self.epsilon = self.epsilon_min

    # def save_model(self, i):
    #     self.q_act.save(self.fname+i+".h5")
    #     f = open(self.fname+i+"_mem", 'wb')
    #     pickle.dump(self.memory, f)

    # def load_model(self, fname=None):
    #     if fname is None:
    #         target = self.fname
    #     else:
    #         target = fname
    #     self.q_act = load_model(target+".h5")
    #     f = open(target+"_mem", 'rb')
    #     self.memory = pickle.load(f)
