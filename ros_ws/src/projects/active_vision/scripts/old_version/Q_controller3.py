#!/usr/bin/env python3

from keras.layers import Input, Dense, Activation, LeakyReLU, Concatenate
from keras.models import Sequential, load_model
import tensorflow as tf
from keras.optimizers import Adam
import numpy as np
import gym
import pickle
import matplotlib
from tqdm import tqdm
import wandb
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
import rospkg
import rospy
from active_vision.srv import controlSRV, controlSRVResponse, restartObjSRV, restartObjSRVResponse

# Shamelessly stolen from https://keras.io/examples/rl/ddpg_pendulum/
class OUActionNoise:
    def __init__(self, mean, std_deviation, theta=0.15, dt=1e-1, x_initial=None):
        self.theta = theta
        self.mean = mean
        self.std_dev = std_deviation
        self.dt = dt
        self.x_initial = x_initial
        self.reset()

    def __call__(self):
        # Formula taken from https://www.wikipedia.org/wiki/Ornstein-Uhlenbeck_process.
        x = (
            self.x_prev
            + self.theta * (self.mean - self.x_prev) * self.dt
            + self.std_dev * np.sqrt(self.dt) * np.random.normal(size=self.mean.shape)
        )
        # Store x into x_prev
        # Makes next noise dependent on current one
        self.x_prev = x
        return x

    def reset(self):
        if self.x_initial is not None:
            self.x_prev = self.x_initial
        else:
            self.x_prev = np.zeros_like(self.mean)


class ReplayBuffer(object):
	def __init__(self, max_size, input_shape, n_actions, discrete=False):
		self.mem_size = max_size
		self.discrete = discrete
		self.state_memory = np.zeros((self.mem_size, input_shape))
		self.new_state_memory = np.zeros((self.mem_size, input_shape))
		if self.discrete:
			dtype = np.int8
		else:
			dtype = np.float32
		self.action_memory = np.zeros((self.mem_size, n_actions), dtype=dtype)
		self.reward_memory = np.zeros(self.mem_size)
		self.terminal_memory = np.zeros(self.mem_size, dtype=np.float32)
		self.mem_cntr = 0

	def store_transition(self, state, action, reward, new_state, done):
		index = self.mem_cntr % self.mem_size
		self.state_memory[index] = state
		self.new_state_memory[index] = new_state
		self.reward_memory[index] = reward
		self.terminal_memory[index] = 1 - int(done)
		if self.discrete:
			actions = np.zeros(self.action_memory.shape[1])
			actions[action] = 1.0
			self.action_memory[index] = actions
		else:
			self.action_memory[index] = action
		self.mem_cntr += 1

	def sample_buffer(self, batch_size):
		max_mem = min(self.mem_cntr, self.mem_size)
		batch = np.random.choice(max_mem, batch_size)
		states = self.state_memory[batch]
		new_states = self.new_state_memory[batch]
		rewards = self.reward_memory[batch]
		actions = self.action_memory[batch]
		terminals = self.terminal_memory[batch]
		return states, actions, rewards, new_states, terminals

def build_dqb(lr, n_actions, input_dims):
	last_init = tf.random_uniform_initializer(minval=-0.005, maxval=0.005)
	state_input = Input(shape=(input_dims))
	state_out = Dense(512, activation="relu")(state_input)
	state_out = Dense(256, activation="relu")(state_out)
	outputs = Dense(n_actions, activation="tanh", kernel_initializer=last_init)(state_out)

	model = tf.keras.Model(state_input, outputs)
	return model

def build_crit(lr, n_actions, input_dims):
	state_input = Input(shape=(input_dims))
	# state_out = Dense(16, activation="relu")(state_input)
	# state_out = Dense(32, activation="relu")(state_out)

	action_input = Input(shape=(n_actions))
	# action_out = Dense(32, activation="relu")(action_input)

	concat = Concatenate()([state_input, action_input])

	out = Dense(512, activation="relu")(concat)
	out = Dense(256, activation="relu")(out)
	outputs = Dense(1)(out)

	model = tf.keras.Model([state_input, action_input], outputs)
	return model

class Agent(object):
	def __init__(self, alpha, gamma, tau, n_actions, epsilon, batch_size, input_dims, discrete,
        rMin=0, rMax=1, epsilon_decount=0.996, epsilon_min=0.01, mem_size=100000, fname='q_model'):
		self.action_space = [i for i in range(n_actions)]
		self.n_actions = n_actions
		self.gamma = gamma
		self.tau = tau
		self.epsilon = epsilon
		self.epsilon_decount = epsilon_decount
		self.epsilon_min = epsilon_min
		self.batch_size = batch_size
		base_dir = rospkg.RosPack().get_path('active_vision')
		path = base_dir + "/QLearning/NN_Models/"
		self.fname = path+fname
		self.discrete = discrete
		self.rMin = rMin
		self.rMax = rMax
		self.ou_noise = OUActionNoise(mean=np.zeros(1), std_deviation=float(0.2) * np.ones(1))

		self.memory = ReplayBuffer(mem_size, input_dims, n_actions, discrete=self.discrete)
		self.q_act = build_dqb(alpha, n_actions, input_dims)
		self.q_act_target = build_dqb(alpha, n_actions, input_dims)
		self.q_crit = build_crit(alpha, n_actions, input_dims)
		self.q_crit_target = build_crit(alpha, n_actions, input_dims)
		self.actor_optimizer = tf.keras.optimizers.Adam(alpha/2.0)
		self.critic_optimizer = tf.keras.optimizers.Adam(alpha)
		if(discrete):
			self.q_act.compile(optimizer=Adam(lr=alpha), loss='mse')

	def remember(self, state, action, reward, new_state, done):
		self.memory.store_transition(state, action, reward, new_state, done)

	def choose_action(self, state):
		state = state[np.newaxis, :]
		rand = np.random.random()
		if self.discrete and rand < self.epsilon:
			if(self.discrete):
				action = np.random.choice(self.action_space)
			else:
				action = np.random.default_rng().uniform(self.rMin, self.rMax, self.n_actions)
		else:
			actions = self.q_act.predict(state)
			if(self.discrete):
				action = np.argmax(actions)
			else:
				action = np.reshape(actions, (self.n_actions,) )
				action += self.ou_noise()
				action = np.clip(action, self.rMin, self.rMax)
		return action

	@tf.function
	def update_target(self, target_weights, weights, tau):
		for (a, b) in zip(target_weights, weights):
			a.assign(b * tau + a * (1 - tau))

	@tf.function
	def updateFloat(self, state, action, reward, next_state, done):
		with tf.GradientTape() as tape:
			target_actions = self.q_act_target(next_state, training=True)
			y = tf.cast(reward, dtype=tf.float32) + \
				tf.cast(self.gamma * self.q_crit_target([next_state, target_actions], training=True) * (1-done), dtype=tf.float32)
			critic_value = self.q_crit([state, action], training=True)
			critic_loss = tf.math.reduce_mean(tf.math.square(y - critic_value))
		critic_grad = tape.gradient(critic_loss, self.q_crit.trainable_variables)
		self.critic_optimizer.apply_gradients(
			zip(critic_grad, self.q_crit.trainable_variables)
		)
		with tf.GradientTape() as tape:
			pred_actions = self.q_act(state, training=True)
			critic_value = self.q_crit([state, pred_actions], training=True)
			actor_loss = -tf.math.reduce_mean(critic_value)
		actor_grad = tape.gradient(actor_loss, self.q_act.trainable_variables)
		self.actor_optimizer.apply_gradients(
			zip(actor_grad, self.q_act.trainable_variables)
		)

	def learn(self):
		if self.memory.mem_cntr < self.batch_size:
			return
		state, action, reward, new_state, done = self.memory.sample_buffer(self.batch_size)
		if(self.discrete):
			action_values = np.array(self.action_space, dtype=np.int8)
			action_indices = np.dot(action, action_values)
			q_action = self.q_act(state)
			q_next = self.q_act(new_state)
			q_target = tf.zeros(q_action.shape, dtype=tf.float32)
			tf.compat.v1.assign(q_target, q_action, validate_shape=False, name='clone')

			batch_index = np.arange(self.batch_size, dtype=np.int32)
			q_target[batch_index, action_indices] = reward + self.gamma*np.max(q_next, axis=1)*done
			# print(self.gamma*np.max(q_next, axis=1)*done)
			_ = self.q_act.fit(state, q_target, verbose=0)
		else:
			self.updateFloat(state, action, reward, new_state, done)
		self.update_target(self.q_act_target.variables, self.q_act.variables, self.tau)
		self.update_target(self.q_crit_target.variables, self.q_crit.variables, self.tau)
		

		if(self.epsilon > self.epsilon_min):
			self.epsilon = self.epsilon*self.epsilon_decount
		else:
			self.epsilon = self.epsilon_min

	def save_model(self, i):
		self.q_act.save(self.fname+i+".h5")
		f = open(self.fname+i+"_mem", 'wb')
		pickle.dump(self.memory, f)

	def load_model(self, fname=None):
		if fname is None:
			target = self.fname
		else:
			target = fname
		self.q_act = load_model(target+".h5")
		f = open(target+"_mem", 'rb')
		self.memory = pickle.load(f)

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
		self.env = gym.make('LunarLanderContinuous-v2')
		self.done = False
		self.infoSpaceSize = 8
		self.actionSpaceSize = 2
		self.rMin = -1.0
		self.rMax = 1.0
		self.discrete = False

def display(epsilons, scores, avgs, stds):
	fig = plt.figure('F1')
	x = range(len(scores))
	fig.clear()
	fig.show()
	ax = fig.add_subplot(111)
	ax.plot(x,epsilons, 'r-', label='Random factor')
	ax.plot(x,scores, 'g-', label='Current Score')
	ax.plot(x,avgs, 'b-', label='Average Score')
	ax.plot(x,stds, 'm-', label='Standard Deviation')
	ax.legend()
	fig.canvas.draw()
	fig.canvas.flush_events()

if __name__ == '__main__':
	#plt.ion()
	# rospy.init_node('Q_controller')
	# rospy.wait_for_service('/active_vision/restartEnv')
	# rospy.wait_for_service('/active_vision/moveKinect')
	# restartEnv = rospy.ServiceProxy('/active_vision/restartEnv', restartObjSRV)
	# nextMove = rospy.ServiceProxy('/active_vision/moveKinect', controlSRV)
	base_dir = rospkg.RosPack().get_path('active_vision')
	path = base_dir + "/QLearning/NN_Models/"
	# env = generalEnvironment()
	env = floatEnvironment()
	n_games = 2000
	grid_size = 5  # Size of the state vector
	num_inputs = env.infoSpaceSize
	num_actions = env.actionSpaceSize
	objs = [1, 2, 3, 5, 6, 7]
	poses = [1, 2, 3, 1, 1, 1] #2
	yaws = [89, 179, 179, 89, 359, 179] #179
	convergence_target = 0.7

	agent = Agent(alpha=0.01, gamma=0.99, tau=0.05, n_actions=num_actions, epsilon=1.0, 
        batch_size=64, input_dims=num_inputs, discrete=env.discrete, rMin = env.rMin, rMax = env.rMax,
        epsilon_decount=0.996, epsilon_min=0.01, mem_size=1000000, fname='3Test')
	scores = []
	std_devs = []
	avg_scores = []
	eps_history = []
	## agent.load_model(path+'4layersall300')
	best = 0

	for i in range(n_games):
		done = False
		score = 0
		cStep = 0
		c = np.random.choice(len(objs))
		obj = objs[c]
		pose = np.random.choice(poses[c])
		yaw = np.random.choice(yaws[c])
		#print("Testing object %d, pose %d, yaw %d" % (obj, pose, yaw))
		##observation = np.array(restartEnv(obj, pose, yaw).stateVec.data)
		observation = env.reset()
		while not done:
			#print(cStep)
			env.render()
			action = agent.choose_action(observation)
			new_observation, reward, done, info = env.step(action)
			## ret = nextMove(action+1)
			## new_observation = np.array(ret.stateVec.data)
			## done = ret.done
			## cStep = ret.steps
			## if(done):
			## 	reward = 5
			## else:
			## 	reward = -1
			agent.remember(observation, action, reward, new_observation, done)
			observation = new_observation
			agent.learn()
			score += reward
		## if not done and 5 == cStep:
		## 	done = True

		eps_history.append(agent.epsilon*100)
		scores.append(score)
		avg_score = np.mean(scores[max(0, i-100):(i+1)])
		if avg_score > best and i >= 100:
			best = avg_score
		avg_scores.append(avg_score)
		std = np.std(scores[max(0, i-100):(i+1)])
		std_devs.append(std)

		print('obj (0-5) ', c, ' episode ', i, ' score %.2f ' %score, ' average score %.2f ' %avg_score, ' best score %.2f ' %best, ' std_dev %.2f' %std)
		if i % 10 == 0 and i > 0:
			agent.save_model("")
			if std < convergence_target:
				print("Reached target!")
				break
		if i % 100 == 0 and i > 0:
			agent.save_model(str(i))
		# if i % 1 == 0 and i > 0:
		# 	display(eps_history, scores, avg_scores, std_devs)