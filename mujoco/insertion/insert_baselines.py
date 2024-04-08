import robosuite as suite
from robosuite import load_controller_config
import numpy as np
from robosuite.wrappers import GymWrapper
import matplotlib.pyplot as plt
from gym import spaces
import pandas as pd
from stable_baselines3 import PPO, SAC
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.monitor import Monitor
from datetime import datetime

np.seterr(divide='ignore', invalid='ignore')

class InsertionTrainer:
    def __init__(self,
                 num_episodes=20,
                 steps_per_episode=1024,
                 live_plot=False,
                 save_obs_data=False):
        '''Initialize Insertion Trainer using Mujoco and Robosuite.

        num_episodes: Number of episodes to train for.
        steps_per_episode: Number of timesteps per episode.
        live_plot: Whether to plot useful data during episode.
                  (Significantly slows down simulation).
        save_obs_data: Whether to save observations to csv.
        '''
        #Store inputs
        self.num_episodes=num_episodes
        self.steps_per_episode=steps_per_episode
        self.save_obs_data=save_obs_data
        #Set/initialize other system parameters
        now=datetime.now()
        now_s=now.strftime("%m%d%Y_%H%M")
        self.save_name=f'{self.num_episodes*steps_per_episode}steps_{now_s}'
        self.control_dof=7 #XYZ RPY Gripper
        self.control='OSC_POSE'
        self.episode_rewards=[]
        self.live_plot=live_plot
        # Configure robosuite enviroment
        self.config = load_controller_config(default_controller=self.control)
        # Create environment
        self.env =GymWrapper(
                    suite.make("Insert", 
                                robots="Panda",
                                controller_configs=self.config,
                                has_renderer=False,
                                has_offscreen_renderer=False,
                                use_camera_obs=False,
                                ),
                    keys=['robot0_eef_pos',  #3 XYZ
                          #'robot0_eef_quat', #4 EE Quaternion
                          'ft_sensor',       #6 fx,fy,fz,tx,ty,tz
                          ]
        )
        #Define action space for OSC_POSE control (Normalized for PPO)
        #See gym_wrapper.py for de-normalized step.
        self.env.action_space = spaces.Box(low=np.array([-1.,-1.,-1.,-1.,-1.,-1.,-1.]), 
                                           high=np.array([1.,1.,1.,1.,1.,1.,1.]))
        self.env.observation_space = spaces.Box(low=np.array([-1.,-1.,-1.,-1.,-1.,-1.,-1.,-1.,-1.]), 
                                                high=np.array([1.,1.,1.,1.,1.,1.,1.,1.,1.]))
        #Pass episode time limit to Insert environment.
        self.env.env.horizon=self.steps_per_episode
        #Pass live_plot arg to Insert environment.
        self.env.env.live_plot=self.live_plot
        #Pass save_obs_data arg to Insert environment.
        self.env.env.save_obs_data=self.save_obs_data
        #Get panda object from environment.
        self.panda=self.env.robots[0]
        #Create agent using stable_baselines
        self.agent = PPO("MlpPolicy", self.env, verbose=1,
                         learning_rate=0.003, #0.0003 default. 0.00003 is current best.
                         batch_size=2048, #64 default. 2048 is current best.
                         n_steps=self.steps_per_episode,
                         tensorboard_log="./ppo_peg/")

    def run_sim(self):
        '''Runs Peg Insertion training.'''
        eval_callback = EvalCallback(Monitor(self.env), 
                                     best_model_save_path='./logs/best_model/'+self.save_name,
                                     eval_freq=20*self.steps_per_episode,
                                     deterministic=False, 
                                     render=False)
        try:
            self.agent.learn(total_timesteps=self.num_episodes*self.steps_per_episode, 
                             callback=eval_callback,
                             log_interval=1)
        except KeyboardInterrupt:
            print('Ending training due to KeyboardInterrupt.')
        finally:    
            self.agent.save(self.save_name)

if __name__=='__main__':
    it=InsertionTrainer(num_episodes=200, steps_per_episode=2048, live_plot=False, save_obs_data=True)
    it.run_sim()

    #OSC/World:
        #X is coming towards you.
        #Y is to the right.
        #Z is up.

    #Peg:
        #Z is roughly aligned with Hole.

    #Hole (XY Origin as at center of table):
        #Z is roughly aligned with Peg.