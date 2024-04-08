from fileinput import filename
import gym
import numpy as np

from stable_baselines3 import DDPG, HerReplayBuffer, PPO
from stable_baselines3.common.buffers import ReplayBuffer
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise
from stable_baselines3.common.vec_env import DummyVecEnv
import gym
from scripts.RL.gazeboEnv import gazeboWrapper
import wandb
from wandb.integration.sb3 import WandbCallback
from imitation.algorithms import bc
from imitation.data import rollout
from imitation.data.wrappers import RolloutInfoWrapper
from stable_baselines3.td3.policies import MlpPolicy
import pickle

env = gym.make("LunarLanderContinuous-v2")
# env = gazeboWrapper(random_poses=False)
pT = False
genNewData = False
config = dict(
        env = "Pytorch Wrapper",
        pretrain =str(pT)
    )

n_actions = env.action_space.shape[-1]
action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))

def buildRollout(bSize = 50):
    buffer = ReplayBuffer(buffer_size= bSize,
        observation_space= env.observation_space,
        action_space= env.action_space,
        handle_timeout_termination=False)
    observation = env.reset()
    model = DDPG("MlpPolicy", env, action_noise=action_noise, verbose=2, tensorboard_log=f"runs/{run.id}")
    model.load("example")
    for i in range(bSize):
        # action = env.cheat()
        action = model.predict(observation)[0]
        new_observation, reward, done, info = env.step(action)
        buffer.add(observation, new_observation, np.array(action), reward, done, info)
        if(done):
            observation = env.reset()
        else:
            observation = new_observation
    print(buffer)
    file = open('models/transition_save_cont','wb')
    pickle.dump(buffer, file)
    file.close()

def sample_expert_transitions():
    model = DDPG("MlpPolicy", env, action_noise=action_noise, verbose=2, tensorboard_log=f"runs/{run.id}")
    model.load("example")
    expert = model

    print("Sampling expert transitions.")
    rollouts = rollout.rollout(
        expert,
        DummyVecEnv([lambda: RolloutInfoWrapper(env)]),
        rollout.make_sample_until(min_timesteps=None, min_episodes=50),
    )
    print(rollouts)
    return rollouts


run = wandb.init(
    # mode = "disabled",
    project="sb3",
    config=config,
    sync_tensorboard=True,  # auto-upload sb3's tensorboard metrics
    monitor_gym=True,  # auto-upload the videos of agents playing the game
    # save_code=True,  # optional

)
def genNewTransitions(fileName = 'models/transition_save'):
    transitions = sample_expert_transitions()
    file = open(fileName,'wb')
    pickle.dump(transitions, file)
    file.close()

if(genNewData):
    buildRollout(50000)

# model = DDPG("MlpPolicy", env, action_noise=action_noise, verbose=2, tensorboard_log=f"runs/{run.id}")
model = PPO("MlpPolicy", env, verbose=2, tensorboard_log=f"runs/{run.id}")
if(pT):
    # file = open('models/transition_save_cont', 'rb')
    # transitions = pickle.load(file)
    # file.close()
    model.load_replay_buffer('models/transition_save_cont')

model.learn(total_timesteps=1000000, log_interval=10, callback=WandbCallback(
        model_save_path=f"models/{run.id}",
        verbose=2,
    ),)
model.save("PPO")

# # model = DDPG.load("ddpg_wrapper_2")

# obs = env.reset()
# while True:
#     action, _states = model.predict(obs)
#     obs, rewards, dones, info = env.step(action)
#     env.render()