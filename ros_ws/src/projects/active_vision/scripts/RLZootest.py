import tempfile
import numpy as np
import gym
from stable_baselines3 import DDPG
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.ppo import MlpPolicy

from imitation.algorithms import bc
from imitation.algorithms.dagger import SimpleDAggerTrainer

r = np.random.default_rng(0)
env = gym.make("LunarLanderContinuous-v2")
expert = DDPG(policy="MlpPolicy", env=env)
expert.learn(1000)
venv = DummyVecEnv([lambda: gym.make("LunarLanderContinuous-v2")])

bc_trainer = bc.BC(
    observation_space=env.observation_space,
    action_space=env.action_space,
)
with tempfile.TemporaryDirectory(prefix="dagger_example_") as tmpdir:
    print(tmpdir)
    dagger_trainer = SimpleDAggerTrainer(
        venv=venv,
        scratch_dir=tmpdir,
        expert_policy=expert,
        bc_trainer=bc_trainer,
    )
    dagger_trainer.train(2000)

print(dagger_trainer.policy)

reward, _ = evaluate_policy(dagger_trainer.policy, env, 10)
print("Reward:", reward)