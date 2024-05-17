import numpy as np
import gym
import time
from stable_baselines3 import PPO, DDPG
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.ppo import MlpPolicy
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.noise import NormalActionNoise
from stable_baselines3.common.policies import BasePolicy
from stable_baselines3.common import policies, torch_layers
import torch as th
from os.path import exists
import tensorflow as tf

from imitation.algorithms import bc
from imitation.data import rollout
from imitation.data.wrappers import RolloutInfoWrapper
from imitation.algorithms.bc import reconstruct_policy
from imitation.algorithms.dagger import SimpleDAggerTrainer
from imitation.util.logger import WandbOutputFormat
from imitation.scripts.common import common as common_config
import wandb
from gazeboEnv import gazeboWrapper, environmentDetails, easyDetails, hardDetails, emptyWrapper
import rospkg

from wandb.integration.sb3 import WandbCallback




import tempfile

baseDir = rospkg.RosPack().get_path('active_vision')
modelsDir = baseDir + "/scripts/RL/models/"
PRETRAIN = 10000
EPOCH_LENGTH = 500
TEST_LENGTH = 20
EASY = [0, 6]
MED = [1, 5, 8, 9]
HARD = [2, 3, 4, 7]
BOTHE_M = [0, 1, 5, 6, 8, 9]
SAVED_EXPERT = {0:[94.15, 2.94],500:[94.6,1.83], 1000:[92.6,5.94], 1500:[93.5, 2.2], 2000:[94, 2.97], 2500:[93.8, 2.8], 3000:[93.45, 2.99], 3500:[94.4, 2.13], 4000:[91.2, 8.42], 4500:[93.6, 2.24], 5000:[93.8, 4.41], 5500:[93.2, 3.04], 6000:[93.85, 2.29], 6500:[92.4, 7.97], 7000:[93.9, 2.21], 7500:[93.8, 3.84], 8000:[92.4, 4.58], 8500:[93.7, 2.26], 9000:[94.45, 3.12], 9500:[93.75, 3.16]}
SAVED_RANDOM = {0:[66.90, 13.23],500:[69.15, 14.20], 1000:[67.90, 10.24], 1500:[68.15, 12.09], 2000:[71.75, 15.89], 2500:[68.35, 15.63], 3000:[65.1, 10.16], 3500:[68.1, 14.65], 4000:[70.3, 12.68], 4500:[70.4, 12.72], 5000:[72.25, 16.2], 5500:[71.5, 14.01], 6000:[66.95, 12.4], 6500:[68.2, 12.83], 7000:[68.9, 13.03], 7500:[66.4, 12.5], 8000:[69.7, 12.83], 8500:[70.6, 13.57], 9000:[70.7, 15.51], 9500:[66.6, 13.04]}
feature_type="ESF"
def buildEnv(random_poses=True, seed=None, target= easyDetails(), length=False):
    env = gazeboWrapper(feature_type,random_poses=random_poses, seed=seed, details=target)
    env.strictLength = length
    env = Monitor(env)
    return env

def evaluatePretrain(expert, clone, naive, startTime, envGen = buildEnv, iter = 0, target=easyDetails()):
    #rebuild the environment before each test to ensure identical run conditions
    trainingTime = time.time()
    rS = iter
    numberOfTests = TEST_LENGTH
    env = envGen(random_poses=True, seed=rS, target=target, length=True)
    expert.updateEnv(env)
    print("Expert")
    if(EPOCH_LENGTH*iter in SAVED_EXPERT):
        print("Loaded")
        r2 = SAVED_EXPERT[EPOCH_LENGTH*iter][0]
        s2 = SAVED_EXPERT[EPOCH_LENGTH*iter][1]
    else:
        r2, s2 = evaluate_policy(expert, env, numberOfTests)
        env = envGen(random_poses=True, seed=rS, target=target, length=True)
    print("Imitation")
    r1, s1 = evaluate_policy(clone.policy, env, numberOfTests)
    env = envGen(random_poses=True, seed=rS, target=target, length=True)
    print("random")
    if(EPOCH_LENGTH*iter in SAVED_RANDOM):
        print("Loaded")
        r3 = SAVED_RANDOM[EPOCH_LENGTH*iter][0]
        s3 = SAVED_RANDOM[EPOCH_LENGTH*iter][1]
    else:
        r3, s3 = evaluate_policy(naive, env, numberOfTests)
    evalTime = time.time()
    ret = "I {}: from base of {:.2f}, {:.2f} expert reached {:.2f}, {:.2f} and clone copied to {:.2f}, {:.2f}\n"
    timeStatement = "Last step to this one took {}, this evaluation took {}.\n"
    print(ret.format(EPOCH_LENGTH*iter, r3, s3, r2, s2, r1, s1))
    print(timeStatement.format(trainingTime-startTime, evalTime-trainingTime))
    with open('/home/uthira/mer_lab/ros_ws/src/projects/active_vision/scripts/RL/features_var_log.txt', 'a') as f:
        f.write(ret.format(EPOCH_LENGTH*iter, r3, s3, r2, s2, r1, s1))
        f.write(timeStatement.format(trainingTime-startTime, evalTime-trainingTime))
    return r1

def testPolicy(model, env, name):
    obs = env.reset()
    done = False
    while not done:
        action, state = model.predict(obs)
        obs, reward, done, info = env.step(action)
        env.render()
    input("Finished with model "+ name)

class RandomWrapper(BasePolicy):
    def __init__(self, environment,
        observation_space ,
        action_space , seed=1
        ):
        super().__init__(observation_space,
            action_space,)
        self.env = environment
        self.seed = seed
        self.rng = np.random.default_rng(self.seed)

    def updateEnv(self, enviornment):
        self.env = enviornment
        self.rng = np.random.default_rng(self.seed)

    def _predict(self, observation, deterministic = False):
        ret = tf.convert_to_tensor((self.rng.random(size=(2,))))
        return ret
class CheatWrapper(BasePolicy):

    def __init__(self, environment,
        observation_space ,
        action_space
        ):
        super().__init__(observation_space,
            action_space,)
        self.env = environment
        

    def updateEnv(self, enviornment):
        self.env = enviornment

    def _predict(self, observation, deterministic = False):
        ret = self.env.cheat()
        # print("Prediction = ...")
        # print(observation[0][-1], ret[0]*360, ret[1]*360)
        ret = tf.convert_to_tensor(np.array(ret))
        return ret

class modelSetup(object):
    def __init__(self, alg, policy, env):
        self.alg = alg
        n_actions = env.action_space.shape[-1]
        action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))
        self.model = alg(policy="MlpPolicy", action_noise=action_noise, env=env, tensorboard_log=f"runs/0")
        # self.model = alg(policy=policy, env=env, tensorboard_log=f"runs/0")
        self.env = env
        self.name = "Base"

    def setup(self):
        if(exists(modelsDir+self.name+".zip")):
            self.load()
        else:
            print("base training")
            run = wandb.init(
                # mode = "disabled",
                project="sb3",
                config=dict(env=self.env, name=self.name),
                sync_tensorboard=True,  # auto-upload sb3's tensorboard metrics
                monitor_gym=True,  # auto-upload the videos of agents playing the game
                # save_code=True,  # optional
            )
            self.model.learn(total_timesteps=PRETRAIN,log_interval=10, callback=WandbCallback())
            self.save()

    def save(self):
        self.model.save(modelsDir+self.name)

    def load(self):
        self.model = self.alg.load(modelsDir+self.name)

class untrainedSetup(modelSetup):
    def __init__(self, alg, policy, env):
        super().__init__(alg, policy, env)
        self.name = "Untrained"

    def setup(self):
        if(exists(modelsDir+self.name+".zip")):
            self.load()
        else:
            print("no training")
            self.save()

class bcSetup(object):
    def __init__(self, expert, policy, env, name="MissingEasyandMedium_dagger", size=32):
        self.expert = expert
        self.env = env
        self.name = name
        if("LOAD" == policy):
            print("Starting loading")
            self.model = bc.BC(
                observation_space=env.observation_space,
                action_space=env.action_space, 
                policy=reconstruct_policy(modelsDir+"n_"+self.name)
            )
            print("Done loading!")
        else:
            extractor = (
                torch_layers.CombinedExtractor
                if isinstance(env.observation_space, gym.spaces.Dict)
                else torch_layers.FlattenExtractor
            )
            bc_policy = policies.ActorCriticPolicy(
                observation_space=env.observation_space,
                action_space=env.action_space,
                # Set lr_schedule to max value to force error if policy.optimizer
                # is used by mistake (should use self.optimizer instead).
                lr_schedule=lambda _: th.finfo(th.float32).max,
                features_extractor_class=extractor,
                net_arch=[size, size]
            )
            self.model = bc.BC(
                observation_space=env.observation_space,
                action_space=env.action_space,
                policy = bc_policy
            )
        

    def save(self):
        self.model.save_policy(modelsDir+"n_"+self.name)

    def load(self):
        self.model = reconstruct_policy(modelsDir+"n_"+self.name)

    def train(self, steps = EPOCH_LENGTH):
        print("dagger imitating")
        venv = DummyVecEnv([lambda: self.env])
        # custom_logger, log_dir = common_config.setup_logging(log_format_strs=['wandb'])
        
        with tempfile.TemporaryDirectory(prefix="dagger_example_") as tmpdir:
            print(tmpdir)
            dagger_trainer = SimpleDAggerTrainer(
                venv=venv,
                scratch_dir=tmpdir,
                # expert_policy=self.expert.model,
                expert_policy=self.expert,
                bc_trainer=self.model,
                # custom_logger = custom_logger 
            )
            print("training")
            dagger_trainer.train(steps)

def sampleAblation(removedItemIndices=[], envName="All_dagger", load=False, size=32):
    rS = 0
    startTime = time.time()
    test_target = mergeOptionList(removedItemIndices, envName)
    train_target = mergeOptionList(removedItemIndices, envName)
    cEnv = buildEnv(random_poses=True, seed=rS, target=train_target, length=False)
    expert = CheatWrapper(cEnv, cEnv.observation_space, cEnv.action_space)
    if load:
        clone = bcSetup(expert, "LOAD", cEnv, envName, size)
    else:
        clone = bcSetup(expert, MlpPolicy, cEnv, envName, size)
    fool = RandomWrapper(cEnv, cEnv.observation_space, cEnv.action_space, rS)
    currentBest = 70
    # currentBest = 92.1
    newScore = 0
    with open('/home/uthira/mer_lab/ros_ws/src/projects/active_vision/scripts/RL/features_var_log.txt', 'a') as f:
        f.write("---------------{}---------------\n".format(train_target.name))
    for i in range(19):
        startTime = time.time()
        expert.updateEnv(cEnv)
        clone.train()
        newScore = evaluatePretrain(expert, clone.model, fool, startTime, envGen = buildEnv, iter = i+0, target=test_target)
        if(newScore > currentBest):
            currentBest = newScore
            clone.save()
        
        
def testEnvs():
    rS = 1
    iterations = 15
    targets = buildFullOptionList()
    cEnv = buildEnv(random_poses=True, seed=rS, target=targets[0], length=True)
    expert = CheatWrapper(cEnv, cEnv.observation_space, cEnv.action_space)
    random = RandomWrapper(cEnv, cEnv.observation_space, cEnv.action_space, rS)
    foolW = bcSetup(expert, MlpPolicy, cEnv)
    for target in targets:
        env = buildEnv(random_poses=True, seed=rS, target=target, length=True)
        expert.updateEnv(env)
        r2, s2 = evaluate_policy(expert, env, iterations)
        env = buildEnv(random_poses=True, seed=rS, target=target, length=True)
        r3, s3 = evaluate_policy(foolW.model.policy, env, iterations)
        env = buildEnv(random_poses=True, seed=rS, target=target, length=True)
        random.updateEnv(env)
        r1, s1 = evaluate_policy(random, env, iterations)
        ret = "Env {}: from base of {:.2f}, {:.2f}/{:.2f}, {:.2f}, expert reached {:.2f}, {:.2f}\n"
        print(ret.format(target.name, r3, s3, r1, s1, r2, s2))
        with open('test_log.txt', 'a') as f:
            f.write(ret.format(target.name, r3, s3, r1, s1, r2, s2))

def testOpt():
    rS = 1
    iterations = 20
    # targets = ["hard", "easy", "med"]
    targets = ["easy"]
    cEnv = buildEnv()
    expert = CheatWrapper(cEnv, cEnv.observation_space, cEnv.action_space)
    for target in targets:
        env = buildEnv(random_poses=True, seed=rS, target=target, length=True)
        expert.updateEnv(env)
        r2, s2 = evaluate_policy(expert, env, iterations)

def mergeOptionList(skip=[], envName="All"):
    targets = buildFullOptionList()
    #Remove all specified indices
    targets = [target for i, target in enumerate(targets) if i not in skip]
    target = targets[0]
    target.name=envName
    for i in range(1, len(targets)):
        target.merge(targets[i])
    return target

def buildFullOptionList():
    # objects = ["009_gelatin_box", 
    # "055_baseball", "072-a_toy_airplane",
    # "010_potted_meat_can", "003_cracker_box", "035_power_drill", 
    # "006_mustard_bottle", "021_bleach_cleanser", "013_apple", "Weisshai_Great_White_Shark"]
    # IDs = [8, 41, 51, 9, 2, 28, 5, 19, 12, 65]

    # objects = ["Weisshai_Great_White_Shark"]
    # IDs = [65]

    # objects = ["009_gelatin_box",
    # "055_baseball", "072-a_toy_airplane",
    # "010_potted_meat_can",  "035_power_drill",
    # "006_mustard_bottle", "021_bleach_cleanser", "013_apple", "Weisshai_Great_White_Shark"]
    # IDs = [8, 41, 51, 9,  28,  5, 19, 12, 65]


    #     For cross-validation, your dataset would be split into different training and testing sets for each fold. Here's how your dataset could be organized for 5-fold cross-validation:

    # - **Fold 1**: Trains on 8 objects, tests on 3 (e.g., trains on baseball, toy airplane, ...; tests on gelatin box, power drill, apple).
    # - **Fold 2**: Trains on 9 objects, tests on 2 (e.g., trains on gelatin box, baseball, ...; tests on toy airplane, Great White Shark).
    # - **Fold 3**: Trains on 9 objects, tests on 2 (e.g., trains on gelatin box, toy airplane, ...; tests on baseball, bleach cleanser).
    # - **Fold 4**: Trains on 9 objects, tests on 2 (e.g., trains on gelatin box, baseball, ...; tests on cracker box, mustard bottle).
    # - **Fold 5**: Trains on 9 objects, tests on 2 (e.g., trains on gelatin box, baseball, ...; tests on potted meat can, tomato soup can).

    # This setup ensures that each object is included in the test set exactly once, allowing for a comprehensive evaluation of your model's performance across all items.
    

    
    # Crossvalidation Set 1

    # objects= ["055_baseball", "072-a_toy_airplane", "010_potted_meat_can", "003_cracker_box", "005_tomato_soup_can", "006_mustard_bottle", "021_bleach_cleanser"]
    # IDs= [41, 51, 9, 2, 4, 5, 19]
    # Test Objects= ["009_gelatin_box", "035_power_drill", "013_apple"]
    # Test IDs= [8, 28, 12]

    # Crossvalidation Set 2

    # objects= ['009_gelatin_box', '055_baseball', '010_potted_meat_can', '003_cracker_box', '035_power_drill', '005_tomato_soup_can', '006_mustard_bottle', '021_bleach_cleanser', '013_apple']
    # IDs= [8, 41, 9, 2, 28, 4, 5, 19, 12]

    # Test Objects= ['072-a_toy_airplane', 'Weisshai_Great_White_Shark']
    # Test IDs= [51, 65]
    # objects= ['009_gelatin_box', '055_baseball']
    # IDs= [8, 41]
    # Crossvalidation Set 3

    objects= ['009_gelatin_box', '072-a_toy_airplane', '010_potted_meat_can', '003_cracker_box', '035_power_drill', '005_tomato_soup_can', '006_mustard_bottle', '013_apple', 'Weisshai_Great_White_Shark']
    IDs= [8, 51, 9, 2, 28, 4, 5, 12, 65] 
    # Test Objects= ['055_baseball', '021_bleach_cleanser']
    # Test IDs= [41, 19]

    # Crossvalidation Set 4

    # Objects= ['009_gelatin_box', '055_baseball', '072-a_toy_airplane', '010_potted_meat_can', '035_power_drill', '005_tomato_soup_can', '021_bleach_cleanser', '013_apple', 'Weisshai_Great_White_Shark']
    # IDs= [8, 41, 51, 9, 28, 4, 19, 12, 65] 
    # Test Objects= ['003_cracker_box', '006_mustard_bottle']
    # Test IDs= [2, 5]

    # Crossvalidation Set 5

    # Objects= ['009_gelatin_box', '055_baseball', '072-a_toy_airplane', '003_cracker_box', '035_power_drill', '006_mustard_bottle', '021_bleach_cleanser', '013_apple', 'Weisshai_Great_White_Shark']
    # IDs= [8, 41, 51, 2, 28, 5, 19, 12, 65] 
    # Test Objects= ['010_potted_meat_can', '005_tomato_soup_can']
    # Test IDs= [9, 4]



    targets = []
    for i in range(len(objects)):
        target = environmentDetails(objects[i])
        target.insert(IDs[i], 1, 359, objects[i])
        targets.append(target)
    return targets

def showRun():
    rS = 0

    model = bcSetup(None, "LOAD", emptyWrapper(), "All_dagger_old").model.policy
    env = buildEnv(random_poses=True, seed=rS, target=hardDetails(), length=True)
    print("Imitation")
    obs = env.reset()
    r = model.predict(obs, state=None, deterministic=True)
    print(r, r[0])
    for i in range(10):
        obs, _, _, _ = env.step(r[0])
        print(r, r[0])
        r = model.predict(obs, state=None, deterministic=True)
    # r1, s1 = evaluate_policy(model, env, 10)


if __name__ == '__main__':

    print(tf.sysconfig.get_build_info())
    # Check TensorFlow version
    print("TensorFlow version:", tf.__version__)

    # List all available GPUs
    gpus = tf.config.list_physical_devices('GPU')
    if gpus:
        for gpu in gpus:
            print("GPU available:", gpu)
    else:
        print("No GPU available, using CPU instead.")
    
    # sampleAblation([], "Train_set_large_GASD1", False, 128)

    # sampleAblation([8, 28, 12], "Train_set_small_HAF_cross1", False, 32) #crossvalidation 1
    # sampleAblation([4], "Train_set_large_FPFH_changed", False, 128) 
    # #crossvalidation 2
    sampleAblation([41, 19], "Train_set_large_ESF_cross3", False, 128) # #crossvalidation 3
    # sampleAblation([2, 5], "Train_set_large_GASD_cross4", False, 128) # #crossvalidation 4
    # sampleAblation([9, 4], "Train_set_large_GASD_cross5", False, 128) # #crossvalidation 5


