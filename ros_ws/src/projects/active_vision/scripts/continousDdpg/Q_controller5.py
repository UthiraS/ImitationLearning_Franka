#!/usr/bin/env python3
from tensorflow import keras
import tensorflow as tf
from tensorflow.keras import optimizers as opt
#https://stackoverflow.com/questions/43990046/tensorflow-blas-gemm-launch-failed
physical_devices = tf.config.list_physical_devices('GPU') 
for device in physical_devices:
    tf.config.experimental.set_memory_growth(device, True)
import numpy as np
import wandb
import time
import rospkg
import rospy
from config import *
from replay_buffer import *
from networks import *
from environment import RosEnvironment
from agent import Agent
# from baselines.common.schedules import LinearSchedule
import gc
import psutil
from colorama import Fore, Style

np.random.seed(0)

if __name__ == '__main__':
    config0 = dict(
        learning_rate_actor = ACTOR_LR,
        learning_rate_critic = CRITIC_LR,
        batch_size = BATCH_SIZE,
        architecture = NETWORK_TYPE,
        infra = OS_NAME,
        env = ENV_NAME
    )
    print("Starting init...")
    wandb.init(
       mode = RECORD,
       project = "tensorflow2_ddpg",
       config = config0,
    )
    print("Finished init")
    wandb.define_metric('Steps')
    wandb.define_metric('Current direction', step_metric = 'Steps')
    wandb.define_metric('Critic loss', step_metric = 'Steps')

    base_dir = rospkg.RosPack().get_path('active_vision')
    path = base_dir + "/model/"
    print("Building env...")
    env = RosEnvironment()
    n_episodes = MAX_EPISODES

    num_inputs = env.infoSpaceSize
    num_actions = env.actionSpaceSize

    convergence_target = 0.7
    print("Building agent...")

    agent = Agent(env)
    
    scores = []
    std_devs = []
    avg_scores = []
    eps_history = []

    best = 0
    
    # beta_annealing = LinearSchedule(n_episodes, initial_p = PRIORITY_BETA, final_p = 1.0)

    evaluation = False
    steps = 0
    print("Begining run...")
    
    for i in range(n_episodes):
        print("---------------------- EPISODE {} --------------------".format(i+1))
        start_time = time.time()
        done = False
        score = 0
        cStep = 0
        ramUsage = psutil.virtual_memory()[2]
        
        if(ramUsage > 94):
            print("Uh oh, too much RAM used!")
            print("saving...")
            agent.save()
            print("saved")
            break
            
        observation = env.reset()
        
        while not done:
            steps += 1
            # time.sleep(0.5)
            if(i > PRETRAIN_LENGTH):
                action = agent.choose_action(observation, evaluation)
            else:
                action = env.cheat()
            # Manual control
            # a1 = float(input("direction: "))
            # action = [a1, 1]
            
            new_observation, reward, done, cStep, direction = env.step(action)
            print("step: {} | reward: {} | done: {} | cStep: {} | action: {},{}".format(steps, reward, done, cStep, np.squeeze(np.clip(action[1] * MAX_STEP_SIZE, -MAX_STEP_SIZE, MAX_STEP_SIZE)), direction))
            
            agent.remember(observation, action, reward, new_observation, done)
            
            observation = new_observation
            
            if(steps % LEARN_RATE == 0 and steps > MIN_SIZE_BUFFER and i > PRETRAIN_LENGTH):
                for j in range(20):
                    agent.learn()
                    print(f"{Fore.BLUE}-------------------- Agent Learning ---------------{Style.RESET_ALL}")
            
            score += reward 
            
            if LENGTH_THRESHOLD <= cStep:
                done = True
                
            
            if(steps % TARGET_UPDATE_RATE == 0 and steps > MIN_SIZE_BUFFER):
                agent.update_target_networks()
                print(f"{Fore.RED}-------------------- Updating Target Networks ---------------{Style.RESET_ALL}")
                
            
            if steps % SAVE_FREQUENCY == 0:
                agent.save(steps)
                print(f"{Fore.GREEN}-------------------- SAVING THE MODEL ---------------{Style.RESET_ALL}")
            
            wandb.log({'Steps': steps, 'Critic loss': agent.getCriticLoss(), 'Current direction': direction}, commit=False)
            print("Step {}, Critic loss {}, Current direction {}".format(steps, agent.getCriticLoss(), direction))
            # score = random.random() * 20
            # done = True
        
        scores.append(score)
        if(len(scores) > 100):
            scores = scores[1:101]
        avg_score = np.mean(scores)
        
        if avg_score > best and i >= 100:
            best = avg_score
        avg_scores.append(avg_score)
        
        std = np.std(scores)
        std_devs.append(std)

        print("Game number {}, Episode {}, Episode reward {}, Average Reward {}, Time {}".format(i, steps, score, avg_score, round(time.time() - start_time, 2)))

        wandb.log({'Game number': i, 'Steps': steps, 
                   'Episode reward': score,
                   'Average reward 100 episodes': avg_score, 
                   'Time taken': round(time.time() - start_time, 2)})
       
        print("------------------------------------- EPISODE END -----------------------------------------".format(i+1))
        # Test the policy
        if (i + 1) % EVALUATION_FREQUENCY == 0:
            print(f"{Fore.YELLOW}-------------------- EVALUATING THE MODEL ---------------{Style.RESET_ALL}")
            evaluation = True
            states = env.reset()
            done = False
            score = 0
            while not done:
                env.render()
                action = agent.choose_action(states, evaluation)
                new_states, reward, done, cStep, direction = env.step(action)
                score += reward
                states = new_states
                if LENGTH_THRESHOLD <= cStep:
                    done = True
                
            wandb.log({'Game number': i+1, 
                       'Steps': steps, 
                       'Evaluation score': score})
            evaluation = False

    print("saving...")
    agent.save(steps)
    print("saved")
