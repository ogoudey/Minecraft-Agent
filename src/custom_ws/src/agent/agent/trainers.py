import time
import torch
import torch.optim as optim
import random
import numpy as np
from collections import deque


""" Trainers must implement train(), as consistant with dispatcher """ 

class PPO_Trainer:

    def __init__(self):
        pass
        
    def train(self, policy, checkpoint, replay_buffer, nodes, len_episode=500, t_iterations=0, iterations=100):
        state_former, performer, rewarder, command = nodes
        # Load policy checkpoint
        if checkpoint is not None:
            print(f"Loading checkpoint from: {checkpoint}")
            state_dict = torch.load(checkpoint)
            policy.load_state_dict(state_dict)
            print("Checkpoint loaded successfully.")
        
        if replay_buffer:
            ### Consume Replay Buffer ###
            while len(replay_buffer) > 0:
                replay = replay_buffer.pop(random.randrange(len(replay_buffer)))
                policy.compute_returns(replay)
            #
        
        len_episode = 500
        
        for t in range(0, t_iterations):
            performer.halt_performance()
            command.reset_learner()
            future = None
            done = False
            buffer = []
            state = state_former.get_state('/player/image_raw')
            for step in range(0, len_episode):
                action = kbi.demonstrate() # no logprobs or value head...
                future = performer.perform(action)
                rclpy.spin_once(performer, timeout_sec=0.01)
                if future and not future.done():
                    rclpy.spin_until_future_complete(performer, future)
                next_state = state_former.get_state('/player/image_raw')
                reward = rewarder.give()
                if reward > 0:
                    print("Hurray! (Giving server some time)")
                    time.sleep(1)
                    done = True
                #buffer.append((state, action, logp.item(), val.item(), reward, done))
            #buffer_replay.append(buffer)   
                if done:
                    break
        if replay_buffer:
            ### Consume Replay Buffer Again ###
            while len(replay_buffer) > 0:
                replay = replay_buffer.pop(random.randrange(len(replay_buffer)))
                policy.compute_returns(replay)
            #
               
        episode_rewards = []
        
        for i in range(0, iterations):
            performer.halt_performance()
            command.reset_learner()
            
            future = None
            done = False
            cumulative_reward = 0
            #############
            #  ROLLOUT  #
            #############
            buffer = []
            state = state_former.get_state('/player/image_raw')

            for step in range(0, len_episode):
                print(f'Step: {step}', end='\r', flush=True)
                with torch.no_grad():
                    act_idx, logp, val = policy.act(state)
                
                #future = node.perform(action)
                future = performer.perform(one_hot(act_idx.item()))
                rclpy.spin_once(node, timeout_sec=0.01)
                if future and not future.done():
                    rclpy.spin_until_future_complete(node, future)
                
                               
                next_state = state_former.get_state('/player/image_raw')
                reward = rewarder.give()
                cumulative_reward += reward
                if reward > 0:
                    print("Hurray! (Giving server some time)")
                    time.sleep(1)
                    rewarder._latest = 0 # hopeful overridew
                    done = True
                
                buffer.append((state, act_idx.item(), logp.item(), val.item(), reward, done))
                
                if (i + 1) % 3 == 0:
                    os.makedirs('/ws/custom/checkpoints', exist_ok=True)
                    torch.save(policy.state_dict(), f'/ws/custom/checkpoints/policy_iter_{i+1}.pth')
                    plot_topics = [episode_rewards]
                    plot(plot_topics)
                
                state = next_state
                if done:
                    break
                  
            node.halt_performance()
            #  END ROLLOUT
            
            ###################
            # COMPUTE RETURNS #
            ###################
            
            policy.compute_returns(buffer)
            
            episode_rewards.append(cumulative_reward)
            
class SAC_Trainer:

    def __init__(self):
        pass
        
        
    def train(self, policy, checkpoint, replay_buffer, nodes, len_episode=500, t_iterations=0, iterations=100):
        state_former, performer, rewarder, command = nodes
        # ignore checkpoint and replay_buffer
        # ignore t_iterations
        
        for i in range(0, iterations):
            performer.halt_performance()
            command.reset_learner()
            
            future = None
            done = False
            cumulative_reward = 0
            #############
            #  ROLLOUT  #
            #############
            buffer = []
            state = state_former.get_state('/player/image_raw')
        
            for step in tqdm(range(1, len_episode)):
                print(f'Step: {step}', end='\r', flush=True)
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        

