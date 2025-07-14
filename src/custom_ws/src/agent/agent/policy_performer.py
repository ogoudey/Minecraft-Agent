import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from minecraft_msgs.srv import DigBlock
from rclpy.executors import MultiThreadedExecutor
import threading
import pygame
import sys
import random
import command
import state_former as stateformer
import algorithm as rl
import torch
import numpy as np
import os
import time
import matplotlib.pyplot as plt

class KeyboardInput(Node):
    def __init__(self):
        super().__init__('keyboard_input')
        
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.dig_cli = self.create_client(DigBlock, '/dig_block')
        
        pygame.init()
        
        screen = pygame.display.set_mode((400,300))
        pygame.display.set_caption("Key Press")
        screen.fill((0,0,0))

        self.clock = pygame.time.Clock()
    
    def just_input(self): 
        while True:
            pygame.event.get()
            pressed = pygame.key.get_pressed()
            msg = Twist()
            if pressed[pygame.K_w]:
                msg.linear.x = 1.0
            if pressed[pygame.K_d]:
                msg.linear.y = -1.0
            if pressed[pygame.K_a]:
                msg.linear.y = 1.0
            if pressed[pygame.K_s]:
                msg.linear.x = 1.0 
            if pressed[pygame.K_SPACE]:
                msg.linear.z = 1.0
            if pressed[pygame.K_LEFT]:
                msg.angular.z = 1.0
            if pressed[pygame.K_RIGHT]:
                msg.angular.z = -1.0
            if pressed[pygame.K_DOWN]:
                msg.angular.y = 1.0
            if pressed[pygame.K_UP]:
                msg.angular.y = -1.0
            if pressed[pygame.K_b]:
                future = self.dig_cli.call_async(DigBlock.Request(timeout=10.0))
                
            self.publisher.publish(msg)
            
            pygame.display.flip()
            self.clock.tick(60) # hz
            
            reward = command.reward()
        pygame.quit()
    
    def demonstrate(self): 

        pygame.event.get()
        pressed = pygame.key.get_pressed()
        action = [0.0] * 10
        if pressed[pygame.K_w]:
            action[0] = True
        if pressed[pygame.K_d]:
            action[1] = True
        if pressed[pygame.K_a]:
            action[2] = True
        if pressed[pygame.K_s]:
            action[3] = True 
        if pressed[pygame.K_SPACE]:
            action[4] = True
        if pressed[pygame.K_LEFT]:
            action[5] = True
        if pressed[pygame.K_RIGHT]:
            action[6] = True
        if pressed[pygame.K_DOWN]:
            action[7] = True
        if pressed[pygame.K_UP]:
            action[8] = True
        if pressed[pygame.K_b]:
            action[9] = True
        
        pygame.display.flip()
        self.clock.tick(60) # hz

        return action
        
class Performer(Node):
    def __init__(self):
        super().__init__('performer')
        
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.dig_cli = self.create_client(DigBlock, '/dig_block')
        
    def perform(self, action):
        #       Action is:
        #   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        #
        future = None
        msg = Twist()
        if action[0]:                  # W
            msg.linear.x = 1.0
        if action[1]:                  # D
            msg.linear.y = -1.0
        if action[2]:                  # A
            msg.linear.y = 1.0
        if action[3]:                  # S
            msg.linear.x = -1.0
        if action[4]:                  # SPACE
            msg.linear.z = 1.0
        if action[5]:                  # LEFT
            msg.angular.z = 1.0
        if action[6]:                  # RIGHT
            msg.angular.z = -1.0
        if action[7]:                  # DOWN
            msg.angular.y = 1.0
        if action[8]:                  # UP
            msg.angular.y = -1.0
        if action[9]:                  # B
            future = self.dig_cli.call_async(DigBlock.Request(timeout=4.0))
        self.publisher.publish(msg)
        return future
        
    def halt_performance(self):
        self.publisher.publish(Twist())

def test(checkpoint):
    rclpy.init()
    node = Performer()
    state_former = stateformer.StateFormer()
    rewarder = command.Rewarder()
    exec_ = MultiThreadedExecutor()
    exec_.add_node(state_former)
    exec_.add_node(rewarder)

    t = threading.Thread(target=exec_.spin, daemon=True)
    t.start()
    
    policy = rl.RGBPolicy()    
    # Load policy checkpoint

    print(f"Loading checkpoint from: {checkpoint}")
    state_dict = torch.load(checkpoint) # , map_location=device
    policy.load_state_dict(state_dict)
    print("Checkpoint loaded successfully.")
    
    len_episode = 500
    command.reset_learner()
    state = state_former.get_state('/player/image_raw')
    
    for step in range(0, len_episode):
            with torch.no_grad():
                act_idx, logp, val = policy.act(state)
            
            #future = node.perform(action)
            future = node.perform(one_hot(act_idx.item()))
            rclpy.spin_once(node, timeout_sec=0.01)
            if future and not future.done():
                rclpy.spin_until_future_complete(node, future)
            
                           
            state = state_former.get_state('/player/image_raw')
            
            reward = rewarder.give()
            if reward > 0:
                print("Hurray! (Giving server some time)")
                break

def plot(plot_topics):
    for plot_topic in plot_topics:
        plt.figure()
        plt.plot(plot_topic)
        plt.xlabel("Episode")
        plt.ylabel("Cumulative Reward") # uhh - how to do this?
        plt.title("Training Progress")
        plt.grid(True)
        plt.savefig("reward_plot.png")  # Optional: save to disk
        plt.show()
        
def one_hot(index: int, size: int = 10):
    vec = [0.0] * size
    vec[index] = 1.0
    return vec
    
def teleop(checkpoint=None, replay_buffer=None, t_iterations=10, iterations=10):
    rclpy.init()
    kbi = KeyboardInput()
    node = Performer()
    state_former = stateformer.StateFormer()
    rewarder = command.Rewarder()
    exec_ = MultiThreadedExecutor()
    exec_.add_node(state_former)
    exec_.add_node(rewarder)
    exec_.add_node(kbi)
    
    t = threading.Thread(target=exec_.spin, daemon=True)
    t.start()

    policy = rl.RGBPolicy()    
    # Load policy checkpoint
    if checkpoint is not None:
        print(f"Loading checkpoint from: {checkpoint}")
        state_dict = torch.load(checkpoint) # , map_location=device
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
        node.halt_performance()
        command.reset_learner()
        future = None
        done = False
        buffer = []
        state = state_former.get_state('/player/image_raw')
        for step in range(0, len_episode):
            action = kbi.demonstrate() # no logprobs or value head...
            future = node.perform(action)
            rclpy.spin_once(node, timeout_sec=0.01)
            if future and not future.done():
                rclpy.spin_until_future_complete(node, future)
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
        node.halt_performance()
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
            with torch.no_grad():
                act_idx, logp, val = policy.act(state)
            
            #future = node.perform(action)
            future = node.perform(one_hot(act_idx.item()))
            rclpy.spin_once(node, timeout_sec=0.01)
            if future and not future.done():
                rclpy.spin_until_future_complete(node, future)
            
                           
            next_state = state_former.get_state('/player/image_raw')
            reward = rewarder.give()
            cumulative_reward += reward
            if reward > 0:
                print("Hurray! (Giving server some time)")
                time.sleep(1)
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
    
    ### Saving ###
    os.makedirs('/ws/custom/checkpoints', exist_ok=True)
    torch.save(policy.state_dict(), '/ws/custom/checkpoints/policy_w_' + str(iterations) + '_iters.pth')
    ###
    
    plot_topics = [episode_rewards]
    plot(plot_topics)
             
    exec_.shutdown()
    state_former.destroy_node()    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--checkpoint', type=str, default=None,
                        help='Path to a saved policy checkpoint (.pth)')
    parser.add_argument('--replay_buffer', type=str, default=None,
                        help='Path to a saved replay buffer (not fully implemented)')
    parser.add_argument('--test', action='store_true',
                        help='Test policy (provide --checkpoint <checkpoint>)')
    args = parser.parse_args()
    if args.test:
        test(checkpoint=args.checkpoint)
    else:
        teleop(checkpoint=args.checkpoint)
