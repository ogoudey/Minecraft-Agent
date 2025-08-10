import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from minecraft_msgs.srv import DigBlock
from rclpy.executors import MultiThreadedExecutor
import threading
import pygame
import sys
import random
from agent import command
from agent import state_former as stateformer
from agent.algorithms import ppo, sac
from agent.trainers import PPO_Trainer, SAC_Trainer
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
        """ Takes keyboard input and outputs a movement """

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                break
        
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
            future = self.dig_cli.call_async(DigBlock.Request(timeout=3.0))
            
        self.publisher.publish(msg)
        
        pygame.display.flip()
        self.clock.tick(60) # hz
    
    def demonstrate(self): 
        """ Takes keyboard input and returns an action """
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

    def halt_performance(self):
        self.publisher.publish(Twist())
        
class Performer(Node):
    def __init__(self):
        super().__init__('performer')
        
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.dig_cli = self.create_client(DigBlock, '/dig_block')
        
    def perform(self, action):
        """
        Takes an action (from KeybopardInput.demonstrate or from policy) and outputs a movement
        
        Action is:
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        """
        speed = 0.5
        look_speed = 1.0
        
        future = None
        msg = Twist()
        if action[0]:                  # W
            msg.linear.x = speed
        if action[1]:                  # D
            msg.linear.y = -speed
        if action[2]:                  # A
            msg.linear.y = speed
        if action[3]:                  # S
            msg.linear.x = -speed
        if action[4]:                  # SPACE
            msg.linear.z = speed
        if action[5]:                  # LEFT
            msg.angular.z = look_speed
        if action[6]:                  # RIGHT
            msg.angular.z = -look_speed
        if action[7]:                  # DOWN
            msg.angular.y = look_speed
        if action[8]:                  # UP
            msg.angular.y = -look_speed
        if action[9]:                  # B
            future = self.dig_cli.call_async(DigBlock.Request(timeout=2.0))
        self.publisher.publish(msg)
        return future
        
    def halt_performance(self):
        self.publisher.publish(Twist())

"""
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
"""

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


dispatch_dict = {
    "ppo": {
        "trainer": PPO_Trainer,
        "module": ppo,
    },
    "sac": {
        "trainer": SAC_Trainer,
        "module": sac,
    }
}
    
def teleop(checkpoint=None, replay_buffer=None, algorithm="ppo", t_iterations=10, iterations=10):
    rclpy.init()
    
    performer = Performer()
    state_former = stateformer.StateFormer()
    rewarder = command.Rewarder()
    exec_ = MultiThreadedExecutor()
    exec_.add_node(state_former)
    exec_.add_node(rewarder)
    if t_iterations > 0:
        kbi = KeyboardInput()
        exec_.add_node(kbi)
    
    t = threading.Thread(target=exec_.spin, daemon=True)
    t.start()
    
    
    dispatches = dispatch_dict[algorithm]
    trainer = dispatches["trainer"]()
    module = dispatches["module"]

    policy = module.Policy()
    nodes = (state_former, performer, rewarder, command)
    trainer.train(policy, checkpoint, replay_buffer, nodes)
    
    policy
    
    # Other algorithms#
    
    
    policy
    
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

def just_keys():
    rclpy.init()
    kbi = KeyboardInput()
    rewarder = command.Rewarder()
    exec_ = MultiThreadedExecutor()
    exec_.add_node(rewarder)
    exec_.add_node(kbi)
    t = threading.Thread(target=exec_.spin, daemon=True)
    t.start()
    
    
    command.reset_learner()
    time.sleep(1)
    
    
    
    
    while True:
        kbi.just_input()
        reward = rewarder.give()
        if reward > 0:
            print("Hurray! (Giving server some time)")
            kbi.halt_performance()
            time.sleep(1)
    exec_.shutdown()
    rclpy.shutdown()       
    
if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--checkpoint', type=str, default=None,
                        help='Path to a saved policy checkpoint (.pth)')
    parser.add_argument('--replay_buffer', type=str, default=None,
                        help='Path to a saved replay buffer (not fully implemented)')
    parser.add_argument('--a', type=str, default="ppo",
                        help='Algorithm (ppo or sac)')
    
    parser.add_argument('--test', action='store_true',
                        help='Test policy (provide --checkpoint <checkpoint>)')
    parser.add_argument('--i', type=int, default=0,
                        help='Iterations of policy-based performance')
    parser.add_argument('--t', type=int, default=0,
                        help='Iterations of teleop-based performance (broken on PPO)')
    args = parser.parse_args()
    if args.test:
        test(checkpoint=args.checkpoint)
    else:
        teleop(checkpoint=args.checkpoint, replay_buffer=args.replay_buffer, algorithm=args.a, t_iterations=args.t, iterations=args.i)
