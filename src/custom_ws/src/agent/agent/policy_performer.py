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
from state_former import StateFormer
import algorithm as rl
import torch
import numpy as np

class KeyboardInput(Node):
    def __init__(self):
        super().__init__('keyboard_input')
        
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.dig_cli = self.create_client(DigBlock, '/dig_block')
        
        pygame.init()
        
        screen = pygame.display.set_mode((400,300))
        pygame.display.set_caption("Key Press")
        screen.fill((0,0,0))

        clock = pygame.time.Clock()
        
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
            clock.tick(60) # hz
            
            reward = command.reward()
        pygame.quit()

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
            future = self.dig_cli.call_async(DigBlock.Request(timeout=10.0))
        self.publisher.publish(msg)
        return future
        
    def halt_performance(self):
        self.publisher.publish(Twist())
        
def train(args=None):
    rclpy.init()
    node = Performer()
    state_former = StateFormer()
    exec_ = MultiThreadedExecutor()
    exec_.add_node(state_former)

    t = threading.Thread(target=exec_.spin, daemon=True)
    t.start() 
    
    
    policy = rl.RGBPolicy()    
    
    len_episode = 500
    iterations = int(input("# iterations: "))
    
    for i in range(0, iterations):
        command.reset_learner()
        
        future = None
        done = False
        #############
        #  ROLLOUT  #
        #############
        buffer = []
        state = state_former.get_state('/player/image_raw')

        for step in range(0, len_episode):
            print("Step:", step)

            if state['/player/image_raw'] is None:
                state = state_former.get_state('/player/image_raw')
                continue
            state_t = img_msg_to_tensor(state['/player/image_raw']).unsqueeze(0)
            print(state_t.shape)
            
            with torch.no_grad():
                act_idx, logp, val = policy.act(state_t)
            
            #future = node.perform(action)
            future = node.perform(one_hot(act_idx.item()))
            rclpy.spin_once(node, timeout_sec=0.01)
            if future and not future.done():
                rclpy.spin_until_future_complete(node, future)
            
                           
            next_state = state_former.get_state('/player/image_raw')
            reward = command.reward()
            if reward > 0:
                done = True
            
            buffer.append((state_t, act_idx.item(), logp.item(), val.item(), reward, done))
            
            state = next_state
            if done:
                break
                
        node.halt_performance()
        #  END ROLLOUT
        
        ###################
        # COMPUTE RETURNS #
        ###################
        
        policy.compute_returns(buffer)
        
        
    exec_.shutdown()
    state_former.destroy_node()    
    node.destroy_node()
    rclpy.shutdown()
        
        
# Helper for state -> tensor

from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()

def one_hot(index: int, size: int = 10):
    vec = [0.0] * size
    vec[index] = 1.0
    return vec

def img_msg_to_tensor(msg, resize=(84, 84)):
    """
    sensor_msgs.msg.Image  ->  torch.FloatTensor  (C,H,W) in [0,1]
    """
    try:

        rgb = bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
    except CvBridgeError as e:
        raise RuntimeError(f'cv_bridge failed: {e}')


    if resize is not None:
        import cv2
        rgb = cv2.resize(rgb, resize, interpolation=cv2.INTER_AREA)   # (h,w,3)


    rgb = np.transpose(rgb, (2, 0, 1)).astype(np.float32) / 255.0     # (3,H,W)
    return torch.from_numpy(rgb)        # torch.FloatTensor    
    
def teleop(args=None):
    rclpy.init(args=args)
    kbi = KeyboardInput()
    rclpy.spin(kbi)
    rclpy.shutdown()

if __name__ == '__main__':
    train()
