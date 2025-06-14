import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from minecraft_msgs.srv import DigBlock
import pygame
import sys

import command

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
        pygame.quit()

def train(args=None):
    len_episode = 100
    iterations = int(input("# iterations: "))
    for i in range(0, iterations):
        command.reset_learner()
        for step in range(0, len_episode):
            pass
            # wait for action to free
            # get state
            # action = policy(state)
            # do action
    
    
    
def teleop(args=None):
    rclpy.init(args=args)
    kbi = KeyboardInput()
    rclpy.spin(kbi)
    rclpy.shutdown()

if __name__ == '__main__':
    train()
