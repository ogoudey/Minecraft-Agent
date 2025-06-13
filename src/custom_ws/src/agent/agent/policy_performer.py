import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from minecraft_msgs.srv import DigBlock

import sys
print(sys.executable)
print(sys.path)



# from custom messages import the custom message

import pygame

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


def main(args=None):
    rclpy.init(args=args)
    kbi = KeyboardInput()
    rclpy.spin(kbi)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
