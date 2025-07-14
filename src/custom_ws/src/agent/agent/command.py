import rclpy
from rclpy.node import Node
from minecraft_msgs.srv import Command 
from std_msgs.msg import Int32

# from overlay_minecraft_msgs import IsBlock

import time

def run_mc_command(cli, node, command):
    future = cli.call_async(Command.Request(command=command))
    rclpy.spin_until_future_complete(node, future)   # block until reply arrives
    try:
        resp = future.result()
    except Exception as e:
        node.get_logger().error(f'service call failed: {e}')
        
    if command == "kill Dev":
        time.sleep(1) # Guessing this might help

def initialize_resetter():
    node = Node('reset_learner')
    cli  = node.create_client(Command, '/minecraft/command')
    if not cli.service_is_ready():
        node.get_logger().info('waiting for /minecraft/command …')
        cli.wait_for_service()
        

    run_mc_command(cli, node, "spawnpoint Dev 0 -60 0")

def reset_learner():
    #rclpy.init()
    node = Node('reset_learner')
    cli  = node.create_client(Command, '/minecraft/command')
    if not cli.service_is_ready():
        node.get_logger().info('waiting for /minecraft/command …')
        cli.wait_for_service()
        

    run_mc_command(cli, node, "kill Dev")
    run_mc_command(cli, node, 'kill @e[type=item]')
    run_mc_command(cli, node, 'clear Dev *')
    
    

    """  
    # Don't need these yet.
    run_mc_command('item replace entity Dev armor.head with minecraft_ros2:velodyne_vlp16')
    
    run_mc_command('fill -6 -60 -6 6 10 6 minecraft:air')
    """    
        
    run_mc_command(cli, node, 'give Dev minecraft:stone_axe')   
    run_mc_command(cli, node, 'time set day')
        
    run_mc_command(cli, node, 'place feature minecraft:oak 0 -60 4')
    
    node.destroy_node()

    #rclpy.shutdown()
    
class Rewarder(Node):
    def __init__(self, log=False):
        super().__init__('rewarder')

        self.create_subscription(
            Int32,                                  # message class
            '/redstone/my_block/output',            # topic name
            self.cb,                                # callback
            10
        )
        self._latest = 1
        self.log = log
        initialize_resetter()
        

    def cb(self, msg: Int32):
        self._latest = msg.data # Policy Evaluation comes from within Minecraft.
        if self.log:
            print(self.give())

    def give(self):
        reward = 0
        if self._latest == 0: # Tree is broken
            reward += 1
        else:
            reward -= 0.01
        
        return reward

def main(args=None):
    rclpy.init(args=args)
    reset_learner()
    
    r = Rewarder(log=True)
    rclpy.spin(r)
    rclpy.shutdown()
    
    
