import rclpy
from rclpy.node import Node
from minecraft_msgs.srv import Command 
from std_msgs.msg import Int32

# from overlay_minecraft_msgs import IsBlock
import random
import time

bedrock_y = -64

device_coords = f"0 {bedrock_y + 1} -22"
x, y, z = map(int, device_coords.split())

valid_tree_coords = [i for i in range(-5, 6) if abs(i) > 2]

def run_mc_command(cli, node, command):
    print("Command:", command)
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
        
    run_mc_command(cli, node, 'gamerule doImmediateRespawn true')
    run_mc_command(cli, node, "spawnpoint Dev 0 -60 0")
    
    # Cross-episode device parts
    run_mc_command(cli, node, f'setblock {x} {y} {z + 1} minecraft:comparator destroy')
    run_mc_command(cli, node, f'setblock {x} {y} {z + 2} minecraft_ros2:redstone_pub_sub')
    run_mc_command(cli, node, f'data merge block {x} {y} {z + 2} {{CustomName:"redstone/my_block", pollRate:10}}')

def reset_learner():
    #rclpy.init()
    node = Node('reset_learner')
    cli  = node.create_client(Command, '/minecraft/command')
    if not cli.service_is_ready():
        node.get_logger().info('waiting for /minecraft/command …')
        cli.wait_for_service()
    
    
    
    run_mc_command(cli, node, f'fill -6 {bedrock_y + 1} -6 6 {bedrock_y + 11} 6 minecraft:air')
    
    
    tree_x = random.choice(valid_tree_coords)
    tree_z = random.choice(valid_tree_coords)
    tree_base_y = bedrock_y + 1
    run_mc_command(cli, node, f'place feature minecraft:oak {tree_x} {tree_base_y} {tree_z}')
    

    

    run_mc_command(cli, node,
        f'setblock {x} {y} {z} minecraft:repeating_command_block[facing=south]{{Command:"execute if block {tree_x} {tree_base_y} {tree_z} minecraft:oak_log if block {tree_x} {tree_base_y + 1} {tree_z} minecraft:oak_log if block {tree_x} {tree_base_y + 2} {tree_z} minecraft:oak_log if block {tree_x} {tree_base_y + 3} {tree_z} minecraft:oak_log"}} destroy'

    )
    run_mc_command(cli, node, f'setblock {x} {y} {z - 1} minecraft:redstone_torch destroy')

    run_mc_command(cli, node, "kill Dev")
    run_mc_command(cli, node, 'kill @e[type=item]')
    run_mc_command(cli, node, 'clear Dev *')
    
    run_mc_command(cli, node, f'setblock {tree_x} {bedrock_y} {tree_z} minecraft:bedrock destroy') # run far from placing oak to give it time (?)

    """  
    # Don't need these yet.
    run_mc_command('item replace entity Dev armor.head with minecraft_ros2:velodyne_vlp16')
    """
    
    
      
        
    run_mc_command(cli, node, 'give Dev minecraft:stone_axe')   
    run_mc_command(cli, node, 'time set day')
    
    
    
    

    # Per-episode device parts
    
    

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
        self._latest = 0
        self.log = log
        initialize_resetter()
        

    def cb(self, msg: Int32):
        self._latest = 1 - msg.data # Policy Evaluation comes from within Minecraft.
        if self.log:
            print(self.give())

    def give(self):
        reward = 0
        if self._latest > 0: # Tree is broken
            reward += 1
        else:
            reward -= 0.001
        
        return reward

def main(args=None):
    rclpy.init(args=args)
    reset_learner()
    
    r = Rewarder(log=True)
    rclpy.spin(r)
    rclpy.shutdown()
    
    
