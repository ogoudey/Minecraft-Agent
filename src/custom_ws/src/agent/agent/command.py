import rclpy
from rclpy.node import Node
from minecraft_msgs.srv import Command 
from std_msgs.msg import Int32

# from overlay_minecraft_msgs import IsBlock

import time

def reset_learner():
    #rclpy.init()
    node = Node('reset_learner')
    cli  = node.create_client(Command, '/minecraft/command')
    if not cli.service_is_ready():
        node.get_logger().info('waiting for /minecraft/command …')
        cli.wait_for_service()
        
    size  = 7
    y     = -61
    block = 'minecraft:grass_block'
    for x in range(-size, size + 1):
        for z in range(-size, size + 1):
            cmd = f'setblock {x} {y} {z} {block} replace'
            future = cli.call_async(Command.Request(command=cmd))
            rclpy.spin_until_future_complete(node, future)     # wait for result
            try:
                resp = future.result()
                if not resp.success:
                    node.get_logger().warn(f'({x},{y},{z}) failed: {resp.message}')
            except Exception as e:
                node.get_logger().error(f'setblock failed at ({x},{y},{z}): {e}')
    
    future = cli.call_async(Command.Request(command='kill Dev'))
   
    rclpy.spin_until_future_complete(node, future)   # block until reply arrives
    
    try:
        resp = future.result()

    except Exception as e:
        node.get_logger().error(f'service call failed: {e}')
    
    time.sleep(1) # Guessing this might help
    future = cli.call_async(Command.Request(command='tp Dev 0 -60 0'))

    rclpy.spin_until_future_complete(node, future)   # block until reply arrives
    
    try:
        resp = future.result()
    except Exception as e:
        node.get_logger().error(f'service call failed: {e}')  
        
    future = cli.call_async(Command.Request(command='clear Dev *'))

    rclpy.spin_until_future_complete(node, future)   # block until reply arrives
    
    try:
        resp = future.result()
    except Exception as e:
        node.get_logger().error(f'service call failed: {e}')
        
    """  
    # Don't need this one yet.
    future = cli.call_async(Command.Request(command='item replace entity Dev armor.head with minecraft_ros2:velodyne_vlp16'))

    rclpy.spin_until_future_complete(node, future)   # block until reply arrives

    try:
        resp = future.result()

    except Exception as e:
        node.get_logger().error(f'service call failed: {e}')
    """    
        
    future = cli.call_async(Command.Request(command='give Dev minecraft:stone_axe'))

    rclpy.spin_until_future_complete(node, future)   # block until reply arrives

    try:
        resp = future.result()

    except Exception as e:
        node.get_logger().error(f'service call failed: {e}')
        
        
    future = cli.call_async(Command.Request(command='time set day'))

    rclpy.spin_until_future_complete(node, future)   # block until reply arrives

    try:
        resp = future.result()

    except Exception as e:
        node.get_logger().error(f'service call failed: {e}')
        
        
    future = cli.call_async(Command.Request(command='setblock 0 -60 4 minecraft:oak_log'))

    rclpy.spin_until_future_complete(node, future)   # block until reply arrives

    try:
        resp = future.result()

    except Exception as e:
        node.get_logger().error(f'service call failed: {e}')
        
    future = cli.call_async(Command.Request(command='setblock 0 -59 4 minecraft:oak_log'))

    rclpy.spin_until_future_complete(node, future)   # block until reply arrives

    try:
        resp = future.result()

    except Exception as e:
        node.get_logger().error(f'service call failed: {e}')
        
    future = cli.call_async(Command.Request(command='setblock 0 -58 4 minecraft:oak_log'))

    rclpy.spin_until_future_complete(node, future)   # block until reply arrives

    try:
        resp = future.result()

    except Exception as e:
        node.get_logger().error(f'service call failed: {e}')
        
    future = cli.call_async(Command.Request(command='setblock 0 -57 4 minecraft:oak_log'))

    rclpy.spin_until_future_complete(node, future)   # block until reply arrives

    try:
        resp = future.result()

    except Exception as e:
        node.get_logger().error(f'service call failed: {e}')

    time.sleep(1)
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

    def cb(self, msg: Int32):
        self._latest = msg.data
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
    
    
