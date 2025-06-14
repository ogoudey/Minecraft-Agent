import rclpy
from rclpy.node import Node
from minecraft_msgs.srv import Command 


def reset_learner():
    #rclpy.init()
    node = Node('reset_learner')
    cli  = node.create_client(Command, '/minecraft/command')

    if not cli.service_is_ready():
        node.get_logger().info('waiting for /minecraft/command …')
        cli.wait_for_service()

    future = cli.call_async(Command.Request(command='tp Dev 0 -60 0'))

    rclpy.spin_until_future_complete(node, future)   # block until reply arrives
    
    try:
        resp = future.result()
        node.get_logger().info(f"success={resp.success}  msg='{resp.message}'")
    except Exception as e:
        node.get_logger().error(f'service call failed: {e}')
        
        
    future = cli.call_async(Command.Request(command='clear Dev *'))

    rclpy.spin_until_future_complete(node, future)   # block until reply arrives
    
    try:
        resp = future.result()
        node.get_logger().info(f"success={resp.success}  msg='{resp.message}'")
    except Exception as e:
        node.get_logger().error(f'service call failed: {e}')
        
        
    future = cli.call_async(Command.Request(command='item replace entity Dev armor.head with minecraft_ros2:velodyne_vlp16'))

    rclpy.spin_until_future_complete(node, future)   # block until reply arrives

    try:
        resp = future.result()
        node.get_logger().info(f"success={resp.success}  msg='{resp.message}'")
    except Exception as e:
        node.get_logger().error(f'service call failed: {e}')
        
        
    future = cli.call_async(Command.Request(command='give Dev minecraft:stone_axe'))

    rclpy.spin_until_future_complete(node, future)   # block until reply arrives

    try:
        resp = future.result()
        node.get_logger().info(f"success={resp.success}  msg='{resp.message}'")
    except Exception as e:
        node.get_logger().error(f'service call failed: {e}')
        
        
    future = cli.call_async(Command.Request(command='time set day'))

    rclpy.spin_until_future_complete(node, future)   # block until reply arrives

    try:
        resp = future.result()
        node.get_logger().info(f"success={resp.success}  msg='{resp.message}'")
    except Exception as e:
        node.get_logger().error(f'service call failed: {e}')
        
        
    future = cli.call_async(Command.Request(command='setblock 0 -60 4 minecraft:oak_block'))

    rclpy.spin_until_future_complete(node, future)   # block until reply arrives

    try:
        resp = future.result()
        node.get_logger().info(f"success={resp.success}  msg='{resp.message}'")
    except Exception as e:
        node.get_logger().error(f'service call failed: {e}')
        
    future = cli.call_async(Command.Request(command='setblock 0 -59 4 minecraft:oak_block'))

    rclpy.spin_until_future_complete(node, future)   # block until reply arrives

    try:
        resp = future.result()
        node.get_logger().info(f"success={resp.success}  msg='{resp.message}'")
    except Exception as e:
        node.get_logger().error(f'service call failed: {e}')
        
    future = cli.call_async(Command.Request(command='setblock 0 -58 4 minecraft:oak_block'))

    rclpy.spin_until_future_complete(node, future)   # block until reply arrives

    try:
        resp = future.result()
        node.get_logger().info(f"success={resp.success}  msg='{resp.message}'")
    except Exception as e:
        node.get_logger().error(f'service call failed: {e}')
        
    future = cli.call_async(Command.Request(command='setblock 0 -57 4 minecraft:oak_block'))

    rclpy.spin_until_future_complete(node, future)   # block until reply arrives

    try:
        resp = future.result()
        node.get_logger().info(f"success={resp.success}  msg='{resp.message}'")
    except Exception as e:
        node.get_logger().error(f'service call failed: {e}')


    node.destroy_node()

    #rclpy.shutdown()

def reward():
    node = Node('compute_reward')
    cli  = node.create_client(Command, '/minecraft/command') 
    #1
    future = cli.call_async(Command.Request(command='execute if block 0 -60 4 minecraft:oak_log'))
    rclpy.spin_until_future_complete(node, future)   # block until reply arrives
    try:
        resp = future.result()
        if not resp.success:
            print("Success!")
            return 1
    except Exception as e:
        node.get_logger().error(f'service call failed: {e}')
    #2
    future = cli.call_async(Command.Request(command='execute if block 0 -59 4 minecraft:oak_log'))
    rclpy.spin_until_future_complete(node, future)   # block until reply arrives
    try:
        resp = future.result()
        if not resp.success:
            print("Success!")
            return 1
    except Exception as e:
        node.get_logger().error(f'service call failed: {e}')
    #3
    future = cli.call_async(Command.Request(command='execute if block 0 -58 4 minecraft:oak_log'))
    rclpy.spin_until_future_complete(node, future)   # block until reply arrives
    try:
        resp = future.result()
        if not resp.success:
            print("Success!")
            return 1
    except Exception as e:
        node.get_logger().error(f'service call failed: {e}')
    return 0


