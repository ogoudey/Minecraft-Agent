import rclpy
from rclpy.node import Node
from minecraft_msgs.srv import Command 


def reset_learner():
    rclpy.init()
    node = Node('reset_learner')
    cli  = node.create_client(Command, '/minecraft/command')

    if not cli.service_is_ready():
        node.get_logger().info('waiting for /minecraft/command …')
        cli.wait_for_service()

    future = cli.call_async(Command.Request(command='tp Dev 0 0 0'))

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

    node.destroy_node()

    rclpy.shutdown()

