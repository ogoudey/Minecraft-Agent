import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg        import Twist
from std_msgs.msg             import Float32       # example for /dig_block/progress
from sensor_msgs.msg          import Image, Imu, PointCloud2
from nav_msgs.msg             import Odometry      # assuming /player/ground_truth publishes Odometry
from tf2_msgs.msg             import TFMessage

TOPICS = {
    '/dig_block/progress' : (Float32,         10),
    '/player/ground_truth': (Odometry,        10),
    '/player/image_raw'   : (Image,            5),
    '/player/imu'         : (Imu,             10),
    '/player/pointcloud'  : (PointCloud2,      5),
    '/tf'                 : (TFMessage,       50),
}


class StateFormer(Node):
    def __init__(self):
        super().__init__('state_former')

        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        self._latest = {}          # topic → ROS message (or None)
        for topic, (msg_type, depth) in TOPICS.items():
            self._latest[topic] = None
            self.create_subscription(
                msg_type, topic,
                lambda msg, t=topic: self._callback(t, msg),
                qos_profile=qos if depth == 10 else QoSProfile(depth=depth)
            )

    def _callback(self, topic: str, msg):
        self._latest[topic] = msg

        
    def get_state(self, *keys):
        if not keys:
            return {k: v for k, v in self._latest.items()}   # shallow copy
        return {k: self._latest.get(k) for k in keys}
        
        
def main(args=None):
    rclpy.init(args=args)
    sf = StateFormer()
    rclpy.spin(sf)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
    
