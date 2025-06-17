import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import numpy as np
import cv2
import torch
from geometry_msgs.msg        import Twist
from std_msgs.msg             import Float32       # example for /dig_block/progress
from sensor_msgs.msg          import Image, Imu, PointCloud2
from nav_msgs.msg             import Odometry      # assuming /player/ground_truth publishes Odometry
from tf2_msgs.msg             import TFMessage
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()

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

        
    def get_state(self, topic):
        return img_msg_to_tensor(self._latest[topic])
        return {k: self._latest.get(k) for k in keys}
        

def img_msg_to_tensor(msg, resize=(84, 84)):
    """
    sensor_msgs.msg.Image  ->  torch.FloatTensor  (C,H,W) in [0,1]
    """
    try:

        rgb = bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
    except CvBridgeError as e:
        raise RuntimeError(f'cv_bridge failed: {e}')


    if resize is not None:
        
        rgb = cv2.resize(rgb, resize, interpolation=cv2.INTER_AREA)   # (h,w,3)


    rgb = np.transpose(rgb, (2, 0, 1)).astype(np.float32) / 255.0     # (3,H,W)
    return torch.from_numpy(rgb.astype(np.float32))  
            
def main(args=None):
    rclpy.init(args=args)
    sf = StateFormer()
    rclpy.spin(sf)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
    
