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

topic_requirements_dispatcher = {
    'tree_detection'      : ['/player/image_raw', '/player/pointcloud'],
    'raw_image'           : ['/player/image_raw']
}

state_form_dispatcher = {
    'tree_detection'      : detect_trees,
    'raw_image'           : just_image
}

class StateFormer(Node):
    def __init__(self, state_type):
        super().__init__('state_former')
        self.state_type = state_type
        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        self._latest = {}          # topic → ROS message (or None)
        
        state_features = topic_requirements_dispatcher[state_type]
        for topic, (msg_type, depth) in TOPICS.items() if topic in state_features:
            self._latest[topic] = None
            self.create_subscription(
                msg_type, topic,
                lambda msg, t=topic: self._callback(t, msg),
                qos_profile=qos if depth == 10 else QoSProfile(depth=depth)
            )

    def _callback(self, topic: str, msg):
        self._latest[topic] = msg

        
    def get_state(self):
        """ Turns latest topics into state features, depending on the state_type of self. """
        return form_dispatcher[self.state_type](self._latest)

    def state_shape(self):
        return None # should be the shape needed to initialize the policy/actor network

def detect_trees(topics):
    raw_image = topics['/player/image_raw']
    bounding_boxes = get_boxes(raw_image)
    centers = get_centers(bounding_boxes)
    depths = get_depths(centers)
    tree_positions = get_positions(depths)
    filtered_positions = filter_positions(tree_positions)
    
    player_position = None          # somehow get player position
    
    # state <-- concatenate player_position and filtered_tree_positions
    
    return state

def just_image(topics):
    try:

        rgb = bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
    except CvBridgeError as e:
        raise RuntimeError(f'cv_bridge failed: {e}')

    return rgb

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
    
    
