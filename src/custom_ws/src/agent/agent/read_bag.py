import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

import rosbag2_py

def read_ros2_bag(bag_path):
    rclpy.init()

    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # Build topic-type map
    topics_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topics_types}

    print("Topics:")
    for name, type_str in type_map.items():
        print(f"  {name}: {type_str}")

    # Read and deserialize messages
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        msg_type_str = type_map[topic]  # e.g., 'sensor_msgs/msg/JointState'
        msg_class = get_message(msg_type_str)
        msg = deserialize_message(data, msg_class)

        print(f"\n[{topic}] @ {timestamp}")
        print(msg)

    rclpy.shutdown()

"""
def read_ros2_bag(bag_path):
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topics_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topics_types}

    print("Topics in bag:", list(type_map.keys()))

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        print(f"Topic: {topic}, Timestamp: {timestamp}, Raw Data: {data}")
"""

import os

def find_latest_rosbag(base_dir="/ws/rosbags"):
    bags = [d for d in os.listdir(base_dir)]
    bags.sort(reverse=True)  # newest first
    for bag in bags:
        path = os.path.join(base_dir, bag)
        if os.path.isfile(os.path.join(path, "metadata.yaml")):
            return path
    raise FileNotFoundError("No valid rosbag directories found.")

bag_path = find_latest_rosbag()
read_ros2_bag(bag_path)
