#!/usr/bin/env python3
import os
import argparse

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message

from sensor_msgs.msg import Image, Imu, JointState
from geometry_msgs.msg import PoseStamped
try:
    from event_msgs.msg import Event
    HAS_EVENT = True
except ImportError:
    HAS_EVENT = False

from cv_bridge import CvBridge
import cv2

def ros2bag_to_cear(bag_path: str, out_dir: str):
    # 1) Prepare output folders & files
    os.makedirs(out_dir, exist_ok=True)
    gt_file     = open(os.path.join(out_dir, 'gt_pose.txt'),      'w')
    joint_file  = open(os.path.join(out_dir, 'mini_cheetah_joint.txt'), 'w')
    imu_file    = open(os.path.join(out_dir, 'vectornav.txt'),    'w')
    rgb_dir     = os.path.join(out_dir, 'rgb');     os.makedirs(rgb_dir,  exist_ok=True)
    depth_dir   = os.path.join(out_dir, 'depth');   os.makedirs(depth_dir,exist_ok=True)
    if HAS_EVENT:
        event_file = open(os.path.join(out_dir, 'event.aedat4'), 'wb')  # placeholder, see below

    # 2) Open the ROS 2 bag
    storage_opts   = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_opts = ConverterOptions(input_serialization_format='cdr',
                                      output_serialization_format='cdr')
    reader = SequentialReader()
    reader.open(storage_opts, converter_opts)

    bridge = CvBridge()

    # 3) Map topic names → message classes
    topic_types = {
        '/gt_topic':    PoseStamped,
        '/joint_topic': JointState,
        '/imu_topic':   Imu,
        '/rgb_topic':   Image,
        '/depth_topic': Image,
    }
    if HAS_EVENT:
        topic_types['/event_topic'] = Event

    # 4) Iterate through all messages
    while reader.has_next():
        topic, data, _bag_ts = reader.read_next()
        if topic not in topic_types:
            continue

        msg_type = topic_types[topic]
        msg = deserialize_message(data, msg_type)

        # Prefer the header's timestamp if present
        sec  = msg.header.stamp.sec
        # note: ROS 2 uses .nanosec
        nsec = getattr(msg.header.stamp, 'nanosec', msg.header.stamp.nanosecond)
        ts_us = int(sec * 1e6 + nsec / 1e3)  # convert to microseconds

        # 5) Dispatch based on topic
        if topic == '/gt_topic':
            p = msg.pose.position
            q = msg.pose.orientation
            gt_file.write(f"{ts_us} {p.x} {p.y} {p.z} {q.x} {q.y} {q.z} {q.w}\n")

        elif topic == '/joint_topic':
            line = " ".join(str(x) for x in msg.position)
            joint_file.write(f"{ts_us} {line}\n")

        elif topic == '/imu_topic':
            g = msg.angular_velocity
            a = msg.linear_acceleration
            o = msg.orientation
            imu_file.write(
                f"{ts_us} "
                f"{g.x} {g.y} {g.z} "
                f"{a.x} {a.y} {a.z} "
                f"{o.x} {o.y} {o.z} {o.w}\n"
            )

        elif topic == '/rgb_topic':
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            fn = f"{ts_us}_rgb.png"
            cv2.imwrite(os.path.join(rgb_dir, fn), cv_img)

        elif topic == '/depth_topic':
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            fn = f"{ts_us}_depth.png"
            cv2.imwrite(os.path.join(depth_dir, fn), cv_img)

        elif HAS_EVENT and topic == '/event_topic':
            # --- event exporting is highly format-specific ---
            # here you could, e.g., buffer up (ts_us, msg.x, msg.y, msg.polarity)
            # and then use a Python AEDAT4 writer (e.g. via PyAedat) to dump
            # a bona fide .aedat4 file. This placeholder just writes raw bytes:
            event_file.write(msg.serialize())

    # 6) Clean up
    gt_file.close()
    joint_file.close()
    imu_file.close()
    if HAS_EVENT:
        event_file.close()


if __name__ == "__main__":
    p = argparse.ArgumentParser(
        description="Convert a ROS 2 bag into the CEAR folder layout"
    )
    p.add_argument("bag_path", help="path to the ROS2 bag (directory or .db3 prefix)")
    p.add_argument("out_dir",  help="where to drop gt_pose.txt, mini_cheetah_joint.txt, ...")
    args = p.parse_args()

    ros2bag_to_cear(args.bag_path, args.out_dir)
