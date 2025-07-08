#!/usr/bin/env python3
import os
import argparse

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message

from sensor_msgs.msg import Image
from unitree_go.msg import LowState
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
    # This will hold: { timestamp → [ (order_index, filename), … ] }
    files_by_ts = {}

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
        '/lowstate': LowState,
        '/imu_topic':   LowState,
        '/camera/color/image_raw':   Image,
        '/camera/aligned_depth_to_color/image_raw': Image,
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


        # 5) Dispatch based on topic
        if topic == '/gt_topic':
            # Prefer the header's timestamp if present
            sec  = msg.header.stamp.sec
            # note: ROS 2 uses .nanosec
            nsec = getattr(msg.header.stamp, 'nanosec', msg.header.stamp.nanosec)
            ts_us = int(sec * 1e6 + nsec / 1e3)  # convert to microseconds
            p = msg.pose.position
            q = msg.pose.orientation
            gt_file.write(f"{ts_us} {p.x} {p.y} {p.z} {q.x} {q.y} {q.z} {q.w}\n")

        # elif topic == '/lowstate':
        #     line = " ".join(str(x) for x in msg.position)
        #     joint_file.write(f"{ts_us} {line}\n")
        elif topic == '/lowstate':
            # # Prefer the header's timestamp if present
            sec  = msg.head[0]
            # # note: ROS 2 uses .nanosec
            nsec = msg.head[1]
            # ts_us = int(sec * 1e6 + nsec / 1e3)  # convert to microseconds
            print("sec", sec, "nsec", nsec)
            ts_us = msg.tick * 1000
            # 1) extract the custom IMUState
            imu = msg.imu_state

            # 2) pull out its fields
            q   = imu.quaternion       # float32[4]
            g   = imu.gyroscope        # float32[3]
            a   = imu.accelerometer    # float32[3]
            # rpy = imu.rpy              # float32[3]
            # t   = imu.temperature      # int (e.g. uint8)

            # 3) write a line: ts, quat(4), gyro(3), accel(3), rpy(3), temp
            imu_file.write(
              f"{ts_us} "
              f"{g[0]} {g[1]} {g[2]} "
              f"{a[0]} {a[1]} {a[2]} "
              f"{0} {0} {0} "
              f"{q[0]} {q[1]} {q[2]} {q[3]} " # wxyz
              f"\n"
            )

        # elif topic == '/camera/color/image_raw':
        #     # Prefer the header's timestamp if present
        #     sec  = msg.header.stamp.sec
        #     # note: ROS 2 uses .nanosec
        #     nsec = getattr(msg.header.stamp, 'nanosec', msg.header.stamp.nanosec)
        #     ts_us = int(sec * 1e6 + nsec / 1e3)  # convert to microseconds
        #     cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        #     fn = f"{ts_us}_rgb.png"
        #     cv2.imwrite(os.path.join(rgb_dir, fn), cv_img)
        #     files_by_ts.setdefault(ts_us, []).append((2, fn))

        # elif topic == '/camera/aligned_depth_to_color/image_raw':
        #     # Prefer the header's timestamp if present
        #     sec  = msg.header.stamp.sec
        #     # note: ROS 2 uses .nanosec
        #     nsec = getattr(msg.header.stamp, 'nanosec', msg.header.stamp.nanosec)
        #     ts_us = int(sec * 1e6 + nsec / 1e3)  # convert to microseconds
        #     cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        #     fn = f"{ts_us}_depth_rgb.png"
        #     fn_tmp = f"{ts_us}_depth_event.png" # todo: temporary placeholder
        #     cv2.imwrite(os.path.join(depth_dir, fn), cv_img)
        #     files_by_ts.setdefault(ts_us, []).append((0, fn))
        #     files_by_ts.setdefault(ts_us, []).append((1, fn_tmp))

        # elif HAS_EVENT and topic == '/event_topic':
        #     # --- event exporting is highly format-specific ---
        #     # here you could, e.g., buffer up (ts_us, msg.x, msg.y, msg.polarity)
        #     # and then use a Python AEDAT4 writer (e.g. via PyAedat) to dump
        #     # a bona fide .aedat4 file. This placeholder just writes raw bytes:
        #     # Prefer the header's timestamp if present
        #     sec  = msg.header.stamp.sec
        #     # note: ROS 2 uses .nanosec
        #     nsec = getattr(msg.header.stamp, 'nanosec', msg.header.stamp.nanosec)
        #     ts_us = int(sec * 1e6 + nsec / 1e3)  # convert to microseconds
        #     event_file.write(msg.serialize())
        #     fn = f"{ts_us}_depth_event.png"
        #     files_by_ts.setdefault(ts_us, []).append((1, fn))

    # 6) Clean up
    gt_file.close()
    joint_file.close()
    imu_file.close()
    if HAS_EVENT:
        event_file.close()

    # 7) Write out the combined manifest in timestamp order
    manifest_path = os.path.join(out_dir, 'realsense_timestamp.txt')
    with open(manifest_path, 'w') as manifest:
        for ts in sorted(files_by_ts):
            # sort by our order_index so depth_rgb (0), depth_event (1), rgb (2)
            for _, fn in sorted(files_by_ts[ts], key=lambda x: x[0]):
                manifest.write(fn + '\n')

if __name__ == "__main__":
    p = argparse.ArgumentParser(
        description="Convert a ROS 2 bag into the CEAR folder layout"
    )
    p.add_argument("bag_path", help="path to the ROS2 bag (directory or .db3 prefix)")
    p.add_argument("out_dir",  help="where to drop gt_pose.txt, mini_cheetah_joint.txt, ...")
    args = p.parse_args()

    ros2bag_to_cear(args.bag_path, args.out_dir)
