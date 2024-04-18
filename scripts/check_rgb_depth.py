#!/usr/bin/env python

import rospy
import rosbag
import cv2
from sensor_msgs.msg import Image
from my_utils import getargs
from cv_bridge import CvBridge

def rgb_callback(msg):
    # Convert the ROS Image message to a NumPy array
    bridge = CvBridge()
    rgb_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    # Print the shape of the NumPy array
    rospy.loginfo("Received RGB message. Shape: %s", str(rgb_image.shape))
    # You can perform additional processing based on the shape or content of the array

def check_rgb_topic(bag_filename):
    # Initialize the ROS node
    rospy.init_node('evnet_depth_topic_checker', anonymous=True)

    # Subscribe to the RGB topic
    rospy.Subscriber('/rgb_depth_topic', Image, rgb_callback)

    # Open the ROS bag file
    with rosbag.Bag(bag_filename, 'r') as bag:
        # Iterate through messages in the bag
        for topic, msg, t in bag.read_messages(topics=['/rgb_depth_topic']):
            # Publish the message to the RGB topic subscriber
            rospy.loginfo("Publishing Depth message. Timestamp: %f", msg.header.stamp.to_sec())
            rgb_callback(msg)

if __name__ == '__main__':
    # Replace 'your_bag_file.bag' with the actual ROS bag file path
    args = getargs()
    bag_filename = args.input_dir+'output.bag'
    
    try:
        check_rgb_topic(bag_filename)
    except rospy.ROSInterruptException:
        pass
