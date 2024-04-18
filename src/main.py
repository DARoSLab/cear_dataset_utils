import os
import cv2
import rosbag
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray 
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from cv_bridge import CvBridge
import rospy


from my_utils import getargs
from my_utils import get_png_files 
from event_msgs.msg import Event

def record_to_rosbag(gt_path, joint_path, imu_path, rgb_path, depth_path,evnet_path, output_bag_filename, args):
    # Initialize a ROS bag file
    with rosbag.Bag(output_bag_filename, 'w') as bag:
        # Create a CvBridge to convert between OpenCV images and ROS images
        bridge = CvBridge()


        # 1. ground truth (3 position+4 orientation)
        if args.gt_pose==True:
            print('reading gt data...')
            with open(gt_path, 'r') as file1:
                gt_sensor = file1.readlines()
                len_gt = len(gt_sensor)
                # ground truth
                for idx, line_gt in  enumerate(gt_sensor):
                    record_gt = map(float, line_gt.split())
                    time_gt_sec = int(record_gt[0]//1e6)
                    time_gt_nsec = int(record_gt[0]%1e6*1000)
                    data_gt = record_gt[1:]
                    msg_gt = PoseStamped()
                    msg_gt.pose.position.x = data_gt[0]
                    msg_gt.pose.position.y = data_gt[1] 
                    msg_gt.pose.position.z = data_gt[2]
                    msg_gt.pose.orientation.x = data_gt[3]
                    msg_gt.pose.orientation.y = data_gt[4]
                    msg_gt.pose.orientation.z = data_gt[5]
                    msg_gt.pose.orientation.w = data_gt[6]
                    # Create a header with the timestamp
                    msg_gt.header = Header(seq=idx, stamp=rospy.Time(time_gt_sec, time_gt_nsec), frame_id="gt_pose")
                    bag.write('/gt_topic', msg_gt, t=rospy.Time(time_gt_sec, time_gt_nsec))
                    if idx==0:
                        min_idx = time_gt_sec
                    elif idx==len_gt-1:
                        max_idx = time_gt_sec
                print(min_idx, max_idx, (max_idx-min_idx)/60, len_gt)
                file1.close()
        # 2. joint angle (12*1)
        if args.joint==True:
            print('reading joint angle...')
            with open(joint_path, 'r') as file2:
                joint_sensor = file2.readlines()
                file2.close()
                len_joint = len(joint_sensor)
                 # joint
            for idx, line_joint in enumerate(joint_sensor):
                record_joint = map(float, line_joint.split())
                time_joint_sec = int(record_joint[0]//1e6)
                time_joint_nsec = int(record_joint[0]%1e6*1000)
                data_joint = record_joint[1:]
                msg_joint = JointState() 
                msg_joint.position = data_joint
                msg_joint.name = ['FR_abduct', 'FR_thigh','FR_knee', 'FL_abduct', 'FL_thigh', 'FL_knee',
                                  'HR_abduct', 'HR_thigh', 'HR_knee', 'HL_abduct', 'HL_thigh', 'HL_knee']
                msg_joint.header = Header(seq=idx, stamp=rospy.Time(time_joint_sec, time_joint_nsec), 
                                          frame_id="joint")
                bag.write('/joint_topic', msg_joint, t=rospy.Time(time_joint_sec, time_joint_nsec))
                if idx==0:
                    min_idx = time_joint_sec
                elif idx==len_joint-1:
                    max_idx = time_joint_sec
            print(min_idx, max_idx, (max_idx-min_idx)/60, len_joint)
        # 3. imu (3 gyro+3 acc+4 oreintation+3*covariance)
        if args.imu==True:
            print('reading imu data...')
            with open(imu_path, 'r') as file3:
                # Read the lines from each file
                imu_sensor = file3.readlines()
                len_imu = len(imu_sensor)
                # imu
                for idx, line_imu in enumerate(imu_sensor):
                    record_imu = map(float, line_imu.split())
                    time_imu_sec = int(record_imu[0]//1e6)
                    time_imu_nsec = int(record_imu[0]%1e6*1000)
                    data_imu = record_imu[1:]
                    msg_imu  = Imu()
                    # gyro
                    msg_imu.angular_velocity.x = data_imu[0]
                    msg_imu.angular_velocity.y = data_imu[1]
                    msg_imu.angular_velocity.z = data_imu[2]

                    # acc
                    msg_imu.linear_acceleration.x = data_imu[3]
                    msg_imu.linear_acceleration.y = data_imu[4]
                    msg_imu.linear_acceleration.z = data_imu[5]

                    # orientation
                    msg_imu.orientation.x = data_imu[9]
                    msg_imu.orientation.y = data_imu[10]
                    msg_imu.orientation.z = data_imu[11]
                    msg_imu.orientation.w = data_imu[12]

                    msg_imu.orientation_covariance =  [1, 0, 0, 0, 1, 0, 0, 0, 1]
                    msg_imu.linear_acceleration_covariance =  [1, 0, 0, 0, 1, 0, 0, 0, 1]
                    msg_imu.angular_velocity_covariance =  [1, 0, 0, 0, 1, 0, 0, 0, 1]
                    msg_imu.header = Header(seq=idx, stamp=rospy.Time(time_imu_sec, time_imu_nsec), 
                                          frame_id="imu")
                    bag.write('/imu_topic', msg_imu, t=rospy.Time(time_imu_sec, time_imu_nsec))
                    if idx==0:
                        min_idx = time_imu_sec
                    elif idx==len_imu-1:
                        max_idx = time_imu_sec
                print(min_idx, max_idx, (max_idx-min_idx)/60, len_imu)
        # 4. rgb data 
        if args.rgb==True:
            print('read rgb data . ..')
            png_files = get_png_files(rgb_path)
            for idx_png, png_path in enumerate(png_files):
                 # Read the image using OpenCV
                file_path = rgb_path+png_path
                image = cv2.imread(file_path, cv2.IMREAD_UNCHANGED)  # Use cv2.IMREAD_UNCHANGED to read alpha channel
                # Convert the image to a ROS Image message
                image_msg = bridge.cv2_to_imgmsg(image, encoding="bgr8")
                time_image =  float(png_path.split('_')[0])
                time_rgb_sec = int(time_image//1e6)
                time_rgb_nsec = int(time_image%1e6*1000)
                image_msg.header = Header(seq=idx_png, stamp=rospy.Time(time_rgb_sec, time_rgb_nsec), 
                                          frame_id="rgb")
                bag.write('/rgb_topic', image_msg, t=rospy.Time(time_rgb_sec, time_rgb_nsec)) 

        # 4. depth data 
        if args.depth==True:
            print('reading depth data ...')
            png_files = get_png_files(depth_path) 
            for idx_png, png_path in enumerate(png_files):
                 # Read the image using OpenCV
                file_path = depth_path+png_path
                image = cv2.imread(file_path, cv2.IMREAD_ANYDEPTH)  # Use cv2.IMREAD_UNCHANGED to read alpha channel
                # Convert the image to a ROS Image message
                image_msg = bridge.cv2_to_imgmsg(image, encoding="passthrough")
                time_image =  float(png_path.split('_')[0])
                time_rgb_sec = int(time_image//1e6)
                time_rgb_nsec = int(time_image%1e6*1000)
                sensor_type = png_path.split('_')[2].split('.')[0]
                if  sensor_type == 'event':
                    image_msg.header = Header(seq=idx_png, stamp=rospy.Time(time_rgb_sec, time_rgb_nsec), 
                                          frame_id="event_depth")
                    bag.write('/event_depth_topic', image_msg, t=rospy.Time(time_rgb_sec, time_rgb_nsec)) 
                elif sensor_type == 'rgb':
                    image_msg.header = Header(seq=idx_png, stamp=rospy.Time(time_rgb_sec, time_rgb_nsec), 
                                          frame_id="rgb_depth")
                    bag.write('/rgb_depth_topic', image_msg, t=rospy.Time(time_rgb_sec, time_rgb_nsec)) 
                
        # 5. event data
        print('reading event')
        # msg_Event = Event()
        # msg_Event.x = 1
        # msg_Event.y = 2
        # msg_Event.ts = rospy.Time(1000, 1000)
        # msg_Event.polarity = False
        # bag.write('/event_topic.msg', msg_Event, t=rospy.Time(1000, 1000))




if __name__ == "__main__":
    # Replace these with actual file paths and image folder path
    args  = getargs()


    # root_folder = './data/mocap_env1_comb/'
    root_folder = args.input_dir
    gt_path = root_folder+'gt_pose.txt'
    joint_path = root_folder+'mini_cheetah_joint.txt'
    imu_path = root_folder+'vectornav.txt'
    rgb_path = root_folder+'rgb/'
    depth_path = root_folder+'depth/'
    aedat_file = root_folder+'event.aedat4'

    output_bag_filename = root_folder+'output.bag'

    record_to_rosbag(gt_path=gt_path, joint_path=joint_path, imu_path=imu_path,
                    rgb_path=rgb_path, depth_path=depth_path, evnet_path=aedat_file,
                    output_bag_filename=output_bag_filename, 
                    args=args)