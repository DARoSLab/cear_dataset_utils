supported version
```
python2.7
ubuntu18.04
ros-melodic
```

# 0. Install
```
git clone https://github.com/DARoSLab/cear_dataset_utils.git
cd cear_dataset_utils
catkin_make
source ./devel/setup.bash
```

# 1. Usage
```
python ./main.py --input_dir ../data/mocap_env1_comb/ --gt_pose --imu --joint --rgb --depth
```

E.g. If you want to get RGB and IMU data in the ROS bag:
```
python ./main.py --input_dir ../data/mocap_env1_comb/ --imu --rgb

```

### 1.1 Convert event aedat4 to rosbag
Please refer to this [repository](https://gitlab.com/inivation/dv/dv-ros/-/tree/master/dv_ros_aedat4?ref_type=heads) for converting aedat4 event file to ROS bag format.

### 1.2 Merge ROS bags
Use [rosbag-merge](https://pypi.org/project/rosbag-merge/) tool to merge ROS bags into one ROS bag.
