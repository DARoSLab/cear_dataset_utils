supported version
```
python2.7
ubuntu18.04
ros-melodic
```

# 0. workspace
```
source ./devel/setup.bash
```

# 1. recording data
```
python ./main.py --input_dir ../data/mocap_env1_comb/ --gt_pose --imu --joint --rgb
```


## 1.1 record event data
1. get event.txt
```
python3 ./read_event_data.py --input_dir ../data/mocap_env1_comb/ 
```

# 2. verify rosbag

```
rosbag play output.bag
```

```
rosbag info output.bag
```

```
rosbag echo <topic name>
```

## 2.1 check rgb, raw depth
```
python ./check_event_depth.py --input_dir ../data/mocap_env1_comb/
```


```
python ./check_rgb_depth.py --input_dir ../data/mocap_env1_comb/
```