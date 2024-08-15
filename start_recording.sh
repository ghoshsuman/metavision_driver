#!/bin/bash
source /home/suman/catkin_ws/devel/setup.bash
rosrun metavision_driver start_recording.py
rosbag record -o /mnt/RIPHD4TB/data/tub-rip-recordings/trial02/frames.bag /natnet_ros/evStereoRig/pose /camera/color/image_raw /camera/aligned_depth_to_color/image_raw __name:=my_bag
