#!/bin/bash
source /home/suman/catkin_ws/devel/setup.bash
rosnode kill /my_bag
rosrun metavision_driver stop_recording.py
echo "Copying event file"
current_time=$(date "+%Y-%m-%d-%H-%M-%S")
mv /mnt/RIPHD4TB/data/tub-rip-recordings/trial02/events.bag /mnt/RIPHD4TB/data/tub-rip-recordings/trial02/events_$current_time.bag
