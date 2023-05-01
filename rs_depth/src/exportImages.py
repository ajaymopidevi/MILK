#!/usr/bin/env python3
import sys
import os
import rosbag
from os.path import join
import numpy as np
import cv2
import csv
import yaml
from tqdm import tqdm
import string
import sensor_msgs.point_cloud2
import shutil
import scipy.io as sio
import inspect


ROS_SAVE_DIR = "/home/ajay/catkin_ws_5302/test"

ROSBAG_PATH =  "/home/ajay/catkin_ws_5302/race_go_straight_left.bag"

print("Reading Rosbag")
bag = rosbag.Bag(ROSBAG_PATH, 'r')
counter=0
#########################################
# process topics based on images
#########################################
print("Reading Image topics")
# for topic, msg, t in tqdm(rosbag.Bag(ROSBAG_PATH, 'r').read_messages(topics=img_topics)):
for topic, msg, t in tqdm(bag.read_messages(topics=['/camera/depth/image_rect_raw'])):
    if topic == '/camera/depth/image_rect_raw':
        counter += 1
        # image_name = 'depth_'+str(msg.header.stamp.secs)+ '.' + "{0:09d}".format(msg.header.stamp.nsecs) + ".png"
        image_name = str(t) + ".png"
        np_arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width, -1)  # 16bit depth image
        cv2.imwrite(os.path.join(ROS_SAVE_DIR, image_name), np_arr)

bag.close()