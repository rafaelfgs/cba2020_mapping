#!/usr/bin/env python

import rospy
import sys
import time
from copy import copy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image

if sys.argv[1] == "veloso":
    depth_range = [1000, 5000]
    period_ms = 5000
else:
    depth_range = [500, 3500]
    period_ms = 1000

bridge = CvBridge()
msg_bool = [False, False, False, False]



def callback_rgb_img(data):
    global rgb_img, msg_bool
    rgb_img = copy(data)
    msg_bool[0] = True

def callback_rgb_info(data):
    global rgb_info, msg_bool
    rgb_info = copy(data)
    msg_bool[1] = True

def callback_depth_img(data):
    global depth_data, depth_img, msg_bool
    depth_data = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    depth_data.setflags(write=1)
    depth_data[(depth_data < depth_range[0]) | (depth_data > depth_range[1])] = 0
    depth_img = bridge.cv2_to_imgmsg(depth_data, encoding="passthrough")
    depth_img = Image()
    depth_img.header = copy(data.header)
    depth_img = copy(data)
    msg_bool[2] = True

def callback_depth_info(data):
    global depth_info, msg_bool
    depth_info = copy(data)
    msg_bool[3] = True



def main_function():
    
    rospy.init_node("sync_node", anonymous=True)

    rospy.Subscriber("/rgb/image_rect",    Image,      callback_rgb_img)
    rospy.Subscriber("/rgb/camera_info",   CameraInfo, callback_rgb_info)
    rospy.Subscriber("/depth/image_rect",  Image,      callback_depth_img)
    rospy.Subscriber("/depth/camera_info", CameraInfo, callback_depth_info)

    pub_rgb_img    = rospy.Publisher("/cloud/rgb/image_rect",    Image,      queue_size=1)
    pub_rgb_info   = rospy.Publisher("/cloud/rgb/camera_info",   CameraInfo, queue_size=1)
    pub_depth_img  = rospy.Publisher("/cloud/depth/image_rect",  Image,      queue_size=1)
    pub_depth_info = rospy.Publisher("/cloud/depth/camera_info", CameraInfo, queue_size=1)

    while not all(msg_bool) and not rospy.is_shutdown():
        sys.stdout.write("\rWaiting for Publications... ")
        sys.stdout.flush()
        time.sleep(0.01)
    sys.stdout.write("Ok!\n")
    sys.stdout.flush()
    
    rate = rospy.Rate(1000.0/period_ms)
    
    while not rospy.is_shutdown():

        pub_rgb_img.publish(rgb_img)
        pub_rgb_info.publish(rgb_info)
        pub_depth_img.publish(depth_img)
        pub_depth_info.publish(depth_info)

        rate.sleep()

if __name__ == "__main__":
    try:
        main_function()
    except rospy.ROSInterruptException:
        pass
