#!/usr/bin/python

import rospy
import cv2
import time
import sys
import numpy
from copy import copy
from math import tan, pi
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
import std_msgs.msg as std


image_size = int(sys.argv[1])

#depth_gain = 0.667

depth_range = [1, 10000]

period_ms = image_size**2/1100

#disp_scale = 51.0 / image_size

mode = False


R = numpy.array([[ 0.9999756813050,  0.004391118884090,  0.005417240317910],
                 [-0.0043891328387,  0.999990284443000, -0.000378685304895],
                 [-0.0054188510403,  0.000354899209924,  0.999985337257000]])
T = numpy.array( [-0.0641773045063,  0.000311704527121, -4.76178320241e-06])

callback_bool = [False, False, False, False]

bridge = CvBridge()
msg_info = CameraInfo()
msg_header = std.Header()


def callback_img_left(data):
    global img_left_data, callback_bool
    img_left_data = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    callback_bool[0] = True

def callback_cimg_left(data):
    global img_left_data, callback_bool
    img_left_data = bridge.compressed_imgmsg_to_cv2(data, desired_encoding="passthrough")
    callback_bool[0] = True

def callback_img_right(data):
    global img_right_data, callback_bool
    img_right_data = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    callback_bool[1] = True

def callback_cimg_right(data):
    global img_right_data, callback_bool
    img_right_data = bridge.compressed_imgmsg_to_cv2(data, desired_encoding="passthrough")
    callback_bool[1] = True

def callback_info_left(data):
    global K_left, D_left, original_height, original_width, callback_bool, time_diff
    K_left = numpy.reshape(data.K,(3,3))
    D_left = numpy.array(data.D[0:4])
    original_height = data.height
    original_width = data.width
    callback_bool[2] = True
    time_diff = rospy.Time.now().to_sec() - data.header.stamp.to_sec()

def callback_info_right(data):
    global K_right, D_right, callback_bool
    K_right = numpy.reshape(data.K,(3,3))
    D_right = numpy.array(data.D[0:4])
    callback_bool[3] = True


def main_function():
    
    global mode, count, center_undistorted, disparity, depth, rgb, img_window
    
    rospy.init_node("t265_depth", anonymous=True)
    
    rospy.Subscriber("/left/image_raw",             Image,           callback_img_left)
    rospy.Subscriber("/right/image_raw",            Image,           callback_img_right)
    rospy.Subscriber("/left/image_raw/compressed",  CompressedImage, callback_cimg_left)
    rospy.Subscriber("/right/image_raw/compressed", CompressedImage, callback_cimg_right)
    rospy.Subscriber("/left/camera_info",           CameraInfo,      callback_info_left)
    rospy.Subscriber("/right/camera_info",          CameraInfo,      callback_info_right)
    
    pub_depth_info  = rospy.Publisher("/depth/camera_info", CameraInfo, queue_size=1)
    pub_depth_image = rospy.Publisher("/depth/image_rect",  Image,      queue_size=1)
    pub_rgb_info    = rospy.Publisher("/rgb/camera_info",   CameraInfo, queue_size=1)
    pub_rgb_image   = rospy.Publisher("/rgb/image_rect",    Image,      queue_size=1)
    
    sys.stdout.write("\nWaiting for camera publications... ")
    sys.stdout.flush()
    while not all(callback_bool) and not rospy.is_shutdown():
        time.sleep(0.1)
    sys.stdout.write("Ok!\n")
    sys.stdout.flush()
    
    window_size = 5
    min_disp = 0
    num_disp = 112 - min_disp
    max_disp = min_disp + num_disp
    
    stereo = cv2.StereoSGBM_create(minDisparity = min_disp,
                                   numDisparities = num_disp,
                                   blockSize = 16,
                                   P1 = 8*3*window_size**2,
                                   P2 = 32*3*window_size**2,
                                   disp12MaxDiff = 1,
                                   uniquenessRatio = 10,
                                   speckleWindowSize = 100,
                                   speckleRange = 32)
    
    stereo_fov_rad = pi/2
    stereo_height_px = image_size
    stereo_width_px = stereo_height_px + max_disp
    stereo_focal_px = stereo_height_px/2 / tan(stereo_fov_rad/2)
    stereo_size = (stereo_width_px, stereo_height_px)
    stereo_cx = (stereo_height_px - 1)/2 + max_disp
    stereo_cy = (stereo_height_px - 1)/2
    
    R_left = numpy.eye(3)
    R_right = R
    
    P_left = numpy.array([[stereo_focal_px,               0, stereo_cx, 0],
                          [              0, stereo_focal_px, stereo_cy, 0],
                          [              0,               0,         1, 0]])
    P_right = copy(P_left)
    P_right[0][3] = T[0]*stereo_focal_px
    
    m1type = cv2.CV_32FC1
    (lm1, lm2) = cv2.fisheye.initUndistortRectifyMap(K_left, D_left, R_left, P_left, stereo_size, m1type)
    (rm1, rm2) = cv2.fisheye.initUndistortRectifyMap(K_right, D_right, R_right, P_right, stereo_size, m1type)
    undistort_rectify = {"left" : (lm1, lm2), "right" : (rm1, rm2)}
    
    msg_info.height = image_size
    msg_info.width = image_size
    msg_info.distortion_model = "plumb_bob"
    
#    msg_info.K = [K_left[0,0]*0.99,  0, image_size/2.0,
#                  0, K_left[1,1]*0.99, image_size/2.0, 0, 0, 1]
#    msg_info.D = [0, 0, 0, 0, 0]
#    msg_info.R = [1, 0, 0, 0, 1, 0, 0, 0, 1]
#    msg_info.P = [K_left[0,0]*0.99,  0, image_size/2.0, 0,
#                  0, K_left[1,1]*0.99, image_size/2.0, 0, 0, 0, 1, 0]
    
#    msg_info.K = [K_left[0,0]*image_size/original_width,  0, image_size/2.0,
#                  0, K_left[1,1]*image_size/original_height, image_size/2.0, 0, 0, 1]
#    msg_info.D = [0, 0, 0, 0, 0]
#    msg_info.R = [1, 0, 0, 0, 1, 0, 0, 0, 1]
#    msg_info.P = [K_left[0,0]*image_size/original_width,  0, image_size/2.0, 0,
#                  0, K_left[1,1]*image_size/original_height, image_size/2.0, 0, 0, 0, 1, 0]
    
    msg_info.K = [stereo_focal_px, 0, image_size/2.0,
                  0, stereo_focal_px, image_size/2.0, 0, 0, 1]
    msg_info.D = [0, 0, 0, 0, 0]
    msg_info.R = [1, 0, 0, 0, 1, 0, 0, 0, 1]
    msg_info.P = [stereo_focal_px, 0, image_size/2.0, 0,
                  0, stereo_focal_px, image_size/2.0, 0, 0, 0, 1, 0]
    
    if mode:
        cv2.namedWindow("Depth Results", cv2.WINDOW_NORMAL)

    rate = rospy.Rate(1000.0/period_ms)
    count = 0
    
    while not rospy.is_shutdown():
        
        tnow = time.time()
        count += 1
        
        frame_data = {"left"  : copy(img_left_data),
                      "right" : copy(img_right_data),
                      "seq"   : copy(count),
                      "stamp" : rospy.Time.from_sec(rospy.Time.to_sec(rospy.Time.now()) - time_diff),
                      "frame" : "t265_depth_optical_frame"}
        
        center_undistorted = {"left" : cv2.remap(src = frame_data["left"],
                                       map1 = undistort_rectify["left"][0],
                                       map2 = undistort_rectify["left"][1],
                                       interpolation = cv2.INTER_LINEAR),
                              "right": cv2.remap(src = frame_data["right"],
                                       map1 = undistort_rectify["right"][0],
                                       map2 = undistort_rectify["right"][1],
                                       interpolation = cv2.INTER_LINEAR)}
        
        disparity = stereo.compute(center_undistorted["left"], center_undistorted["right"]) / 16.0 #* disp_scale
        disparity = disparity[:,max_disp:] # /(image_size/35.0)  #35.0 #6.0 #8.0 #11.0 #14.0 #17.0 
        
#        depth = numpy.uint16(depth_gain * K_left[0,0] * 1000.0*abs(T[0]) / (disparity+0.001))
        depth = numpy.uint16(stereo_focal_px * 1000.0*abs(T[0]) / (disparity+0.001))
        depth[depth<depth_range[0]] = 0.0
        depth[depth>depth_range[1]] = 0.0
        
        rgb = cv2.cvtColor(center_undistorted["left"][:,max_disp:], cv2.COLOR_GRAY2RGB)
        
        msg_depth = bridge.cv2_to_imgmsg(depth, encoding="passthrough")
        msg_rgb = bridge.cv2_to_imgmsg(rgb, encoding="rgb8")
        
        msg_header.seq = frame_data["seq"]
        msg_header.stamp = frame_data["stamp"]
        msg_header.frame_id = frame_data["frame"]
        
        msg_depth.header = msg_header
        msg_rgb.header = msg_header
        msg_info.header = msg_header
        
        pub_depth_image.publish(msg_depth)
        pub_depth_info.publish(msg_info)
        pub_rgb_image.publish(msg_rgb)
        pub_rgb_info.publish(msg_info)
        
        if mode:
            img_window = numpy.hstack((rgb[:,:,0], numpy.uint8(255.0*depth/numpy.max(depth))))
            cv2.imshow("Depth Results", img_window)
            cv2.waitKey(1)
            if cv2.getWindowProperty("Depth Results", cv2.WND_PROP_VISIBLE) < 1:
                mode = False
    
        sys.stdout.write("\rTime elapsed (ms): %5d /%5d" % (1000.0*(time.time()-tnow), period_ms))
        sys.stdout.flush()
        
#        time.sleep(0.02)
        rate.sleep()
    
    sys.stdout.write("\n\n")
    sys.stdout.flush()
    

if __name__ == "__main__":
    try:
        main_function()
    except rospy.ROSInterruptException:
        pass