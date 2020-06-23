#!/usr/bin/env python

import rospy
import rosbag
import numpy
#import cv2
import sys
import os
from math import pi
from copy import copy
from cv_bridge import CvBridge
#from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
#from geometry_msgs.msg import Pose
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler


mode = "rtabmap" # t265, rtabmap


t265_gain = 1.0 # 1.038

depth_gain = 1.5

depth_shift_time = 1.1

match_bags = False


input_file = ["/mnt/WD500/Espeleo/CBA/original/2020-02-21-17-07-46.bag"]
output_file = "/mnt/WD500/Espeleo/CBA/veloso_" + mode + ".bag"
#output_file = "/home/rafael/test_ws/src/rtabmap_tutorials/bags/1_rgbd_handheld.bag"


tf_chassis = (0.000, 0.0, 0.150, +0.0, +0.0, +0.0)
tf_xsens   = (0.000, 0.0, 0.085, +0.0, +0.0, +0.0)
tf_os1     = (0.000, 0.0, 0.190, +0.0, +0.0, +0.0)
tf_t265    = (0.070, 0.0, 0.160, +0.0, -5.0, +0.0)
tf_d435i   = (0.070, 0.0, 0.160, +0.0, +1.0, +0.0)

tf_chassis = tf_chassis[:3] + tuple(quaternion_from_euler(pi/180*tf_chassis[3], pi/180*tf_chassis[4], pi/180*tf_chassis[5]))
tf_xsens   = tf_xsens[:3]   + tuple(quaternion_from_euler(pi/180*tf_xsens[3],   pi/180*tf_xsens[4],   pi/180*tf_xsens[5]  ))
tf_os1     = tf_os1[:3]     + tuple(quaternion_from_euler(pi/180*tf_os1[3],     pi/180*tf_os1[4],     pi/180*tf_os1[5]    ))
tf_t265    = tf_t265[:3]    + tuple(quaternion_from_euler(pi/180*tf_t265[3],    pi/180*tf_t265[4],    pi/180*tf_t265[5]   ))
tf_d435i   = tf_d435i[:3]   + tuple(quaternion_from_euler(pi/180*tf_d435i[3],   pi/180*tf_d435i[4],   pi/180*tf_d435i[5]  ))

#tf_chassis = (0.000, 0.0, 0.150, 0.0, +0.0000000, 0.0, 1.0000000)
#tf_xsens   = (0.000, 0.0, 0.085, 0.0, +0.0000000, 0.0, 1.0000000)
#tf_os1     = (0.000, 0.0, 0.190, 0.0, +0.0000000, 0.0, 1.0000000)
#tf_t265    = (0.070, 0.0, 0.160, 0.0, -0.0419877, 0.0, 0.9991181) # -0.0499792, 0.0, 0.9987503) # -0.0419877, 0.0, 0.9991181)
#tf_d435i   = (0.070, 0.0, 0.160, 0.0, +0.0149994, 0.0, 0.9998875) # +0.0499792, 0.0, 0.9987503) # +0.0099998, 0.0, 0.9999500)

#tf_chassis = [0.0,  0.0, 0.15]
#tf_os1     = [0.0,  0.0, 0.19]
#tf_t265    = [0.07, 0.0, 0.16]
#tf_d435i   = [0.07, 0.0, 0.16]

#eu_chassis = [0.0,  0.0,  0.0]
#eu_os1     = [0.0,  0.0,  0.0]
#eu_t265    = [0.0, -1.2,  0.0]
#eu_d435i   = [0.0,  5.0,  0.0]
#
#tf_chassis = quaternion_from_euler(pi/180*(eu_chassis[0]), pi/180*(eu_chassis[1]), pi/180*(eu_chassis[2]))
#tf_os1     = quaternion_from_euler(pi/180*(eu_os1[0]),     pi/180*(eu_os1[1]),     pi/180*(eu_os1[2]))
#tf_t265    = quaternion_from_euler(pi/180*(eu_t265[0]),    pi/180*(eu_t265[1]),    pi/180*(eu_t265[2]))
#tf_d435i   = quaternion_from_euler(pi/180*(eu_d435i[0]),   pi/180*(eu_d435i[1]),   pi/180*(eu_d435i[2]))

tf_args = numpy.array([
                       [0                 , 0                 , 0                 , 0                 , 0                 , 0                 , 1                 , "world"                               , "base_init"                           , 200 ],
                       [tf_chassis[0]     , tf_chassis[1]     , tf_chassis[2]     , tf_chassis[3]     , tf_chassis[4]     , tf_chassis[5]     , tf_chassis[6]     , "base_init"                           , "chassis_init"                        , 200 ],
                       [0                 , 0                 , 0                 , 0                 , 0                 , 0                 , 1                 , "chassis_init"                        , "wheel_init"                          , 10  ],
                       [0                 , 0                 , 0                 , 0                 , 0                 , 0                 , 1                 , "chassis_init"                        , "wheel_ekf_init"                      , 10  ],
                       [tf_t265[0]        , tf_t265[1]        , tf_t265[2]        , tf_t265[3]        , tf_t265[4]        , tf_t265[5]        , tf_t265[6]        , "chassis_init"                        , "t265_init"                           , 200 ],
#                       [tf_os1[0]         , tf_os1[1]         , tf_os1[2]         , 0                 , 0                 , 0                 , 1                 , "chassis_init"                        , "os1_init"                            , 100 ],
#                       [tf_os1[0]         , tf_os1[1]         , tf_os1[2]         , 0                 , 0                 , 0                 , 1                 , "chassis_init"                        , "os1_ekf_init"                        , 200 ],
                       [tf_d435i[0]       , tf_d435i[1]       , tf_d435i[2]       , tf_d435i[3]       , tf_d435i[4]       , tf_d435i[5]       , tf_d435i[6]       , "chassis_init"                        , "rtabmap_init"                        , 30  ],
                       [0                 , 0                 , 0                 , 0                 , 0                 , 0                 , 1                 , "wheel_pose"                          , "wheel_chassis"                       , 10  ],
                       [0                 , 0                 , 0                 , 0                 , 0                 , 0                 , 1                 , "wheel_ekf_pose"                      , "wheel_ekf_chassis"                   , 10  ],
                       [-tf_t265[0]       , -tf_t265[1]       , -tf_t265[2]       , -tf_t265[3]       , -tf_t265[4]       , -tf_t265[5]       , tf_t265[6]        , "t265_pose"                           , "t265_chassis"                        , 200 ],
#                       [-tf_t265[0]       , -tf_t265[1]       , -tf_t265[2]       , -tf_t265[3]       , -tf_t265[4]       , -tf_t265[5]       , tf_t265[6]        , "t265_pose"                           , "rtabmap_pose"                        , 200 ],
#                       [-tf_os1[0]        , -tf_os1[1]        , -tf_os1[2]        , 0                 , 0                 , 0                 , 1                 , "os1_pose"                            , "os1_chassis"                         , 100 ],
#                       [-tf_os1[0]        , -tf_os1[1]        , -tf_os1[2]        , 0                 , 0                 , 0                 , 1                 , "os1_ekf_pose"                        , "os1_ekf_chassis"                     , 200 ],
#                       [0.15              , 0.01              , -0.085            , 0                 , 0                 , 0                 , 1                 , "decawave_pose"                       , "decawave_chassis"                    , 10  ],
#                       [-0.27             , 0                 , -0.025            , 0                 , 0                 , 0                 , 1                 , "arfront_pose"                        , "axis_front_chassis"                  , 30  ],
#                       [0.27              , 0                 , -0.025            , 0                 , 0                 , 0                 , 1                 , "arback_pose"                         , "axis_back_chassis"                   , 30  ],
                       [-tf_d435i[0]      , -tf_d435i[1]      , -tf_d435i[2]      , -tf_d435i[3]      , -tf_d435i[4]      , -tf_d435i[5]      , tf_d435i[6]       , "rtabmap_pose"                        , "rtabmap_chassis"                     , 30 ],
                       [0                 , 0                 , 0.085             , 0                 , 0                 , 0                 , 1                 , "wheel_ekf_chassis"                   , "robot_imu"                           , 100 ],
                       [tf_xsens[0]       , tf_xsens[1]       , tf_xsens[2]       , tf_xsens[3]       , tf_xsens[4]       , tf_xsens[5]       , tf_xsens[6]       , mode + "_chassis"                     , "rtabmap_imu"                         , 100 ],
                       [-tf_chassis[0]    , -tf_chassis[1]    , -tf_chassis[2]    , -tf_chassis[3]    , -tf_chassis[4]    , -tf_chassis[5]    , tf_chassis[6]     , mode + "_chassis"                     , "base_link"                           , 200 ],
                       [tf_chassis[0]     , tf_chassis[1]     , tf_chassis[2]     , tf_chassis[3]     , tf_chassis[4]     , tf_chassis[5]     , tf_chassis[6]     , "base_link"                           , "chassis"                             , 200 ],
#                       [0.215             , 0.175             , 0                 , 0                 , 0                 , 0                 , 1                 , "chassis"                             , "left_front_wheel"                    , 10  ],
#                       [0                 , 0.225             , 0                 , 0                 , 0                 , 0                 , 1                 , "chassis"                             , "left_middle_wheel"                   , 10  ],
#                       [-0.215            , 0.175             , 0                 , 0                 , 0                 , 0                 , 1                 , "chassis"                             , "left_back_wheel"                     , 10  ],
#                       [0.215             , -0.175            , 0                 , 0                 , 0                 , 0                 , 1                 , "chassis"                             , "right_front_wheel"                   , 10  ],
#                       [0                 , -0.225            , 0                 , 0                 , 0                 , 0                 , 1                 , "chassis"                             , "right_middle_wheel"                  , 10  ],
#                       [-0.215            , -0.175            , 0                 , 0                 , 0                 , 0                 , 1                 , "chassis"                             , "right_back_wheel"                    , 10  ],
#                       [0                 , 0                 , 0.085             , 0                 , 0                 , 0                 , 1                 , "chassis"                             , "imu_link"                            , 100 ],
#                       [tf_os1[0]         , tf_os1[1]         , tf_os1[2]         , 0                 , 0                 , 0                 , 1                 , "chassis"                             , "os1_link"                            , 100 ],
#                       [0.27              , 0                 , 0.025             , 0                 , 0                 , 0                 , 1                 , "chassis"                             , "axis_link"                           , 30  ],
#                       [0.27              , 0                 , 0.025             , 0                 , 0                 , 0                 , 1                 , "chassis"                             , "axis_front_link"                     , 30  ],
#                       [-0.27             , 0                 , 0.025             , 0                 , 0                 , 0                 , 1                 , "chassis"                             , "axis_back_link"                      , 30  ],
                       [tf_t265[0]        , tf_t265[1]        , tf_t265[2]        , tf_t265[3]        , tf_t265[4]        , tf_t265[5]        , tf_t265[6]        , "chassis"                             , "t265_link"                           , 200 ],
                       [tf_d435i[0]       , tf_d435i[1]       , tf_d435i[2]       , tf_d435i[3]       , tf_d435i[4]       , tf_d435i[5]       , tf_d435i[6]       , "chassis"                             , "d435i_link"                          , 200 ],
#                       [-0.15             , -0.01             , 0.085             , 0                 , 0                 , 0                 , 1                 , "chassis"                             , "decawave_link"                       , 10  ],
#                       [0.2               , 0.0325            , 0.125             , 0                 , 0                 , 0                 , 1                 , "chassis"                             , "kinect_link"                         , 60  ],
#                       [0                 , 0                 , 0                 , 0                 , 0                 , 0                 , 1                 , "imu_link"                            , "imu"                                 , 100 ],
#                       [0                 , 0                 , 0.03618           , 0                 , 0                 , 1                 , 0                 , "os1_link"                            , "os1_lidar"                           , 10  ],
#                       [0.006253          , -0.011775         , 0.007645          , 0                 , 0                 , 0                 , 1                 , "os1_link"                            , "os1_imu"                             , 100 ],
#                       [0                 , 0                 , 0                 , -0.5              , 0.5               , -0.5              , 0.5               , "axis_link"                           , "axis_optical_frame"                  , 30  ],
#                       [0                 , 0                 , 0                 , -0.5              , 0.5               , -0.5              , 0.5               , "axis_front_link"                     , "axis_front_optical_frame"            , 30  ],
#                       [0                 , 0                 , 0                 , -0.5              , -0.5              , 0.5               , 0.5               , "axis_back_link"                      , "axis_back_optical_frame"             , 30  ],
#                       [3.80186684197e-05 , 0.0320889987051   , 2.4195331207e-05  , 0.00326163647696  , -0.999985337257   , -0.00424767192453 , 0.000792266393546 , "t265_link"                           , "t265_fisheye1_frame"                 , 30  ],
#                       [0                 , 0                 , 0                 ,  0.5              , -0.5              , -0.5              , 0.5               , "t265_fisheye1_frame"                 , "t265_fisheye1_optical_frame"         , 30  ],
#                       [-3.80186684197e-05, -0.0320890024304  , -2.41953239311e-05, 0.000553577148821 , -0.99999755621    , -0.00205105380155 , 0.000613214506302 , "t265_link"                           , "t265_fisheye2_frame"                 , 30  ],
#                       [0                 , 0                 , 0                 , 0.5               , -0.5              , -0.5              , 0.5               , "t265_fisheye2_frame"                 , "t265_fisheye2_optical_frame"         , 30  ],
#                       [-3.17073136102e-05, 0.0213896092027   , 0.000115149479825 , 0                 , 0                 , 1                 , 0                 , "t265_link"                           , "t265_gyro_frame"                     , 200 ],
#                       [0                 , 0                 , 0                 , 0.5               , -0.5              , -0.5              , 0.5               , "t265_gyro_frame"                     , "t265_gyro_optical_frame"             , 200 ],
#                       [-3.17073136102e-05, 0.0213896092027   , 0.000115149479825 , 0                 , 0                 , 1                 , 0                 , "t265_link"                           , "t265_accel_frame"                    , 62  ],
#                       [0                 , 0                 , 0                 , 0.5               , -0.5              , -0.5              , 0.5               , "t265_accel_frame"                    , "t265_accel_optical_frame"            , 62  ],
#                       [0                 , 0                 , 0                 , 0                 , 0                 , 0                 , 1                 , "t265_link"                           , "t265_depth_frame"                    , 30  ],
#                       [0                 , 0                 , 0                 , -0.5              , 0.5               , -0.5              , 0.5               , "t265_depth_frame"                    , "t265_depth_optical_frame"            , 30  ],
#                       [0                 , 0                 , 0                 , -0.5              , 0.5               , -0.5              , 0.5               , "t265_pose_frame"                     , "t265_depth_optical_frame"            , 200 ],
#                       [0                 , 0                 , 0                 , 0                 , 0                 , 0                 , 1                 , "t265_pose_frame"                     , "t265_link"                           , 200 ],
#                       [0                 , 0                 , 0                 , 0                 , 0                 , 0                 , 1                 , "t265_pose_frame"                     , "d435i_link"                          , 100 ],
                       [0                 , 0                 , 0                 , 0                 , 0                 , 0                 , 1                 , "d435i_link"                          , "d435i_depth_frame"                   , 100 ],
                       [0                 , 0                 , 0                 , -0.5              , 0.5               , -0.5              , 0.5               , "d435i_depth_frame"                   , "d435i_depth_optical_frame"           , 100 ],
                       [-0.000214694606257, 0.0150816757232   , 7.47397498344e-05 , 0.0015756865032   , -0.000297236372717, 0.0036272148136   , 0.999992132187    , "d435i_link"                          , "d435i_color_frame"                   , 60  ],
                       [-0.000214694606257, 0.0150816757232   , 7.47397498344e-05 , 0.0015756865032   , -0.000297236372717, 0.0036272148136   , 0.999992132187    , "d435i_link"                          , "d435i_aligned_depth_to_color_frame"  , 60  ],
                       [0                 , 0                 , 0                 , -0.5              , 0.5               , -0.5              , 0.5               , "d435i_aligned_depth_to_color_frame"  , "d435i_color_optical_frame"           , 60  ],
#                       [0                 , 0                 , 0                 , 0                 , 0                 , 0                 , 1                 , "d435i_link"                          , "d435i_infra1_frame"                  , 100 ],
#                       [0                 , 0                 , 0                 , 0                 , 0                 , 0                 , 1                 , "d435i_link"                          , "d435i_aligned_depth_to_infra1_frame" , 100 ],
#                       [0                 , 0                 , 0                 , -0.5              , 0.5               , -0.5              , 0.5               , "d435i_aligned_depth_to_infra1_frame" , "d435i_infra1_optical_frame"          , 100 ],
#                       [0                 , -0.0499469712377  , 0                 , 0                 , 0                 , 0                 , 1                 , "d435i_link"                          , "d435i_infra2_frame"                  , 100 ],
#                       [0                 , 0                 , 0                 , -0.5              , 0.5               , -0.5              , 0.5               , "d435i_infra2_frame"                  , "d435i_infra2_optical_frame"          , 100 ],
                       [-0.012            , -0.006            , 0.005             , 0                 , 0                 , 0                 , 1                 , "d435i_link"                          , "d435i_accel_frame"                   , 250 ],
                       [0                 , 0                 , 0                 , -0.5              , 0.5               , -0.5              , 0.5               , "d435i_accel_frame"                   , "d435i_accel_optical_frame"           , 250 ],
                       [-0.012            , -0.006            , 0.005             , 0                 , 0                 , 0                 , 1                 , "d435i_link"                          , "d435i_gyro_frame"                    , 400 ],
                       [0                 , 0                 , 0                 , -0.5              , 0.5               , -0.5              , 0.5               , "d435i_gyro_frame"                    , "d435i_gyro_optical_frame"            , 400 ],
                       ], dtype=object)



def qq_mult(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    return x, y, z, w

def q_conjugate(q):
    x, y, z, w = q
    return -x, -y, -z, w

def qv_mult(q1, v1):
    q2 = v1 + (0.0,)
    return qq_mult(qq_mult(q1, q2), q_conjugate(q1))[:-1]


def main_function():
    
    #global img_last, img_curr, img_depth, img_color, t_last, t_curr, t_color, t_norm, odom_msg
    
    rospy.init_node("bagmerge_node", anonymous=True)
    rospy.sleep(0.1)
    
    for k in range(len(input_file)):
        sys.stdout.write("\n%s" % input_file[k])
        sys.stdout.flush()
        rospy.sleep(0.01)
        
    if raw_input("\nAre you sure these are the correct files? (y/n): ") == "y":
        sys.stdout.write("\n%s\n" % output_file)
        sys.stdout.flush()
        if os.path.exists(output_file):
            if raw_input("Output file already exists, do you want to replace it? (y/n): ") == "y":
                os.remove(output_file)
            else:
                sys.exit(0)
    else:
        sys.exit(0)
        
    bag_in = numpy.empty(len(input_file), dtype=object)
    bag_time = numpy.zeros((2,len(input_file)))
    
    for k in range(len(input_file)):
        
        sys.stdout.write("\nOpening bag %d... " % (k+1))
        sys.stdout.flush()
        bag_in[k] = rosbag.Bag(input_file[k])
        sys.stdout.write("Ok!")
        sys.stdout.flush()
        
        bag_time[1,k] = bag_in[k].get_end_time() - bag_in[k].get_start_time()
        for topic, msg, t in bag_in[k].read_messages():
            bag_time[0,k] = rospy.Time.to_sec(t)
            break
            
    if match_bags:
        bag_start_time = min(bag_time[0])
        bag_end_time = min(bag_time[0]) + max(bag_time[1])
        bag_shift_time = bag_time[0] - min(bag_time[0])
    else:
        bag_start_time = min(bag_time[0])
        bag_end_time = max(bag_time[0] + bag_time[1])
        bag_shift_time = numpy.zeros(len(input_file))
        
    sys.stdout.write("\n\nBag starting: %10.9f\nBag ending:   %10.9f\nBag duration: %3.1fs\n" % (bag_start_time, bag_end_time, (bag_end_time-bag_start_time)))
    for k in range(len(input_file)):
        sys.stdout.write("Bag %d shift:  %3.1f\n" % ((k+1), bag_shift_time[k]))
    sys.stdout.flush()
    sys.stdout.write("\n")
    
    bag_out = rosbag.Bag(output_file, "w")
    
    bridge = CvBridge()
    img_curr = numpy.zeros((480,640),dtype="uint16")
    
    seq_color = 0
    seq_depth = 0
    t_curr = 0.0
    t_color = []
    img_color = numpy.zeros((0,480,640,3),dtype="uint8")
    pub_bool = [False, False, False, False]
    
    k = 0
    
    while k < len(input_file):
        
        if "2020-02-21-17-07-46" in input_file[k]:
            tf_t265_init  = (0.37515220046, -0.0101201804355, 0.00968149211258, -0.0046105417423, -0.0330555289984, -0.0177801921964, 0.999284684658)
            tf_wheel_init = (0.389882879089, -0.00783591647163, 0.0, 0.0, 0.0, -0.00117437191261, 0.999999310425)
        else:
            tf_t265_init  = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
            tf_wheel_init = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)

        
        for topic, msg, t in bag_in[k].read_messages():
            
            topic_bool = False
            
            if rospy.is_shutdown():
                k = len(input_file)
                break
                
            if match_bags and hasattr(msg, "header"):
                t = rospy.Time.from_sec(rospy.Time.to_sec(t) - bag_shift_time[k])
                msg.header.stamp = copy(t)
                
#            if (topic[:7]+topic[8:]) == "/device/get_joint_state":
#                topic_bool = True
#                bag_out.write(topic, msg, t)
#                
#            if topic == "/cmd_vel":
#                topic_bool = True
#                bag_out.write(topic, msg, t)
                
            elif topic == "/d435i/gyro/sample":
                topic_bool = True
                bag_out.write(topic, msg, t)
                ang_vel = copy(msg.angular_velocity)
                
            elif topic == "/d435i/accel/sample":
                topic_bool = True
                bag_out.write(topic, msg, t)
                lin_acc = copy(msg.linear_acceleration)
                
                if "ang_vel" in locals():
                    imu_data = copy(msg)
                    imu_data.header.frame_id = 'rtabmap_imu'
                    imu_data.linear_acceleration.x = +copy(lin_acc.z)
                    imu_data.linear_acceleration.y = -copy(lin_acc.x)
                    imu_data.linear_acceleration.z = -copy(lin_acc.y)
                    imu_data.angular_velocity.x = +copy(ang_vel.z)
                    imu_data.angular_velocity.y = -copy(ang_vel.x)
                    imu_data.angular_velocity.z = -copy(ang_vel.y)
                    bag_out.write("/rtabmap/imu_raw", imu_data, t)
                    msg_wheel = copy(imu_data)
                    msg_wheel.header.frame_id = "robot_imu"
                    bag_out.write("/robot_imu_raw", msg_wheel, t)
                
            elif topic == "/d435i/color/camera_info":
                topic_bool = True
                msg_color_info = copy(msg)
                pub_bool[0] = True
                
            elif topic == "/d435i/color/image_raw":
                
                topic_bool = True
                
                if "msg_color_info" in locals():
                    bag_out.write("/d435i/color/camera_info", msg_color_info, msg_color_info.header.stamp)
                    bag_out.write("/d435i/color/image_raw", msg, msg.header.stamp)
                
                msg_color_raw = copy(msg)
                pub_bool[1] = True
                t_color.append(rospy.Time.to_sec(t))
                
                img_color_raw = bridge.imgmsg_to_cv2(msg_color_raw, desired_encoding="passthrough")
                img_color_raw = numpy.reshape(img_color_raw,(1,480,640,3))
                img_color = numpy.concatenate((img_color, img_color_raw), axis=0)
                
                if all(pub_bool):
                    
                    seq_color += 1
                    msg_color_info.header.seq = copy(seq_color)
                    msg_color_raw.header.seq = copy(seq_color)
                    msg_color_info.header.stamp = copy(t)
                    msg_color_raw.header.stamp = copy(t)
                    msg_color_raw.header.frame_id = "d435i_depth_optical_frame"
                    msg_color_info.header.frame_id = "d435i_depth_optical_frame"
                    
                    bag_out.write("/rgb/camera_info", msg_color_info, msg_color_info.header.stamp)
                    bag_out.write("/rgb/image_rect", msg_color_raw, msg_color_raw.header.stamp)
                
            elif topic == "/d435i/depth/camera_info":
                
                topic_bool = True
                msg_depth_info = copy(msg)
                pub_bool[2] = True
                
            elif topic == "/d435i/depth/image_rect_raw":
                
                topic_bool = True
                
                if "msg_depth_info" in locals():
                    bag_out.write("/d435i/depth/camera_info", msg_depth_info, msg_depth_info.header.stamp)
                    bag_out.write("/d435i/depth/image_rect_raw", msg, msg.header.stamp)
                
                img_last = copy(img_curr)
                img_curr = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                    
                t_last = copy(t_curr)
                t_curr = rospy.Time.to_sec(t)
                
                if all(pub_bool):
                    
                    t_norm = (numpy.array(t_color)-t_last)/(t_curr-t_last)
                    
                    img_depth = numpy.zeros((len(t_color),480,640),dtype="uint16")
                    
#                    k = len(input_file)
#                    break
                    
                    for n in range(len(t_color)):
                        
                        img_depth[n] = (1-t_norm[n]) * img_last + t_norm[n] * img_curr
                        img_depth[n] = depth_gain * img_depth[n]
                        img_depth[n][(img_last==0) | (img_curr==0) | (img_depth[n]>10000) | (img_depth[n]<0)] = 0
#                        img_crop = img_depth[n,90:380,120:507]
#                        img_align = cv2.resize(img_crop,(640,480), interpolation = cv2.INTER_NEAREST)
                        img_align = img_depth[n]
                        
                        seq_depth += 1
                        
                        msg_align_raw = bridge.cv2_to_imgmsg(img_align, encoding="passthrough")
                        msg_align_raw.header.seq = copy(seq_depth)
                        msg_align_raw.header.stamp = rospy.Time.from_sec(t_color[n]-depth_shift_time)
                        msg_align_raw.header.frame_id = "d435i_depth_optical_frame"
                        
                        msg_align_info = msg_depth_info
                        msg_align_info.header.seq = copy(seq_depth)
                        msg_align_info.header.stamp = rospy.Time.from_sec(t_color[n]-depth_shift_time)
                        msg_align_info.header.frame_id = "d435i_depth_optical_frame"
                        
                        bag_out.write("/depth/camera_info", msg_align_info, msg_align_info.header.stamp)
                        bag_out.write("/depth/image_rect", msg_align_raw, msg_align_raw.header.stamp)
                
                pub_bool[3] = True
                t_color = []
                img_color = numpy.zeros((0,480,640,3),dtype="uint8")
                
            elif topic == "/odom":
                
                topic_bool = True
                topic = "/robot_odom"
                msg.header.frame_id = "wheel_init"
                msg.child_frame_id = "wheel_pose"
                
                p_old = (-copy(msg.pose.pose.position.x), -copy(msg.pose.pose.position.y), copy(msg.pose.pose.position.z))
                q_old = (copy(msg.pose.pose.orientation.x), copy(msg.pose.pose.orientation.y), copy(msg.pose.pose.orientation.z), copy(msg.pose.pose.orientation.w))
                v_old = (copy(msg.twist.twist.linear.x), copy(msg.twist.twist.linear.y), copy(msg.twist.twist.linear.z))
                w_old = (copy(msg.twist.twist.angular.x), copy(msg.twist.twist.angular.y), copy(msg.twist.twist.angular.z))
                p_new = qv_mult(q_conjugate(tf_wheel_init[3:]), (p_old[0]-tf_wheel_init[0],p_old[1]-tf_wheel_init[1],p_old[2]-tf_wheel_init[2]))
                q_new = qq_mult(q_conjugate(tf_wheel_init[3:]), copy(q_old))
                v_new = qv_mult(q_conjugate(tf_wheel_init[3:]), copy(v_old))
                w_new = qv_mult(q_conjugate(tf_wheel_init[3:]), copy(w_old))
                
                msg.pose.pose.position.x = p_new[0]
                msg.pose.pose.position.y = p_new[1]
                msg.pose.pose.position.z = p_new[2]
                msg.pose.pose.orientation.x = q_new[0]
                msg.pose.pose.orientation.y = q_new[1]
                msg.pose.pose.orientation.z = q_new[2]
                msg.pose.pose.orientation.w = q_new[3]
                msg.twist.twist.linear.x = v_new[0]
                msg.twist.twist.linear.y = v_new[1]
                msg.twist.twist.linear.z = v_new[2]
                msg.twist.twist.angular.x = w_new[0]
                msg.twist.twist.angular.y = w_new[1]
                msg.twist.twist.angular.z = w_new[2]
                
                bag_out.write(topic, msg, t)
                
                tf_msg = TransformStamped()
                tf_msg.header.stamp = copy(t)
                tf_msg.header.frame_id = copy(msg.header.frame_id)
                tf_msg.child_frame_id = copy(msg.child_frame_id)
                tf_msg.transform.translation = copy(msg.pose.pose.position)
                tf_msg.transform.rotation = copy(msg.pose.pose.orientation)
                bag_out.write("/tf", TFMessage([tf_msg]), t)
                
#            elif topic == "/os1_cloud_node/imu":
#                topic_bool = True
#                bag_out.write(topic, msg, t)
                
#            elif topic == "/os1_cloud_node/points":
#                topic_bool = True
#                bag_out.write(topic, msg, t)
                
            elif topic == "/t265/odom/sample":
                
                topic_bool = True
                msg.header.frame_id = "t265_init"
                msg.child_frame_id = "t265_pose"
                
                p_old = (copy(msg.pose.pose.position.x), copy(msg.pose.pose.position.y), copy(msg.pose.pose.position.z))
                q_old = (copy(msg.pose.pose.orientation.x), copy(msg.pose.pose.orientation.y), copy(msg.pose.pose.orientation.z), copy(msg.pose.pose.orientation.w))
                v_old = (copy(msg.twist.twist.linear.x), copy(msg.twist.twist.linear.y), copy(msg.twist.twist.linear.z))
                w_old = (copy(msg.twist.twist.angular.x), copy(msg.twist.twist.angular.y), copy(msg.twist.twist.angular.z))
                p_new = qv_mult(q_conjugate(tf_t265_init[3:]), (p_old[0]-tf_t265_init[0],p_old[1]-tf_t265_init[1],p_old[2]-tf_t265_init[2]))
                q_new = qq_mult(q_conjugate(tf_t265_init[3:]), copy(q_old))
                v_new = qv_mult(q_conjugate(tf_t265_init[3:]), copy(v_old))
                w_new = qv_mult(q_conjugate(tf_t265_init[3:]), copy(w_old))
                
                msg.pose.pose.position.x = t265_gain * p_new[0]
                msg.pose.pose.position.y = t265_gain * p_new[1]
                msg.pose.pose.position.z = t265_gain * p_new[2]
                msg.pose.pose.orientation.x = q_new[0]
                msg.pose.pose.orientation.y = q_new[1]
                msg.pose.pose.orientation.z = q_new[2]
                msg.pose.pose.orientation.w = q_new[3]
                msg.twist.twist.linear.x = t265_gain * v_new[0]
                msg.twist.twist.linear.y = t265_gain * v_new[1]
                msg.twist.twist.linear.z = t265_gain * v_new[2]
                msg.twist.twist.angular.x = w_new[0]
                msg.twist.twist.angular.y = w_new[1]
                msg.twist.twist.angular.z = w_new[2]
                
                bag_out.write(topic, msg, t)
                
                tf_msg = TransformStamped()
                tf_msg.header.stamp = copy(t)
                tf_msg.header.frame_id = copy(msg.header.frame_id)
                tf_msg.child_frame_id = copy(msg.child_frame_id)
                tf_msg.transform.translation = copy(msg.pose.pose.position)
                tf_msg.transform.rotation = copy(msg.pose.pose.orientation)
                bag_out.write("/tf", TFMessage([tf_msg]), t)
                
                clk_msg = Clock()
                clk_msg.clock = copy(t)
                bag_out.write("/clock", clk_msg, t)
                
#                pos_init = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
#                ori_init = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
#                lin_init = (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z)
#                ang_init = (msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z)
#                pos_world = qv_mult(tuple(tf_t265), pos_init)
#                pos_world = (pos_world[0] + tf_t265[0] + tf_chassis[0], pos_world[1] + tf_t265[1] + tf_chassis[1], pos_world[2] + tf_t265[2] + tf_chassis[2])
#                ori_world = qq_mult(tuple(tf_t265), ori_init)
#                lin_world = qv_mult(tuple(tf_t265), lin_init)
#                ang_world = qv_mult(tuple(tf_t265), ang_init)
#                
#                odom_msg = copy(msg)
#                odom_msg.header.frame_id = "world"
#                odom_msg.pose.pose.position.x = pos_world[0]
#                odom_msg.pose.pose.position.y = pos_world[1]
#                odom_msg.pose.pose.position.z = pos_world[2]
#                odom_msg.pose.pose.orientation.x = ori_world[0]
#                odom_msg.pose.pose.orientation.y = ori_world[1]
#                odom_msg.pose.pose.orientation.z = ori_world[2]
#                odom_msg.pose.pose.orientation.w = ori_world[3]
#                odom_msg.twist.twist.linear.x = lin_world[0]
#                odom_msg.twist.twist.linear.y = lin_world[1]
#                odom_msg.twist.twist.linear.z = lin_world[2]
#                odom_msg.twist.twist.angular.x = ang_world[0]
#                odom_msg.twist.twist.angular.y = ang_world[1]
#                odom_msg.twist.twist.angular.z = ang_world[2]
#                bag_out.write("/t265/odom/world", odom_msg, t)
                
            if topic_bool:
                status_time = 100.0 * (rospy.Time.to_sec(t) - bag_start_time) / (bag_end_time - bag_start_time)
                sys.stdout.write("\rBag  %d/%d - Publishing %-28s %5.1f%%" % ((k+1), len(input_file), topic, status_time))
                sys.stdout.flush()
            
        sys.stdout.write("\n")
        sys.stdout.flush()
        
        k += 1
        
    sys.stdout.write("\n")
    sys.stdout.flush()
    
    for k in range(len(tf_args)):
#        break
        t = bag_start_time
        
        while t < bag_end_time and not rospy.is_shutdown():
            
            tf_msg = TransformStamped()
            tf_msg.header.stamp = rospy.Time.from_sec(t)
            tf_msg.header.frame_id = tf_args[k,7]
            tf_msg.child_frame_id = tf_args[k,8]
            tf_msg.transform.translation.x = tf_args[k,0]
            tf_msg.transform.translation.y = tf_args[k,1]
            tf_msg.transform.translation.z = tf_args[k,2]
            tf_msg.transform.rotation.x = tf_args[k,3]
            tf_msg.transform.rotation.y = tf_args[k,4]
            tf_msg.transform.rotation.z = tf_args[k,5]
            tf_msg.transform.rotation.w = tf_args[k,6]
            
            bag_out.write("/tf", TFMessage([tf_msg]), tf_msg.header.stamp)
            
            t += 1.0/tf_args[k,9]
            
            status_time = 100.0 * (t - bag_start_time) / (bag_end_time - bag_start_time)
            sys.stdout.write("\rTF %2d/%2d - Publishing %-28s %5.1f%%" % ((k+1),len(tf_args),tf_args[k,8],status_time))
            sys.stdout.flush()
            
        sys.stdout.write("\n")
        sys.stdout.flush()
        
    for k in range(len(input_file)):
        bag_in[k].close()
    bag_out.close()
    
    sys.stdout.write("\nFiles Closed\n\n")
    sys.stdout.flush()
    
    
if __name__ == "__main__":
    try:
        main_function()
    except rospy.ROSInterruptException:
        pass