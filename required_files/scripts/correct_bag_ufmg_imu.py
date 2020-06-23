#!/usr/bin/env python

import rospy
import rosbag
import numpy
#import cv2
import sys
import os
#from math import pi
from copy import copy
#from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
#from geometry_msgs.msg import Pose
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
#from tf.transformations import quaternion_from_euler



bag  = "corredor" # "corredor", "volta"
mode = "rtabmap"  # "rtabmap" , "t265"
        
match_bags = False

input_file = ["/mnt/WD500/Espeleo/CBA/original/200212_161243_corredor_lab_reta_ziguezague.bag"]
output_file = "/mnt/WD500/Espeleo/CBA/200212_161243_corredor_" + mode + ".bag"

if bag == "volta": output_file = output_file[:46] + "volta_" + output_file[46:]

if bag == "volta": tf_world = (22.85, 0.0, 0.0, 0.0, 0.0, 0.9818535, 0.1896408)
else: tf_world = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)


tf_chassis = (0.000, 0.0, 0.150, 0.0, +0.0000000, 0.0, 1.0000000)
tf_xsens   = (0.000, 0.0, 0.085, 0.0, +0.0000000, 0.0, 1.0000000)
tf_os1     = (0.000, 0.0, 0.190, 0.0, +0.0000000, 0.0, 1.0000000)
tf_t265    = (0.270, 0.0, 0.070, 0.0, +0.0000000, 0.0, 1.0000000)
tf_d435i   = (0.270, 0.0, 0.070, 0.0, +0.0000000, 0.0, 1.0000000)



#tf_chassis = [0.0,  0.0, 0.15]
#tf_os1     = [0.0,  0.0, 0.19]
#tf_t265    = [0.27, 0.0, 0.07]
#tf_d435i   = [0.27, 0.0, 0.07]
#
#eu_chassis = [0.0,  0.0,  0.0]
#eu_os1     = [0.0,  0.0,  0.0]
#eu_t265    = [0.0,  0.0,  0.0]
#eu_d435i   = [0.0,  0.0,  0.0]
#
#qt_chassis = quaternion_from_euler(pi/180*(eu_chassis[0]), pi/180*(eu_chassis[1]), pi/180*(eu_chassis[2]))
#qt_os1     = quaternion_from_euler(pi/180*(eu_os1[0]),     pi/180*(eu_os1[1]),     pi/180*(eu_os1[2]))
#qt_t265    = quaternion_from_euler(pi/180*(eu_t265[0]),    pi/180*(eu_t265[1]),    pi/180*(eu_t265[2]))
#qt_d435i   = quaternion_from_euler(pi/180*(eu_d435i[0]),   pi/180*(eu_d435i[1]),   pi/180*(eu_d435i[2]))



tf_args = numpy.array([
                       [tf_world[0]       , tf_world[1]       , tf_world[2]       , tf_world[3]       , tf_world[4]       , tf_world[5]       , tf_world[6]       , "world"                               , "base_init"                           , 200 ],
                       [tf_chassis[0]     , tf_chassis[1]     , tf_chassis[2]     , tf_chassis[3]     , tf_chassis[4]     , tf_chassis[5]     , tf_chassis[6]     , "base_init"                           , "chassis_init"                        , 200 ],
                       [0                 , 0                 , 0                 , 0                 , 0                 , 0                 , 1                 , "chassis_init"                        , "wheel_init"                          , 10  ],
                       [0                 , 0                 , 0                 , 0                 , 0                 , 0                 , 1                 , "chassis_init"                        , "wheel_ekf_init"                      , 10  ],
                       [tf_t265[0]        , tf_t265[1]        , tf_t265[2]        , tf_t265[3]        , tf_t265[4]        , tf_t265[5]        , tf_t265[6]        , "chassis_init"                        , "t265_init"                           , 200 ],
#                       [tf_os1[0]         , tf_os1[1]         , tf_os1[2]         , tf_os1[3]         , tf_os1[4]         , tf_os1[5]         , tf_os1[6]         , "chassis_init"                        , "os1_init"                            , 100 ],
#                       [tf_os1[0]         , tf_os1[1]         , tf_os1[2]         , tf_os1[3]         , tf_os1[4]         , tf_os1[5]         , tf_os1[6]         , "chassis_init"                        , "os1_ekf_init"                        , 200 ],
                       [tf_d435i[0]       , tf_d435i[1]       , tf_d435i[2]       , tf_d435i[3]       , tf_d435i[4]       , tf_d435i[5]       , tf_d435i[6]       , "chassis_init"                        , "rtabmap_init"                        , 30  ],
                       [tf_xsens[0]       , tf_xsens[1]       , tf_xsens[2]       , tf_xsens[3]       , tf_xsens[4]       , tf_xsens[5]       , tf_xsens[6]       , "chassis_init"                        , "imu_link"                            , 100 ],
                       [0                 , 0                 , 0                 , 0                 , 0                 , 0                 , 1                 , "imu_link"                            , "imu"                                 , 100 ],
                       [0                 , 0                 , 0                 , 0                 , 0                 , 0                 , 1                 , "wheel_pose"                          , "wheel_chassis"                       , 10  ],
                       [0                 , 0                 , 0                 , 0                 , 0                 , 0                 , 1                 , "wheel_ekf_pose"                      , "wheel_ekf_chassis"                   , 10  ],
                       [tf_xsens[0]       , tf_xsens[1]       , tf_xsens[2]       , tf_xsens[3]       , tf_xsens[4]       , tf_xsens[5]       , tf_xsens[6]       , "wheel_ekf_chassis"                   , "imu_wheel_ekf"                       , 100 ],
                       [-tf_t265[0]       , -tf_t265[1]       , -tf_t265[2]       , -tf_t265[3]       , -tf_t265[4]       , -tf_t265[5]       , tf_t265[6]        , "t265_pose"                           , "t265_chassis"                        , 200 ],
#                       [-tf_os1[0]        , -tf_os1[1]        , -tf_os1[2]        , tf_os1[3]         , tf_os1[4]         , tf_os1[5]         , tf_os1[6]         , "os1_pose"                            , "os1_chassis"                         , 100 ],
#                       [-tf_os1[0]        , -tf_os1[1]        , -tf_os1[2]        , -tf_os1[3]        , -tf_os1[4]        , -tf_os1[5]        , -tf_os1[6]        , "os1_ekf_pose"                        , "os1_ekf_chassis"                     , 200 ],
#                       [0.15              , 0.01              , -0.085            , 0                 , 0                 , 0                 , 1                 , "decawave_pose"                       , "decawave_chassis"                    , 10  ],
#                       [-0.27             , 0                 , -0.025            , 0                 , 0                 , 0                 , 1                 , "arfront_pose"                        , "axis_front_chassis"                  , 30  ],
#                       [0.27              , 0                 , -0.025            , 0                 , 0                 , 0                 , 1                 , "arback_pose"                         , "axis_back_chassis"                   , 30  ],
                       [-tf_d435i[0]      , -tf_d435i[1]      , -tf_d435i[2]      , -tf_d435i[3]      , -tf_d435i[4]      , -tf_d435i[5]      , tf_d435i[6]       , "rtabmap_pose"                        , "rtabmap_chassis"                     , 30 ],
                       [-tf_chassis[0]    , -tf_chassis[1]    , -tf_chassis[2]    , -tf_chassis[3]    , -tf_chassis[4]    , -tf_chassis[5]    , tf_chassis[6]     , mode + "_chassis"                     , "base_link"                           , 200 ],
                       [tf_chassis[0]     , tf_chassis[1]     , tf_chassis[2]     , tf_chassis[3]     , tf_chassis[4]     , tf_chassis[5]     , tf_chassis[6]     , "base_link"                           , "chassis"                             , 200 ],
#                       [0.215             , 0.175             , 0                 , 0                 , 0                 , 0                 , 1                 , "chassis"                             , "left_front_wheel"                    , 10  ],
#                       [0                 , 0.225             , 0                 , 0                 , 0                 , 0                 , 1                 , "chassis"                             , "left_middle_wheel"                   , 10  ],
#                       [-0.215            , 0.175             , 0                 , 0                 , 0                 , 0                 , 1                 , "chassis"                             , "left_back_wheel"                     , 10  ],
#                       [0.215             , -0.175            , 0                 , 0                 , 0                 , 0                 , 1                 , "chassis"                             , "right_front_wheel"                   , 10  ],
#                       [0                 , -0.225            , 0                 , 0                 , 0                 , 0                 , 1                 , "chassis"                             , "right_middle_wheel"                  , 10  ],
#                       [-0.215            , -0.175            , 0                 , 0                 , 0                 , 0                 , 1                 , "chassis"                             , "right_back_wheel"                    , 10  ],
#                       [tf_os1[0]         , tf_os1[1]         , tf_os1[2]         , tf_os1[3]         , tf_os1[4]         , tf_os1[5]         , tf_os1[6]         , "chassis"                             , "os1_link"                            , 100 ],
#                       [0.27              , 0                 , 0.025             , 0                 , 0                 , 0                 , 1                 , "chassis"                             , "axis_link"                           , 30  ],
#                       [0.27              , 0                 , 0.025             , 0                 , 0                 , 0                 , 1                 , "chassis"                             , "axis_front_link"                     , 30  ],
#                       [-0.27             , 0                 , 0.025             , 0                 , 0                 , 0                 , 1                 , "chassis"                             , "axis_back_link"                      , 30  ],
                       [tf_t265[0]        , tf_t265[1]        , tf_t265[2]        , tf_t265[3]        , tf_t265[4]        , tf_t265[5]        , tf_t265[6]        , "chassis"                             , "t265_link"                           , 200 ],
#                       [tf_d435i[0]       , tf_d435i[1]       , tf_d435i[2]       , tf_d435i[3]       , tf_d435i[4]       , tf_d435i[5]       , tf_d435i[6]       , "chassis"                             , "d435i_link"                          , 200 ],
#                       [-0.15             , -0.01             , 0.085             , 0                 , 0                 , 0                 , 1                 , "chassis"                             , "decawave_link"                       , 10  ],
#                       [0.2               , 0.0325            , 0.125             , 0                 , 0                 , 0                 , 1                 , "chassis"                             , "kinect_link"                         , 60  ],
#                       [0                 , 0                 , 0.03618           , 0                 , 0                 , 1                 , 0                 , "os1_link"                            , "os1_lidar"                           , 10  ],
#                       [0.006253          , -0.011775         , 0.007645          , 0                 , 0                 , 0                 , 1                 , "os1_link"                            , "os1_imu"                             , 100 ],
#                       [0                 , 0                 , 0                 , -0.5              , 0.5               , -0.5              , 0.5               , "axis_link"                           , "axis_optical_frame"                  , 30  ],
#                       [0                 , 0                 , 0                 , -0.5              , 0.5               , -0.5              , 0.5               , "axis_front_link"                     , "axis_front_optical_frame"            , 30  ],
#                       [0                 , 0                 , 0                 , -0.5              , -0.5              , 0.5               , 0.5               , "axis_back_link"                      , "axis_back_optical_frame"             , 30  ],
                       [3.80186684197e-05 , 0.0320889987051   , 2.4195331207e-05  , 0.00326163647696  , -0.999985337257   , -0.00424767192453 , 0.000792266393546 , "t265_link"                           , "t265_fisheye1_frame"                 , 30  ],
                       [0                 , 0                 , 0                 ,  0.5              , -0.5              , -0.5              , 0.5               , "t265_fisheye1_frame"                 , "t265_fisheye1_optical_frame"         , 30  ],
                       [-3.80186684197e-05, -0.0320890024304  , -2.41953239311e-05, 0.000553577148821 , -0.99999755621    , -0.00205105380155 , 0.000613214506302 , "t265_link"                           , "t265_fisheye2_frame"                 , 30  ],
                       [0                 , 0                 , 0                 , 0.5               , -0.5              , -0.5              , 0.5               , "t265_fisheye2_frame"                 , "t265_fisheye2_optical_frame"         , 30  ],
#                       [-3.17073136102e-05, 0.0213896092027   , 0.000115149479825 , 0                 , 0                 , 1                 , 0                 , "t265_link"                           , "t265_gyro_frame"                     , 200 ],
#                       [0                 , 0                 , 0                 , 0.5               , -0.5              , -0.5              , 0.5               , "t265_gyro_frame"                     , "t265_gyro_optical_frame"             , 200 ],
#                       [-3.17073136102e-05, 0.0213896092027   , 0.000115149479825 , 0                 , 0                 , 1                 , 0                 , "t265_link"                           , "t265_accel_frame"                    , 62  ],
#                       [0                 , 0                 , 0                 , 0.5               , -0.5              , -0.5              , 0.5               , "t265_accel_frame"                    , "t265_accel_optical_frame"            , 62  ],
                       [0                 , 0                 , 0                 , 0                 , 0                 , 0                 , 1                 , "t265_link"                           , "t265_depth_frame"                    , 30  ],
                       [0                 , 0                 , 0                 , -0.5              , 0.5               , -0.5              , 0.5               , "t265_depth_frame"                    , "t265_depth_optical_frame"            , 30  ],
#                       [0                 , 0                 , 0                 , -0.5              , 0.5               , -0.5              , 0.5               , "t265_pose_frame"                     , "t265_depth_optical_frame"            , 200 ],
#                       [0                 , 0                 , 0                 , 0                 , 0                 , 0                 , 1                 , "t265_pose_frame"                     , "t265_link"                           , 200 ],
#                       [0                 , 0                 , 0                 , 0                 , 0                 , 0                 , 1                 , "t265_pose_frame"                     , "d435i_link"                          , 100 ],
#                       [0                 , 0                 , 0                 , 0                 , 0                 , 0                 , 1                 , "d435i_link"                          , "d435i_depth_frame"                   , 100 ],
#                       [0                 , 0                 , 0                 , -0.5              , 0.5               , -0.5              , 0.5               , "d435i_depth_frame"                   , "d435i_depth_optical_frame"           , 100 ],
#                       [-0.000214694606257, 0.0150816757232   , 7.47397498344e-05 , 0.0015756865032   , -0.000297236372717, 0.0036272148136   , 0.999992132187    , "d435i_link"                          , "d435i_color_frame"                   , 60  ],
#                       [-0.000214694606257, 0.0150816757232   , 7.47397498344e-05 , 0.0015756865032   , -0.000297236372717, 0.0036272148136   , 0.999992132187    , "d435i_link"                          , "d435i_aligned_depth_to_color_frame"  , 60  ],
#                       [0                 , 0                 , 0                 , -0.5              , 0.5               , -0.5              , 0.5               , "d435i_aligned_depth_to_color_frame"  , "d435i_color_optical_frame"           , 60  ],
#                       [0                 , 0                 , 0                 , 0                 , 0                 , 0                 , 1                 , "d435i_link"                          , "d435i_infra1_frame"                  , 100 ],
#                       [0                 , 0                 , 0                 , 0                 , 0                 , 0                 , 1                 , "d435i_link"                          , "d435i_aligned_depth_to_infra1_frame" , 100 ],
#                       [0                 , 0                 , 0                 , -0.5              , 0.5               , -0.5              , 0.5               , "d435i_aligned_depth_to_infra1_frame" , "d435i_infra1_optical_frame"          , 100 ],
#                       [0                 , -0.0499469712377  , 0                 , 0                 , 0                 , 0                 , 1                 , "d435i_link"                          , "d435i_infra2_frame"                  , 100 ],
#                       [0                 , 0                 , 0                 , -0.5              , 0.5               , -0.5              , 0.5               , "d435i_infra2_frame"                  , "d435i_infra2_optical_frame"          , 100 ],
#                       [-0.012            , -0.006            , 0.005             , 0                 , 0                 , 0                 , 1                 , "d435i_link"                          , "d435i_accel_frame"                   , 250 ],
#                       [0                 , 0                 , 0                 , -0.5              , 0.5               , -0.5              , 0.5               , "d435i_accel_frame"                   , "d435i_accel_optical_frame"           , 250 ],
#                       [-0.012            , -0.006            , 0.005             , 0                 , 0                 , 0                 , 1                 , "d435i_link"                          , "d435i_gyro_frame"                    , 400 ],
#                       [0                 , 0                 , 0                 , -0.5              , 0.5               , -0.5              , 0.5               , "d435i_gyro_frame"                    , "d435i_gyro_optical_frame"            , 400 ],
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
    
    global msg
    
    rospy.init_node("bagmerge_node", anonymous=True)
    
    rospy.sleep(0.5)
    
    for k in range(len(input_file)):
        sys.stdout.write("\n%s" % input_file[k])
        sys.stdout.flush()
    
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
    
#    sys.stdout.write("\n%s" % input_file)
#    sys.stdout.write("\n%s\n" % output_file)
#    sys.stdout.flush()
#    os.remove(output_file)
    
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
    
    odom_seq = 0
    k = 0
    
    while k < len(input_file):
        
        if "volta" in output_file:
            tf_t265_init  = (21.4719467163, 0.453556478024, -0.0645855739713, -0.000818398199044, 0.00119601248298, 0.983669400215, 0.179978996515)
        elif "200210_173013" in input_file[k]:
            tf_t265_init  = (-10.7891979218, -8.670835495, -0.227906405926, 0.00645246729255, 0.000483442592667, 0.979790627956, -0.199921101332)
        else:
            tf_t265_init  = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        
        for topic, msg, t in bag_in[k].read_messages():
            
            topic_bool = False
            
            if rospy.is_shutdown():
                k = len(input_file)
                break
            
            if match_bags and hasattr(msg, "header"):
                t = rospy.Time.from_sec(rospy.Time.to_sec(t) - bag_shift_time[k])
                msg.header.stamp = copy(t)
            
            if False:
                pass
            
#            elif topic == "/ar_pose_marker":
#                topic_bool = True
#                bag_out.write(topic, msg, t)
#            
#            elif topic == "/ar_pose_marker_back":
#                topic_bool = True
#                bag_out.write(topic, msg, t)
#            
#            elif topic == "/axis_back/camera_info":
#                topic_bool = True
#                bag_out.write(topic, msg, t)
#            
#            elif topic == "/axis_back/image_raw/compressed":
#                topic_bool = True
#                bag_out.write(topic, msg, t)
#            
#            elif topic == "/axis_front/camera_info":
#                topic_bool = True
#                bag_out.write(topic, msg, t)
#            
#            elif topic == "/axis_front/image_raw/compressed":
#                topic_bool = True
#                bag_out.write(topic, msg, t)
#            
            elif topic == "/cmd_vel":
                topic_bool = True
                bag_out.write(topic, msg, t)
#            
#            elif topic == "/decawave/tag_pose":
#                topic_bool = True
#                bag_out.write(topic, msg, t)
#                uwb_msg = Odometry()
#                uwb_msg.header = msg.header
#                uwb_msg.pose.pose.position.x = msg.x
#                uwb_msg.pose.pose.position.y = msg.y
#                uwb_msg.pose.pose.position.z = msg.z
#                uwb_msg.pose.pose.orientation.w = 1.0
#                bag_out.write("/decawave/odom", uwb_msg, t)
#            
#            elif topic == "/device1/get_joint_state":
#                topic_bool = True
#                bag_out.write(topic, msg, t)
#            
#            elif topic == "/device2/get_joint_state":
#                topic_bool = True
#                bag_out.write(topic, msg, t)
#            
#            elif topic == "/device3/get_joint_state":
#                topic_bool = True
#                bag_out.write(topic, msg, t)
#            
#            elif topic == "/device4/get_joint_state":
#                topic_bool = True
#                bag_out.write(topic, msg, t)
#            
#            elif topic == "/device5/get_joint_state":
#                topic_bool = True
#                bag_out.write(topic, msg, t)
#            
#            elif topic == "/device6/get_joint_state":
#                topic_bool = True
#                bag_out.write(topic, msg, t)
            
            elif topic == "/imu/data":
                topic_bool = True
                bag_out.write(topic, msg, t)
                msg_wheel = copy(msg)
                msg_wheel.header.frame_id = "imu_wheel_ekf"
                bag_out.write("/imu_wheel_ekf/data", msg_wheel, t)
            
#            elif topic == "/laser_odom_to_init":
#                topic_bool = True
#                bag_out.write(topic, msg, t)
#            
#            elif topic == "/integrated_to_init":
#                topic_bool = True
#                bag_out.write(topic, msg, t)
            
            elif topic == "/os1_cloud_node/points":
                topic_bool = True
                bag_out.write(topic, msg, t)
            
            elif topic == "/robot_odom" or topic == "/odom":
                topic_bool = True
                k = 0.01 + abs(msg.pose.pose.orientation.z)
                msg.pose.covariance = [      k,     0.0,     0.0,     0.0,     0.0,     0.0,
                                           0.0,       k,     0.0,     0.0,     0.0,     0.0,
                                           0.0,     0.0,     1.0,     0.0,     0.0,     0.0,
                                           0.0,     0.0,     0.0,     1.0,     0.0,     0.0,
                                           0.0,     0.0,     0.0,     0.0,     1.0,     0.0,
                                           0.0,     0.0,     0.0,     0.0,     0.0, 100.0*k]
                bag_out.write("/odom", msg, t)
            
            elif topic == "/realsense/gyro/sample":
                topic_bool = True
                topic = "/t265" + topic[10:]
                msg.header.frame_id = "t265" + msg.header.frame_id[9:]
                ang_vel = msg.angular_velocity
            
            elif topic == "/realsense/accel/sample":
                topic_bool = True
                topic = "/t265" + topic[10:]
                msg.header.frame_id = "t265" + msg.header.frame_id[9:]
                if "ang_vel" in locals():
                    imu_data = copy(msg)
                    imu_data.linear_acceleration.x = +copy(msg.linear_acceleration.z)
                    imu_data.linear_acceleration.y = -copy(msg.linear_acceleration.x)
                    imu_data.linear_acceleration.z = -copy(msg.linear_acceleration.y)
                    imu_data.angular_velocity.x = +copy(ang_vel.z)
                    imu_data.angular_velocity.y = -copy(ang_vel.x)
                    imu_data.angular_velocity.z = -copy(ang_vel.y)
                    bag_out.write("/t265/imu/sample", imu_data, t)
            
            elif topic == "/realsense/fisheye1/image_raw/compressed":
                topic_bool = True
                topic = "/left/image_raw/compressed"
                msg.header.frame_id = "t265" + msg.header.frame_id[9:]
                bag_out.write(topic, msg, t)
            
            elif topic == "/realsense/fisheye1/camera_info":
                topic_bool = True
                topic = "/left/camera_info"
                msg.header.frame_id = "t265" + msg.header.frame_id[9:]
                bag_out.write(topic, msg, t)
            
            elif topic == "/realsense/fisheye2/image_raw/compressed" or topic == "realsense/fisheye2/image_raw/compressed":
                topic_bool = True
                topic = "/right/image_raw/compressed"
                msg.header.frame_id = "t265" + msg.header.frame_id[9:]
                bag_out.write(topic, msg, t)
            
            elif topic == "/realsense/fisheye2/camera_info" or topic == "/realsense/fisheye2/camera_info/":
                topic_bool = True
                topic = "/right/camera_info"
                msg.header.frame_id = "t265" + msg.header.frame_id[9:]
                Tx = msg.P[0] * (2.0 * tf_args[tf_args[:,8]=="t265_fisheye2_frame",1][0])
                msg.P = msg.P[0:3] + (Tx,) + msg.P[4:12]
                bag_out.write(topic, msg, t)
            
            elif topic == "/realsense/odom/sample":
                
                topic_bool = True
                topic = "/t265" + topic[10:]
                msg.header.frame_id = "t265_init"
                msg.child_frame_id = "t265_pose"
                v_old = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
                q_old = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
                v_new = qv_mult(q_conjugate(tf_t265_init[3:]), (v_old[0]-tf_t265_init[0],v_old[1]-tf_t265_init[1],v_old[2]-tf_t265_init[2]))
                q_new = qq_mult(q_conjugate(tf_t265_init[3:]), q_old)
                msg.pose.pose.position.x = v_new[0]
                msg.pose.pose.position.y = v_new[1]
                msg.pose.pose.position.z = v_new[2]
                msg.pose.pose.orientation.x = q_new[0]
                msg.pose.pose.orientation.y = q_new[1]
                msg.pose.pose.orientation.z = q_new[2]
                msg.pose.pose.orientation.w = q_new[3]
                bag_out.write(topic, msg, t)
                
                clk_msg = Clock()
                clk_msg.clock = copy(t)
                bag_out.write("/clock", clk_msg, t)
            
            elif topic == "/tf":
                
                if msg.transforms[0].child_frame_id == "realsense_pose_frame":
                    
                    topic_bool = True
                    msg.transforms[0].header.frame_id = "t265_init"
                    msg.transforms[0].child_frame_id = "t265_pose"
                    v_old = (msg.transforms[0].transform.translation.x, msg.transforms[0].transform.translation.y, msg.transforms[0].transform.translation.z)
                    q_old = (msg.transforms[0].transform.rotation.x, msg.transforms[0].transform.rotation.y, msg.transforms[0].transform.rotation.z, msg.transforms[0].transform.rotation.w)
                    v_new = qv_mult(q_conjugate(tf_t265_init[3:]), (v_old[0]-tf_t265_init[0],v_old[1]-tf_t265_init[1],v_old[2]-tf_t265_init[2]))
                    q_new = qq_mult(q_conjugate(tf_t265_init[3:]), q_old)
                    msg.transforms[0].transform.translation.x = v_new[0]
                    msg.transforms[0].transform.translation.y = v_new[1]
                    msg.transforms[0].transform.translation.z = v_new[2]
                    msg.transforms[0].transform.rotation.x = q_new[0]
                    msg.transforms[0].transform.rotation.y = q_new[1]
                    msg.transforms[0].transform.rotation.z = q_new[2]
                    msg.transforms[0].transform.rotation.w = q_new[3]
                    bag_out.write(topic, msg, t)
                    
                    if "200123_163834" in input_file[k]:
                        odom_seq += 1
                        odom_msg = Odometry()
                        odom_msg.header = copy(msg.transforms[0].header)
                        odom_msg.header.seq = copy(odom_seq)
                        odom_msg.child_frame_id = copy(msg.transforms[0].child_frame_id)
                        odom_msg.pose.pose.position = copy(msg.transforms[0].transform.translation)
                        odom_msg.pose.pose.orientation = copy(msg.transforms[0].transform.rotation)
                        bag_out.write("/t265/odom/sample", odom_msg, t)
            
#                topic_bool = True
#                if msg.transforms[0].child_frame_id == "estimation":
#                    uwb_ekf_msg = Pose()
#                    uwb_ekf_msg.position = copy(msg.transforms[0].transform.translation)
#                    uwb_ekf_msg.orientation = copy(msg.transforms[0].transform.rotation)
#                    bag_out.write("/espeleo/pose", uwb_ekf_msg, t)
                
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