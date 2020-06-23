#!/usr/bin/env python

import rospy
import sys
from copy import copy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped


place = sys.argv[1] # corredor, volta, veloso

if place == "volta":
    tf_world = (22.85, 0.0, 0.15, 0.0, 0.0, 0.9818535, 0.1896408)
else:
    tf_world = (0.0, 0.0, 0.15, 0.0, 0.0, 0.0, 1.0)


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



def callback_ekf(data):
    
    odom = Odometry()
    odom.header = copy(data.header)
    odom.child_frame_id = "wheel_ekf_init"
    odom.pose = copy(data.pose)
    pub = rospy.Publisher("/robot_pose_ekf/odom_combined", Odometry, queue_size=1)
    pub.publish(odom)
    
    if place == "veloso":
        tf_base = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    else:
        tf_base = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    
    v_sensor = (data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z)
    q_sensor = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    
    v_base = qv_mult(q_sensor, tf_base[:3])
    v_base = qv_mult(q_conjugate(tf_base[3:]), (v_sensor[0]-v_base[0], v_sensor[1]-v_base[1], v_sensor[2]-v_base[2]))
    q_base = qq_mult(q_conjugate(tf_base[3:]), q_sensor)
    
    v_init = qv_mult(tf_base[3:], (v_base[0]+tf_base[0], v_base[1]+tf_base[1], v_base[2]+tf_base[2]))
    q_init = qq_mult(tf_base[3:], q_base)
    
#    v_base = qv_mult(q_sensor, tf_base[:3])
#    v_base = qv_mult(q_conjugate(tf_base[3:]), (v_sensor[0]-v_base[0], v_sensor[1]-v_base[1], v_sensor[2]-v_base[2]))
#    q_base = qq_mult(q_conjugate(tf_base[3:]), q_sensor)
    
#    v_init = qv_mult(tf_base[3:], (v_base[0]+tf_base[0], v_base[1]+tf_base[1], v_base[2]+tf_base[2]))
#    q_init = qq_mult(tf_base[3:], q_base)
    
    v_world = qv_mult(tf_world[3:], v_init)
    v_world = (v_world[0]+tf_world[0], v_world[1]+tf_world[1], v_world[2]+tf_world[2])
    q_world = qq_mult(tf_world[3:], q_init)
    
    odom = Odometry()
    
    odom.header.seq += 1
    odom.header.frame_id = "world"
    odom.header.stamp = copy(data.header.stamp)
    
    odom.child_frame_id = "wheel_ekf_pose"
    
    odom.pose.pose.position.x = v_world[0]
    odom.pose.pose.position.y = v_world[1]
    odom.pose.pose.position.z = v_world[2] + 0.1
    
    odom.pose.pose.orientation.x = q_world[0]
    odom.pose.pose.orientation.y = q_world[1]
    odom.pose.pose.orientation.z = q_world[2]
    odom.pose.pose.orientation.w = q_world[3]
    
    odom.pose.covariance = copy(data.pose.covariance)
    
    pub = rospy.Publisher("/wheel_ekf/odom_world", Odometry, queue_size=1)
    pub.publish(odom)



def callback_rtab(data):
    
    if place == "veloso":
        tf_base = (0.07, 0.0, 0.16, 0.0, +0.0087265, 0.0, 0.9999619)
    else:
        tf_base = (0.27, 0.0, 0.07, 0.0, -0.0017453, 0.0, 0.9999985)
    
    v_sensor = (data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z)
    q_sensor = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    
    v_base = qv_mult(q_sensor, tf_base[:3])
    v_base = qv_mult(tf_base[3:], (v_sensor[0]-v_base[0], v_sensor[1]-v_base[1], v_sensor[2]-v_base[2]))
    q_base = qq_mult(tf_base[3:], q_sensor)
    
    v_init = (v_base[0]+tf_base[0], v_base[1]+tf_base[1], v_base[2]+tf_base[2])
    q_init = copy(q_base)
    
#    v_base = qv_mult(q_sensor, tf_base[:3])
#    v_base = qv_mult(q_conjugate(tf_base[3:]), (v_sensor[0]-v_base[0], v_sensor[1]-v_base[1], v_sensor[2]-v_base[2]))
#    q_base = qq_mult(q_conjugate(tf_base[3:]), q_sensor)
    
#    v_init = qv_mult(tf_base[3:], (v_base[0]+tf_base[0], v_base[1]+tf_base[1], v_base[2]+tf_base[2]))
#    q_init = qq_mult(tf_base[3:], q_base)
    
    v_world = qv_mult(tf_world[3:], v_init)
    v_world = (v_world[0]+tf_world[0], v_world[1]+tf_world[1], v_world[2]+tf_world[2])
    q_world = qq_mult(tf_world[3:], q_init)
    
    odom = Odometry()
    
    odom.header.seq += 1
    odom.header.frame_id = "world"
    odom.header.stamp = copy(data.header.stamp)
    
    odom.child_frame_id = "rtabmap_pose"
    
    odom.pose.pose.position.x = v_world[0]
    odom.pose.pose.position.y = v_world[1]
    odom.pose.pose.position.z = v_world[2] + 0.1
    
    odom.pose.pose.orientation.x = q_world[0]
    odom.pose.pose.orientation.y = q_world[1]
    odom.pose.pose.orientation.z = q_world[2]
    odom.pose.pose.orientation.w = q_world[3]
    
    odom.pose.covariance = copy(data.pose.covariance)
    
    pub = rospy.Publisher("/rtabmap/odom_world", Odometry, queue_size=1)
    pub.publish(odom)



def callback_t265(data):
    
    if place == "veloso":
        tf_base = (0.07, 0.0, 0.16, 0.0, -0.0419877, 0.0, 0.9991181)
    else:
        tf_base = (0.27, 0.0, 0.07, 0.0, -0.0017453, 0.0, 0.9999985)
    
    v_sensor = (data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z)
    q_sensor = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    
    v_base = qv_mult(q_sensor, tf_base[:3])
    v_base = qv_mult(tf_base[3:], (v_sensor[0]-v_base[0], v_sensor[1]-v_base[1], v_sensor[2]-v_base[2]))
    q_base = qq_mult(tf_base[3:], q_sensor)
    
    v_init = (v_base[0]+tf_base[0], v_base[1]+tf_base[1], v_base[2]+tf_base[2])
    q_init = copy(q_base)
    
#    v_base = qv_mult(q_sensor, tf_base[:3])
#    v_base = qv_mult(q_conjugate(tf_base[3:]), (v_sensor[0]-v_base[0], v_sensor[1]-v_base[1], v_sensor[2]-v_base[2]))
#    q_base = qq_mult(q_conjugate(tf_base[3:]), q_sensor)
    
#    v_init = qv_mult(tf_base[3:], (v_base[0]+tf_base[0], v_base[1]+tf_base[1], v_base[2]+tf_base[2]))
#    q_init = qq_mult(tf_base[3:], q_base)
    
    v_world = qv_mult(tf_world[3:], v_init)
    v_world = (v_world[0]+tf_world[0], v_world[1]+tf_world[1], v_world[2]+tf_world[2])
    q_world = qq_mult(tf_world[3:], q_init)
    
    odom = Odometry()
    
    odom.header.seq += 1
    odom.header.frame_id = "world"
    odom.header.stamp = data.header.stamp
    
    odom.child_frame_id = "t265_pose"
    
    odom.pose.pose.position.x = v_world[0]
    odom.pose.pose.position.y = v_world[1]
    odom.pose.pose.position.z = v_world[2] + 0.1
    
    odom.pose.pose.orientation.x = q_world[0]
    odom.pose.pose.orientation.y = q_world[1]
    odom.pose.pose.orientation.z = q_world[2]
    odom.pose.pose.orientation.w = q_world[3]
    
    odom.pose.covariance = data.pose.covariance
    
    pub = rospy.Publisher("/t265/odom_world", Odometry, queue_size=1)
    pub.publish(odom)



def main_function():

    rospy.init_node("output_to_world_node", anonymous=True)
    
    rospy.Subscriber("/robot_pose_ekf/pose_combined", PoseWithCovarianceStamped, callback_ekf)
    rospy.Subscriber("/rtabmap/odom", Odometry, callback_rtab)
    rospy.Subscriber("/t265/odom/sample", Odometry, callback_t265)

    sys.stdout.write("Republishing odom in respect to world\n")
    sys.stdout.flush()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()



if __name__ == "__main__":
    try:
        main_function()
    except rospy.ROSInterruptException:
            pass