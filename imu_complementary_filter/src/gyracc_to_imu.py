#!/usr/bin/env python

import rospy
import genpy
import sys
from copy import copy
from sensor_msgs.msg import Imu


freq = 60
frame_id = "d435i_link"

msg = Imu()
msg_bool = [False, False]

def callback_gyro(data):
    global ang_vel, ang_vel_cov, msg_time, msg_bool
    ang_vel = copy(data.angular_velocity)
    ang_vel_cov = copy(data.angular_velocity_covariance)
    msg_time = rospy.Time.now().to_sec() - data.header.stamp.to_sec()
    msg_bool[0] = True


def callback_accel(data):
    global lin_acc, lin_acc_cov, msg_bool
    lin_acc = copy(data.linear_acceleration)
    lin_acc_cov = copy(data.linear_acceleration_covariance)
    msg_bool[1] = True


def main_function():
    
    global msg
    
    rospy.init_node("sync_node", anonymous=True)

    rospy.Subscriber("/d435i/accel/sample", Imu, callback_accel)
    rospy.Subscriber("/d435i/gyro/sample",  Imu, callback_gyro)

    pub = rospy.Publisher("/d435i/imu/sample", Imu, queue_size=1)
    
    while not all(msg_bool) and not rospy.is_shutdown():
        sys.stdout.write("\rWaiting for data publications... ")
        sys.stdout.flush()
        rospy.sleep(0.01)
    sys.stdout.write("Ok!\n")
    sys.stdout.flush()
    
    rate = rospy.Rate(freq)
    
    while not rospy.is_shutdown():
        
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.from_sec(rospy.Time.now().to_sec() - msg_time)
        msg.header.frame_id = frame_id
        
        msg.angular_velocity.x = +copy(ang_vel.z)
        msg.angular_velocity.y = -copy(ang_vel.x)
        msg.angular_velocity.z = -copy(ang_vel.y)
        msg.angular_velocity_covariance = copy(ang_vel_cov)
        
        msg.linear_acceleration.x = +copy(lin_acc.z)
        msg.linear_acceleration.y = -copy(lin_acc.x)
        msg.linear_acceleration.z = -copy(lin_acc.y)
        msg.linear_acceleration_covariance = copy(lin_acc_cov)

        pub.publish(msg)

        rate.sleep()

if __name__ == "__main__":
    try:
        main_function()
    except rospy.ROSInterruptException:
        pass
