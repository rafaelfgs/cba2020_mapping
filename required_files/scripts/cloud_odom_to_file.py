#!/usr/bin/python

import rospy
import genpy
import sys
import os
from copy import copy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud



place = sys.argv[1] # corredor, veloso
mode  = sys.argv[2] # nuvem, depth, triang, stereo

file_pcl   = "/mnt/WD500/UFMG/CBA/results/" + place + "_pointcloud_" + mode + ".pcd"
file_t265  = "/mnt/WD500/UFMG/CBA/results/" + place + "_t265.txt"
file_wheel = "/mnt/WD500/UFMG/CBA/results/" + place + "_wheel.txt"
file_odom  = "/mnt/WD500/UFMG/CBA/results/" + place + "_" + mode + ".txt"

px_t265,  py_t265,  pz_t265,  qx_t265,  qy_t265,  qz_t265,  qw_t265,  t_t265,  n_t265  = [], [], [], [], [], [], [], [], 0
px_odom,  py_odom,  pz_odom,  qx_odom,  qy_odom,  qz_odom,  qw_odom,  t_odom,  n_odom  = [], [], [], [], [], [], [], [], 0
px_wheel, py_wheel, pz_wheel, qx_wheel, qy_wheel, qz_wheel, qw_wheel, t_wheel, n_wheel = [], [], [], [], [], [], [], [], 0
xyz_pcl,  rgb_pcl,  num_pcl = [], [], 0

t0 = 0.0

subscribe_data = True



def callback_pcl(data):
    
    if subscribe_data:
    
        global xyz_pcl, rgb_pcl, num_pcl
        
        xyz_pcl = data.points
        rgb_pcl = data.channels[0].values
        num_pcl = len(data.points)



def callback_t265(data):
    
    if subscribe_data:
    
        global t_t265, px_t265, py_t265, pz_t265, qx_t265, qy_t265, qz_t265, qw_t265, n_t265, t0
        
        if t0 == 0.0:
            t0 = genpy.Time.to_sec(data.header.stamp)
        
        t_t265  += [genpy.Time.to_sec(data.header.stamp) - t0]
        
        px_t265 += [copy(data.pose.pose.position.x)]
        py_t265 += [copy(data.pose.pose.position.y)]
        pz_t265 += [copy(data.pose.pose.position.z)]
        
        qx_t265 += [copy(data.pose.pose.orientation.x)]
        qy_t265 += [copy(data.pose.pose.orientation.y)]
        qz_t265 += [copy(data.pose.pose.orientation.z)]
        qw_t265 += [copy(data.pose.pose.orientation.w)]
        
        n_t265 = len(t_t265)



def callback_wheel(data):
    
    if subscribe_data:
    
        global t_wheel, px_wheel, py_wheel, pz_wheel, qx_wheel, qy_wheel, qz_wheel, qw_wheel, n_wheel, t0
        
        if t0 == 0.0:
            t0 = genpy.Time.to_sec(data.header.stamp)
        
        t_wheel  += [genpy.Time.to_sec(data.header.stamp) - t0]
        
        px_wheel += [copy(data.pose.pose.position.x)]
        py_wheel += [copy(data.pose.pose.position.y)]
        pz_wheel += [copy(data.pose.pose.position.z)]
        
        qx_wheel += [copy(data.pose.pose.orientation.x)]
        qy_wheel += [copy(data.pose.pose.orientation.y)]
        qz_wheel += [copy(data.pose.pose.orientation.z)]
        qw_wheel += [copy(data.pose.pose.orientation.w)]
        
        n_wheel = len(t_wheel)



def callback_odom(data):
    
    if subscribe_data:
    
        global t_odom, px_odom, py_odom, pz_odom, qx_odom, qy_odom, qz_odom, qw_odom, n_odom, t0
        
        if t0 == 0.0:
            t0 = genpy.Time.to_sec(data.header.stamp)
        
        t_odom  += [genpy.Time.to_sec(data.header.stamp) - t0]
        
        px_odom += [copy(data.pose.pose.position.x)]
        py_odom += [copy(data.pose.pose.position.y)]
        pz_odom += [copy(data.pose.pose.position.z)]
        
        qx_odom += [copy(data.pose.pose.orientation.x)]
        qy_odom += [copy(data.pose.pose.orientation.y)]
        qz_odom += [copy(data.pose.pose.orientation.z)]
        qw_odom += [copy(data.pose.pose.orientation.w)]
        
        n_odom = len(t_odom)



def main_function():
    
    
    
    global subscribe_data, save_cloud, save_odom, t0
    
    rospy.init_node("data_to_file_node", anonymous=True)
    
    
    
    bool_input = [False, False, False, False]
    
    if raw_input("Save Pointcloud File? (y/n): ") == "y":
        bool_input[0] = True
    if mode != "nuvem":
        if raw_input("Save RTAB-Map " + mode + " Odometry File? (y/n): ") == "y":
            bool_input[1] = True
        if (place == "corredor" and mode == "triang") or (place == "veloso" and mode == "depth"):
            if raw_input("Save T265 Odometry File? (y/n): ") == "y":
                bool_input[2] = True
            if raw_input("Save EKF Odometry File? (y/n): ") == "y":
                bool_input[3] = True
    
    
    
#    if os.path.exists(file_pcl):
#        if raw_input("Pointcloud file already exists, do you want to replace it? (y/n): ") == "y":
#            bool_input[0] = True
#    if mode != "nuvem":
#        if os.path.exists(file_odom):
#            if raw_input("RTAB-Map " + mode + " Odometry file already exists, do you want to replace it? (y/n): ") == "y":
#                bool_input[3] = True
#        if (place == "corredor" and mode == "triang") or (place == "veloso" and mode == "depth"):
#            if os.path.exists(file_t265):
#                if raw_input("T265 Odometry file already exists, do you want to replace it? (y/n): ") == "y":
#                    bool_input[1] = True
#            if os.path.exists(file_wheel):
#                if raw_input("Wheel-EKF file already exists, do you want to replace it? (y/n): ") == "y":
#                    bool_input[2] = True
#    
#    
#    
#    if any(bool_input):
#        str_replace = "Replace files and start subscriptions? (y/n): "
#    else:
#        str_replace = "Start subscriptions? (y/n): "
    
    
    
    if raw_input("Start Subscriptions? (y/n): ") == "y":
        if bool_input[0] and os.path.exists(file_pcl):   os.remove(file_pcl)
        if bool_input[1] and os.path.exists(file_odom):  os.remove(file_odom)
        if bool_input[2] and os.path.exists(file_t265):  os.remove(file_t265)
        if bool_input[3] and os.path.exists(file_wheel): os.remove(file_wheel)
    else:
        sys.exit(0)
    
    
    
    if bool_input[0]: rospy.Subscriber("/cloud/points",         PointCloud, callback_pcl)
    if bool_input[1]: rospy.Subscriber("/rtabmap/odom_world",   Odometry,   callback_odom)
    if bool_input[2]: rospy.Subscriber("/t265/odom_world",      Odometry,   callback_t265)
    if bool_input[3]: rospy.Subscriber("/wheel_ekf/odom_world", Odometry,   callback_wheel)
    
    
    
    raw_input("Press enter to stop subscribing and save the data (Crtl+C enter to shut down): ")
    subscribe_data = False
    
    
    
    if rospy.is_shutdown():
        
        
        
        sys.stdout.write("Shutting down...\n")
        sys.stdout.flush()
        
        
        
    else:
        
        if num_pcl > 0:
        
            f = open(file_pcl,"w")
            
            f.write("# .PCD v0.7 - Point Cloud Data file format\n")
            f.write("VERSION 0.7\n")
            f.write("FIELDS x y z rgb\n")
            f.write("SIZE 4 4 4 4\n")
            f.write("TYPE F F F F\n")
            f.write("COUNT 1 1 1 1\n")
            f.write("WIDTH 1\n")
            f.write("HEIGHT %s\n" % num_pcl)
            f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            f.write("POINTS %s\n" % num_pcl)
            f.write("DATA ascii")
            
            k = 0
            while not rospy.is_shutdown() and k < num_pcl:
                
                f.write("\n%s %s %s %s" % (xyz_pcl[k].x, xyz_pcl[k].y, xyz_pcl[k].z, rgb_pcl[k]))
                k += 1
                sys.stdout.write("\rSaving PointCloud File (%d)... %5.1f%%" % (num_pcl, 100.0*k/num_pcl))
                sys.stdout.flush()
                
            sys.stdout.write("\n")
            sys.stdout.flush()
            f.close()
        
        
        
        if mode != "nuvem":
            
            
            
            if n_odom > 0:
                
                f = open(file_odom,"w")
                
                k = 0
                f.write("t px py pz qx qy qz qw")
                while not rospy.is_shutdown() and k < n_odom:
                    
                    f.write("\n%s %s %s %s %s %s %s %s" % (t_odom[k],  px_odom[k], py_odom[k], pz_odom[k],
                                                           qx_odom[k], qy_odom[k], qz_odom[k], qw_odom[k]))
                    k += 1
                    sys.stdout.write("\rSaving RTAB-Map " + mode + " Odometry File... %5.1f%%" % (100.0*k/n_odom))
                    sys.stdout.flush()
                    
                sys.stdout.write("\n")
                sys.stdout.flush()
                f.close()
            
            
            
            if n_t265 > 0 and mode != "stereo":
            
                f = open(file_t265,"w")
                
                k = 0
                f.write("t px py pz qx qy qz qw")
                while not rospy.is_shutdown() and k < n_t265:
                    
                    f.write("\n%s %s %s %s %s %s %s %s" % (t_t265[k],  px_t265[k], py_t265[k], pz_t265[k],
                                                           qx_t265[k], qy_t265[k], qz_t265[k], qw_t265[k]))
                    k += 1
                    sys.stdout.write("\rSaving T265 Odometry File... %5.1f%%" % (100.0*k/n_t265))
                    sys.stdout.flush()
                    
                sys.stdout.write("\n")
                sys.stdout.flush()
                f.close()
            
            
            
            if n_wheel > 0 and mode != "stereo":
            
                f = open(file_wheel,"w")
                
                k = 0
                f.write("t px py pz qx qy qz qw")
                while not rospy.is_shutdown() and k < n_wheel:
                    
                    f.write("\n%s %s %s %s %s %s %s %s" % (t_wheel[k],  px_wheel[k], py_wheel[k], pz_wheel[k],
                                                           qx_wheel[k], qy_wheel[k], qz_wheel[k], qw_wheel[k]))
                    k += 1
                    sys.stdout.write("\rSaving Wheel-EKF Odometry File... %5.1f%%" % (100.0*k/n_wheel))
                    sys.stdout.flush()
                    
                sys.stdout.write("\n")
                sys.stdout.flush()
                f.close()
            


if __name__ == "__main__":
    try:
        main_function()
    except rospy.ROSInterruptException:
        pass