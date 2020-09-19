#!/usr/bin/python

import rospy
import numpy
import time
import sys
import struct
from math import pi
from copy import copy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
from sensor_msgs.msg import ChannelFloat32, PointCloud
from tf.transformations import quaternion_from_euler



dmin = 2.0 # 0.2
dmax = 5.0 # 4.5
period_ms = 5000 #  4500

v_rs = (0.27, 0.0, 0.35)
q_rs = quaternion_from_euler(0.0, pi/180*7, pi/180*1)

msg_ready = [False, False, False, False, False]
code_ready = True

bridge = CvBridge()
msg_cloud = PointCloud()



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



def callback_rgb_img(data):
    if code_ready:
        global rgb_img, msg_cloud, msg_ready
        rgb_img = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        msg_cloud.header.stamp = copy(data.header.stamp)
        msg_ready[0] = True

def callback_rgb_info(data):
    if code_ready:
        global fx_rgb, fy_rgb, msg_cloud, msg_ready
        fx_rgb = data.K[0]
        fy_rgb = data.K[4]
        msg_cloud.header.stamp = copy(data.header.stamp)
        msg_ready[1] = True

def callback_depth_img(data):
    if code_ready:
        global depth_img, msg_cloud, msg_ready
        depth_img = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        msg_cloud.header.stamp = copy(data.header.stamp)
        msg_ready[2] = True

def callback_depth_info(data):
    if code_ready:
        global fx_depth, fy_depth, msg_cloud, msg_ready
        fx_depth = data.K[0] * 1.5
        fy_depth = data.K[4] * 1.5
        msg_cloud.header.stamp = copy(data.header.stamp)
        msg_ready[3] = True

def callback_odom(data):
    if code_ready:
        global odom_pos, odom_ori, msg_ready
        odom_pos = data.pose.pose.position
        odom_ori = data.pose.pose.orientation
        msg_ready[4] = True



def main_function():
    
    global msg_ready, code_ready, msg_cloud, pub_cloud, rgb
    
    rospy.init_node("depth_to_cloud_node", anonymous=True)
    
    rospy.Subscriber("/rgb/image_rect",    Image,      callback_rgb_img)
    rospy.Subscriber("/rgb/camera_info",   CameraInfo, callback_rgb_info)
    rospy.Subscriber("/depth/image_rect",  Image,      callback_depth_img)
    rospy.Subscriber("/depth/camera_info", CameraInfo, callback_depth_info)
    rospy.Subscriber("/t265/odom_world",   Odometry,   callback_odom)
    
    pub_cloud = rospy.Publisher("/cloud/points", PointCloud, queue_size=1)
    
    msg_cloud.header.frame_id = "world"
    msg_cloud.channels = [ChannelFloat32()]
    msg_cloud.channels[0].name = "rgb" #"intensity"
    num = 0
    
    t = time.time()
    sys.stdout.write("\n")
    
    while not rospy.is_shutdown():
        
        while not rospy.is_shutdown() and time.time()-t < period_ms/1000.0:
            
            time.sleep(0.001)
            sys.stdout.write("\rWaiting for Topics Publishing... ")
            sys.stdout.flush()
            
            if all(msg_ready):
                code_ready = False
                msg_ready = [False, False, False, False, False]
                sys.stdout.write("Ok!")
                sys.stdout.flush()
                
        t = time.time()
        
        if not code_ready:
            
            m = len(depth_img)
            n = len(depth_img[0])
            
            z_depth = depth_img.flatten() / 1000.0
            z_depth[z_depth==0.0] = dmax + 1.0
            
            i_depth = numpy.arange(0,m*n) / n
            j_depth = numpy.arange(0,m*n) % n
            
            u_depth = j_depth - n / 2.0 - 0.5
            v_depth = i_depth - m / 2.0 - 0.5
            
            x_depth = u_depth * z_depth / fx_depth
            y_depth = v_depth * z_depth / fy_depth
            
            u_rgb = fx_rgb * x_depth / z_depth
            v_rgb = fy_rgb * y_depth / z_depth
            i_rgb = (v_rgb + m / 2.0 + 1.0).astype(int)
            j_rgb = (u_rgb + n / 2.0 + 1.0).astype(int)
            
            idx = (i_rgb>=0) & (i_rgb<m) & (j_rgb>=0) & (j_rgb<n) & (z_depth>dmin) & (z_depth<dmax)
            
            x = x_depth[idx]
            y = y_depth[idx]
            z = z_depth[idx]
            num = len(z)
            
            msg_cloud.header.seq += 1
            
            rgb = rgb_img.reshape(m*n,3)[idx].astype(int)
            
            rgb_hex = (rgb[:,0]<<16) + (rgb[:,1]<<8) + (rgb[:,2])
            
            for k in range(num):
                rgb_float = struct.unpack("f", struct.pack("i", rgb_hex[k]))[0]
                msg_cloud.channels[0].values += [rgb_float]
            
            
#            rgb_float = numpy.mean(rgb,1)
#            
#            rgb_float = (rgb[:,0]).astype(int)<<16 + (rgb[:,1]).astype(int)<<8 + (rgb[:,2]).astype(int)
#            rgb_float = (rgb[:,0]).astype(int)<<16 | (rgb[:,1]).astype(int)<<8 | (rgb[:,2]).astype(int)
#            rgb_float = rgb[:,0]<<16 + rgb[:,1]<<8 + rgb[:,2]
#            rgb_float = rgb[:,0]<<16 | rgb[:,1]<<8 | rgb[:,2]
#            rgb_float = rgb[:,0]<<32 + rgb[:,1]<<16 + rgb[:,2]<<8 + numpy.repeat(255,num)
#            rgb_float = rgb[:,0]<<32 | rgb[:,1]<<16 | rgb[:,2]<<8 | numpy.repeat(255,num)
#            
#            for k in range(num):
#                rgb_float = struct.unpack('I', struct.pack('BBBB', rgb[k,2], rgb[k,1], rgb[k,0], 255))[0]
#                msg_cloud.channels[0].values += [rgb_float]
#            
#            msg_cloud.channels[0].values += rgb_float.tolist()
            
            k0 = len(msg_cloud.points)
            msg_cloud.points += [None] * num
            
            for k in range(num):
                
                v_optical = (z[k], -x[k], -y[k])
                q_optical = (q_rs[0], q_rs[1], q_rs[2], q_rs[3])
                v_local = qv_mult(q_optical, v_optical)
                v_odom = (odom_pos.x, odom_pos.y, odom_pos.z)
                q_odom = (odom_ori.x, odom_ori.y, odom_ori.z, odom_ori.w)
                v_temp = qv_mult(q_odom, v_local)
                v_global = (v_rs[0]+v_odom[0]+v_temp[0], v_rs[1]+v_odom[1]+v_temp[1], v_rs[2]+v_odom[2]+v_temp[2])
                
                msg_k = k0 + k
                msg_cloud.points[msg_k] = Point32()
                msg_cloud.points[msg_k].x = v_global[0]
                msg_cloud.points[msg_k].y = v_global[1]
                msg_cloud.points[msg_k].z = v_global[2]
        
        pub_cloud.publish(msg_cloud)
        
        code_ready = True
        
        sys.stdout.write("\n%d Points Computed in %dms\n" % (num, 1000.0*(time.time()-t)))
        sys.stdout.write("Global Map with %d Points\n\n" % len(msg_cloud.points))
        sys.stdout.flush()
        

if __name__ == "__main__":
    try:
        main_function()
    except rospy.ROSInterruptException:
        pass