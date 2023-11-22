#!/usr/bin/env python
''' 
    Purpose: Compute the transform between sensors to Ring Road Local UTM Reference Frame
    Subscribed topics: /lvx_client_node/gsof/49
    Published topics: /transforms/LUTM_T_A /transforms/LUTM_T_L /ego_odom

    Project: WATonoBus
    Author: Neel Bhatt
'''

import rospy
import numpy as np
from math import sin,cos,pi
import geodesy.utm # sudo apt-get install ros-$YOUR_DISTRO_NAME-geodesy
from transformation_utils import * # Custom function in root directory
from sensor_msgs.msg import NavSatFix
from applanix_publisher.msg import NavigationSolution
from std_msgs.msg import Float32MultiArray
from rospy.numpy_msg import numpy_msg
from nav_msgs.msg import Odometry
import tf
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from tf.transformations import euler_from_matrix

import getpass

# Local reference frame offsets
ref_easting = 537132
ref_northing = 4813391

tf_broadcast = tf.TransformBroadcaster()
def callback_Applanix(nav_msg):
    utm_pos = geodesy.utm.fromLatLong(nav_msg.lla.latitude, nav_msg.lla.longitude)

    roll_alpha_applanix = 0
    pitch_beta_applanix = 180
    yaw_gamma_applanix = -1*nav_msg.heading

    translation_x_applanix = utm_pos.easting - ref_easting
    translation_y_applanix = utm_pos.northing - ref_northing
    translation_z_applanix = 0 # Or use nav_msg.lla.altitude

    # Publish Odom of Applanix frame
    q=quaternion_from_euler(roll_alpha_applanix*pi/180, pitch_beta_applanix*pi/180, yaw_gamma_applanix*pi/180)
    ego_odom = Odometry()
    ego_odom.header.frame_id = "map"
    ego_odom.pose.pose.position.x=translation_x_applanix
    ego_odom.pose.pose.position.y=translation_y_applanix
    ego_odom.pose.pose.position.z=translation_z_applanix
    ego_odom.pose.pose.orientation.x = q[0]
    ego_odom.pose.pose.orientation.y = q[1]
    ego_odom.pose.pose.orientation.z = q[2]
    ego_odom.pose.pose.orientation.w = q[3]
    ego_odom.twist.twist.linear.x = float(nav_msg.total_speed)  # This is a hack - linear.x contains total speed instad of the x component
    applanix_pub.publish(ego_odom)

    # Publish tf for Rviz
    t_tf = translation_x_applanix, translation_y_applanix, translation_z_applanix
    q_tf = quaternion_from_euler(roll_alpha_applanix*pi/180,pitch_beta_applanix*pi/180,yaw_gamma_applanix*pi/180)

    tf_broadcast.sendTransform(t_tf,q_tf,rospy.Time.now(),"applanix","map")

    # --------------------- Generate Transformation Matrix From A to UTM ---------------------
    UTM_p_UTMA = np.array([[translation_x_applanix],[translation_y_applanix],[translation_z_applanix]])
    UTM_R_A = rot_matrix(roll_alpha_applanix,pitch_beta_applanix,yaw_gamma_applanix)

    rotation_matrix_applanix = UTM_R_A
    translation_vector_applanix = UTM_p_UTMA
    
    # ----------- Generate Transformation Matrix From A to UTM i,e. UTM_T_A, given Rotation and Translation -----------
    UTM_T_A = np.hstack((rotation_matrix_applanix,translation_vector_applanix))
    UTM_T_A = np.vstack((UTM_T_A,np.array([0,0,0,1])))

    # --------------------- Generate Transformation Matrix From L to UTM ---------------------
    UTM_T_L = np.matmul(UTM_T_A,A_T_L)
    
    # Publish tf for Rviz
    translation_x_velodyne = UTM_T_L[0,3]
    translation_y_velodyne = UTM_T_L[1,3]
    translation_z_velodyne = UTM_T_L[2,3]

    roll_alpha_velodyne, pitch_beta_velodyne, yaw_gamma_velodyne = euler_from_matrix(UTM_T_L[0:3,0:3], 'sxyz')

    t_tf = translation_x_velodyne, translation_y_velodyne, translation_z_velodyne
    q_tf = quaternion_from_euler(roll_alpha_velodyne,pitch_beta_velodyne,yaw_gamma_velodyne)    

    tf_broadcast.sendTransform(t_tf,q_tf,rospy.Time.now(),"rslidar_front","map")
    
    # --------------------- Generate Transformation Matrix From R_F to UTM ---------------------
    A_T_BPF = np.matmul(A_T_L,L_T_BPF)
    UTM_T_BPF = np.matmul(UTM_T_A,A_T_BPF)
    
    # Publish tf for Rviz
    translation_x_BPF = UTM_T_L[0,3]
    translation_y_BPF = UTM_T_L[1,3]
    translation_z_BPF = UTM_T_L[2,3]

    roll_alpha_BPF, pitch_beta_BPF, yaw_gamma_BPF = euler_from_matrix(UTM_T_L[0:3,0:3], 'sxyz')

    t_tf = translation_x_BPF, translation_y_BPF, translation_z_BPF
    q_tf = quaternion_from_euler(roll_alpha_BPF,pitch_beta_BPF,yaw_gamma_BPF)    

    tf_broadcast.sendTransform(t_tf,q_tf,rospy.Time.now(),"rslidar_BP_F","map")

    # Publish a flattened transform array for UTM_T_A and UTM_T_L
    applanix_transform_pub.publish(Float32MultiArray(data=UTM_T_A.flatten()))
    velodyne_transform_pub.publish(Float32MultiArray(data=UTM_T_L.flatten()))
    BPF_transform_pub.publish(Float32MultiArray(data=UTM_T_BPF.flatten()))

def sensors2localUTMpublisher():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('sensors2localUTMpublisher', anonymous=True)
    print("[INFO]: Transform Publisher Running ... Available Topics:")
    print("[INFO]: /transforms/LUTM_T_A")
    print("[INFO]: /transforms/LUTM_T_L")
    print("[INFO]: /transforms/LUTM_T_BPF") 
    print("[INFO]: /ego_odom")

    rospy.Subscriber('/lvx_client_node/gsof/49', NavigationSolution, callback_Applanix)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    applanix_transform_pub = rospy.Publisher('/transforms/LUTM_T_A',Float32MultiArray, queue_size=1)
    velodyne_transform_pub = rospy.Publisher('/transforms/LUTM_T_L',Float32MultiArray, queue_size=1)
    BPF_transform_pub = rospy.Publisher('/transforms/LUTM_T_BPF',Float32MultiArray, queue_size=1)
    applanix_pub = rospy.Publisher('/ego_odom',Odometry, queue_size=1)
    sensors2localUTMpublisher()