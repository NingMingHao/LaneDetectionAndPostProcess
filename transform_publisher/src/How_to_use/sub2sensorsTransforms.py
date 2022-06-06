#!/usr/bin/env python
''' 
    Purpose: Obtain sensors transforms - transform is from the sensor frame to the local Ring Road reference frame 
    Subscribed topics: /transforms/LUTM_T_A /transforms/LUTM_T_L

    Project: WATonoBus
    Author: Neel Bhatt
'''

import rospy
import numpy as np
from math import sin,cos,pi
from std_msgs.msg import Float32MultiArray

np.set_printoptions(suppress=True,precision=2)

# Obtain UTM_T_A
def callback_transform_applanix(raw_transform):
    global UTM_T_A
    UTM_T_A = np.array(raw_transform.data).reshape(4,4)
    # print("-------------- UTM_T_A --------------")
    # print(UTM_T_A)

# Obtain UTM_T_L
def callback_transform_velodyne(raw_transform):
    global UTM_T_L
    UTM_T_L = np.array(raw_transform.data).reshape(4,4)
    # print("-------------- UTM_T_L --------------")
    # print(UTM_T_L)

# Obtain UTM_T_R
def callback_transform_BPF(raw_transform):
    global UTM_T_R
    UTM_T_R = np.array(raw_transform.data).reshape(4,4)
    # print("-------------- UTM_T_R --------------")
    # print(UTM_T_R)

def sub2sensorTransforms():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('sub2sensorTransforms', anonymous=True)

    rospy.Subscriber('/transforms/LUTM_T_A',Float32MultiArray, callback_transform_applanix)
    rospy.Subscriber('/transforms/LUTM_T_L',Float32MultiArray, callback_transform_velodyne)
    rospy.Subscriber('/transforms/LUTM_T_BPF',Float32MultiArray, callback_transform_BPF)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    sub2sensorTransforms()