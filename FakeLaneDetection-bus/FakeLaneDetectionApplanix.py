#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov  5 17:22:10 2021

@author: minghao
"""


import sys
import numpy as np


###ROS Things
import rospy
from sensor_msgs.msg import CompressedImage
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from lane_detection_msgs.msg import LaneDetectionMsgFull
from std_msgs.msg import ColorRGBA
import os
from std_msgs.msg import Float32MultiArray, Float32
from transform_publisher.msg import Floats

cur_working_dir = os.path.dirname(os.path.realpath(__file__))
#cur_working_dir = os.getcwd()

### RingroadMap
sys.path.append(os.path.join(cur_working_dir, 'ringroadmap'))
# print(sys.path)
from LaneUpdaterWC import Lane_updater
from ApplanixToLidar import matrix2lidar

def rot2bus(x, y, rot_heading_deg=-3, half_length=2):
    if np.all(x==0) or (rot_heading_deg==0 and half_length==0):
        return x, y, [0,0,0]
    else:
        rot_heading = np.deg2rad(rot_heading_deg)
        cos_head = np.cos(rot_heading)
        sin_head = np.sin(rot_heading)
        rot_matrix = np.array( [ [cos_head, sin_head], [-sin_head, cos_head] ] )
        xy = np.vstack([x,y]).T
        rot_xy = xy @ rot_matrix
        rot_x = rot_xy[:,0]+half_length
        rot_y = rot_xy[:,1]
        new_fit = np.polyfit(rot_x, rot_y, 2)
        poly_f = np.poly1d(new_fit)
        new_x = np.linspace(0, 20 , 21)
        new_y = poly_f(new_x)
        return new_x, new_y, new_fit

def draw_one_line_strip(x_wc_to_draw, y_wc_to_draw, rgba=(1.0,0,0,1), lane_id=0, is_line_strip = True, color_weight=[]):
    lane_line_strip = Marker()
    lane_line_strip.header.frame_id = 'rslidar_front'
    lane_line_strip.action = Marker.ADD
    lane_line_strip.pose.orientation.w = 1
    lane_line_strip.id = lane_id
    if is_line_strip:
        lane_line_strip.type = Marker.LINE_STRIP
    else:
        lane_line_strip.type = Marker.POINTS
    lane_line_strip.scale.x = 0.2
    lane_line_strip.scale.y = 0.2
    lane_line_strip.scale.z = 0.2
    lane_line_strip.color.r = rgba[0]
    lane_line_strip.color.g = rgba[1]
    lane_line_strip.color.b = rgba[2]
    lane_line_strip.color.a = rgba[3]
    
    for i in range( len(y_wc_to_draw) ):
        i_point = Point()
        i_wcx = x_wc_to_draw[i]
        i_wcy = y_wc_to_draw[i]
        i_point.y = i_wcy
        i_point.x = i_wcx
        i_point.z = -1
        lane_line_strip.points.append(i_point)
    
        if len(color_weight):
            i_color = ColorRGBA()
            i_color.r = rgba[0] * color_weight[i]
            i_color.g = rgba[1] * color_weight[i]
            i_color.b = rgba[2] * color_weight[i]
            i_color.a = rgba[3]
            lane_line_strip.colors.append(i_color)
    return lane_line_strip
    

def draw_visualization_and_pub(updater):      
    # publish partial lanes in applanix frame
    rot_heading_deg = 1.5
    half_length = 3.2 

    ret_lane_detection_msg = LaneDetectionMsgFull()
    x_wc_to_draw = np.linspace(0, 20, 21)
    if updater.centerlane_f is not None:
        y_wc_to_draw = updater.centerlane_f(x_wc_to_draw)
        rot_x_centerlane, rot_y_centerlane, rot_fit_centerlane = rot2bus(x_wc_to_draw, y_wc_to_draw, rot_heading_deg=rot_heading_deg, half_length=half_length)
        ret_lane_detection_msg.CenterLaneX = rot_x_centerlane
        ret_lane_detection_msg.CenterLaneY = rot_y_centerlane
        ret_lane_detection_msg.PolyFitC2 = rot_fit_centerlane[0]
        ret_lane_detection_msg.PolyFitC1 = rot_fit_centerlane[1]
        ret_lane_detection_msg.PolyFitC0 = rot_fit_centerlane[2]
        ret_lane_detection_msg.LateralDis = rot_fit_centerlane[2]
        ret_lane_detection_msg.Confidence = 1
        centerlane_msg = draw_one_line_strip(rot_y_centerlane, rot_x_centerlane, rgba=(1,0,0,1), lane_id=3, is_line_strip=True)
        centerlane_msg.header.frame_id = 'applanix'
        lane_marker_pub.publish(centerlane_msg)
        
    # if updater.centerline_f is not None:
    #     y_wc_to_draw = updater.centerline_f(x_wc_to_draw)
    #     ret_lane_detection_msg.CenterLineX = x_wc_to_draw
    #     ret_lane_detection_msg.CenterLineY = y_wc_to_draw
    #     centerline_msg = draw_one_line_strip(x_wc_to_draw, y_wc_to_draw, rgba=(1,1,0,1), lane_id=4, is_line_strip=True)
    #     lane_marker_pub.publish(centerline_msg)
        
    # if updater.rightcurb_f is not None:
    #     y_wc_to_draw = updater.rightcurb_f(x_wc_to_draw)
    #     ret_lane_detection_msg.RightCurbX = x_wc_to_draw
    #     ret_lane_detection_msg.RightCurbY = y_wc_to_draw
    #     rightcurb_msg = draw_one_line_strip(x_wc_to_draw, y_wc_to_draw, rgba=(0,1,0,1), lane_id=5, is_line_strip=True)
    #     lane_marker_pub.publish(rightcurb_msg)
    
    fake_lane_detection_pub.publish(ret_lane_detection_msg)


def draw_visualization_and_pub_s(updater):      
    # publish partial lanes in rslidar frame
    ret_lane_detection_msg = LaneDetectionMsgFull()
    s_x_wc_to_draw = np.linspace(0, 20, 21)
    
    if updater.centerlane_f is not None:
        x_wc_to_draw = updater.centerlane_f[0](s_x_wc_to_draw)
        y_wc_to_draw = updater.centerlane_f[1](s_x_wc_to_draw)
        ret_lane_detection_msg.CenterLaneX = x_wc_to_draw
        ret_lane_detection_msg.CenterLaneY = y_wc_to_draw
        lane_fit_parameters = updater.centerlane_f.c
        ret_lane_detection_msg.PolyFitC2 = lane_fit_parameters[0]
        ret_lane_detection_msg.PolyFitC1 = lane_fit_parameters[1]
        ret_lane_detection_msg.PolyFitC0 = lane_fit_parameters[2]
        ret_lane_detection_msg.LateralDis = rot_fit_centerlane[2] + rot_fit_centerlane[1]*4 + rot_fit_centerlane[0]*4**2
        ret_lane_detection_msg.Confidence = 1
        centerlane_msg = draw_one_line_strip(x_wc_to_draw, y_wc_to_draw, rgba=(1,0,0,1), lane_id=3, is_line_strip=True)
        lane_marker_pub.publish(centerlane_msg)
    
    fake_lane_detection_pub.publish(ret_lane_detection_msg)
    

def callBackPublishMap(time_msg):
    # publish whole lanes in map frame
    x_wc_to_draw = updater.roadmap.center_line_east
    y_wc_to_draw = updater.roadmap.center_line_north
    center_lane_msg = draw_one_line_strip(x_wc_to_draw, y_wc_to_draw, rgba=(1,0,0,1), lane_id=7, is_line_strip=False)
    center_lane_msg.header.frame_id = "map"
    lane_marker_pub.publish(center_lane_msg)

def img_callback(img_msg):
    global UTM_T_L    
    if UTM_T_L is not None:
        x_lutm, y_lutm, heading_lutm = matrix2lidar(UTM_T_L)# heading:deg
        current_pos = (x_lutm, y_lutm, heading_lutm)
        updater.update(current_pos)  # update current location, and get lanes from CSV file
        # Draw visualization things and publish them
        if is_xlot:
            draw_visualization_and_pub_s(updater)
        else:
            draw_visualization_and_pub(updater)
            
def callback_transform_velodyne(raw_transform):
    global UTM_T_L
    UTM_T_L = np.array(raw_transform.data).reshape(4,4)

if __name__ == "__main__":
    #### Start Ros Communication:
    rospy.init_node("Fake_Lane_Detection_Node")
    print("initial fake detection node")
    
    is_xlot = False
    args = sys.argv
    if len(args) > 1:
        xlot_arg = eval(args[1])
        print('arg is ', xlot_arg)
        if xlot_arg == True:
            is_xlot = True
            
    #is_xlot = True
    # updater = Lane_updater( csv_file = './tracker/oct4_rr_latest.csv')
    if is_xlot:
        print('Test on X lot')
        updater = Lane_updater( csv_file = './ringroadmap/parkinglot_Dec1.csv', is_xlot=True, fitting_degree=6)
    else:
        print('Test on Ringroad')
        updater = Lane_updater( csv_file = './ringroadmap/oct4_rr_latest_Jan17.csv', fitting_degree=2)
    
    UTM_T_L = None
    IS_OLD_BAG = False
    
    if IS_OLD_BAG:
        rospy.Subscriber('/transforms/LUTM_T_L',Floats, callback_transform_velodyne)
    else:
        rospy.Subscriber('/transforms/LUTM_T_L',Float32MultiArray, callback_transform_velodyne)
    rospy.Timer(rospy.Duration(0.1), callBackPublishMap)
    img_sub = rospy.Subscriber("/pylon_camera_node_center/image_rect/compressed", CompressedImage, img_callback, queue_size=1, buff_size=2**30)
    fake_lane_detection_pub = rospy.Publisher("/Lane_Detection_Result_Fake", LaneDetectionMsgFull, queue_size=10)
    lane_marker_pub = rospy.Publisher('/lane_marker_fake', Marker, queue_size=10)
    rospy.spin()
    
