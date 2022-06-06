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
from lane_detection_msgs.msg import LaneDetectionMsg
from std_msgs.msg import ColorRGBA
import os
from std_msgs.msg import Float32MultiArray, Float32
from transform_publisher.msg import Floats

# cur_working_dir = os.path.dirname(os.path.realpath(__file__))
cur_working_dir = os.getcwd()

### RingroadMap
sys.path.append(os.path.join(cur_working_dir, 'ringroadmap'))
# print(sys.path)
from LaneUpdaterWC import Lane_updater
from ApplanixToLidar import matrix2lidar

    
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
    lane_line_strip.scale.x = 0.1
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
    # publish partial lanes in rslidar frame
    ret_lane_detection_msg = LaneDetectionMsg()
    x_wc_to_draw = np.linspace(0, 20, 21)
    # if updater.centerlane_f is not None:
    #     y_wc_to_draw = updater.centerlane_f(x_wc_to_draw)
    #     ret_lane_detection_msg.CenterLaneX = x_wc_to_draw
    #     ret_lane_detection_msg.CenterLaneY = y_wc_to_draw
    #     centerlane_msg = draw_one_line_strip(x_wc_to_draw, y_wc_to_draw, rgba=(1,0,0,1), lane_id=3, is_line_strip=True)
    #     lane_marker_pub.publish(centerlane_msg)
        
    if updater.centerline_f is not None:
        y_wc_to_draw = updater.centerline_f(x_wc_to_draw)
        ret_lane_detection_msg.CenterLineX = x_wc_to_draw
        ret_lane_detection_msg.CenterLineY = y_wc_to_draw
        centerline_msg = draw_one_line_strip(x_wc_to_draw, y_wc_to_draw, rgba=(1,1,0,1), lane_id=4, is_line_strip=True)
        lane_marker_pub.publish(centerline_msg)
        
    # if updater.rightcurb_f is not None:
    #     y_wc_to_draw = updater.rightcurb_f(x_wc_to_draw)
    #     ret_lane_detection_msg.RightCurbX = x_wc_to_draw
    #     ret_lane_detection_msg.RightCurbY = y_wc_to_draw
    #     rightcurb_msg = draw_one_line_strip(x_wc_to_draw, y_wc_to_draw, rgba=(0,1,0,1), lane_id=5, is_line_strip=True)
    #     lane_marker_pub.publish(rightcurb_msg)
    
    fake_lane_detection_pub.publish(ret_lane_detection_msg)


def draw_visualization_and_pub_s(updater):      
    # publish partial lanes in rslidar frame
    ret_lane_detection_msg = LaneDetectionMsg()
    s_x_wc_to_draw = np.linspace(0, 20, 21)
    
    # if updater.centerlane_f is not None:
    #     x_wc_to_draw = updater.centerlane_f[0](s_x_wc_to_draw)
    #     y_wc_to_draw = updater.centerlane_f[1](s_x_wc_to_draw)
    #     ret_lane_detection_msg.CenterLaneX = x_wc_to_draw
    #     ret_lane_detection_msg.CenterLaneY = y_wc_to_draw
    #     centerlane_msg = draw_one_line_strip(x_wc_to_draw, y_wc_to_draw, rgba=(1,0,0,1), lane_id=3, is_line_strip=True)
    #     lane_marker_pub.publish(centerlane_msg)
        
    if updater.centerline_f is not None:
        x_wc_to_draw = updater.centerline_f[0](s_x_wc_to_draw)
        y_wc_to_draw = updater.centerline_f[1](s_x_wc_to_draw)
        ret_lane_detection_msg.CenterLineX = x_wc_to_draw
        ret_lane_detection_msg.CenterLineY = y_wc_to_draw
        centerline_msg = draw_one_line_strip(x_wc_to_draw, y_wc_to_draw, rgba=(1,1,0,1), lane_id=4, is_line_strip=True)
        lane_marker_pub.publish(centerline_msg)
        
    # if updater.rightcurb_f is not None:
    #     x_wc_to_draw = updater.rightcurb_f[0](s_x_wc_to_draw)
    #     y_wc_to_draw = updater.rightcurb_f[1](s_x_wc_to_draw)
    #     ret_lane_detection_msg.RightCurbX = x_wc_to_draw
    #     ret_lane_detection_msg.RightCurbY = y_wc_to_draw
    #     rightcurb_msg = draw_one_line_strip(x_wc_to_draw, y_wc_to_draw, rgba=(0,1,0,1), lane_id=5, is_line_strip=True)
    #     lane_marker_pub.publish(rightcurb_msg)
    
    fake_lane_detection_pub.publish(ret_lane_detection_msg)
    
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
        updater = Lane_updater( csv_file = './ringroadmap/parkinglot_Dec1.csv', is_xlot=True, fitting_degree=12)
    else:
        print('Test on Ringroad')
        updater = Lane_updater( csv_file = './ringroadmap/oct4_rr_latest_Jan17.csv')
    
    UTM_T_L = None
    IS_OLD_BAG = False
    
    if IS_OLD_BAG:
        rospy.Subscriber('/transforms/LUTM_T_L',Floats, callback_transform_velodyne)
    else:
        rospy.Subscriber('/transforms/LUTM_T_L',Float32MultiArray, callback_transform_velodyne)
    
    img_sub = rospy.Subscriber("/pylon_camera_node_center/image_rect/compressed", CompressedImage, img_callback, queue_size=1, buff_size=2**30)
    fake_lane_detection_pub = rospy.Publisher("/Lane_Detection_Result", LaneDetectionMsg, queue_size=10)
    lane_marker_pub = rospy.Publisher('/lane_marker', Marker, queue_size=10)
    rospy.spin()
    
