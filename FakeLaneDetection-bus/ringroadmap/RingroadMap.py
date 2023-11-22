#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Nov 17 22:08:38 2021

@author: minghao
"""

import pandas as pd
import numpy as np


class RingroadMap(object):
    def __init__(self, csv_file = 'RingroadWithCenterlineNotGood.csv'):
        self.df = pd.read_csv(csv_file, skiprows=(0,1))
        self.right_curb_east = np.array(self.df['Right_curb_east'])
        self.right_curb_north = np.array(self.df['Right_curb_north'])
        self.left_curb_east = np.array(self.df['Left_curb_east'])
        self.left_curb_north = np.array(self.df['Left_curb_north'])
        self.center_line_east = np.array(self.df['Center_line_east'])
        self.center_line_north = np.array(self.df['Center_line_north'])
        self.ref_path_east = np.array(self.df['East_P'])
        self.ref_path_north = np.array(self.df['North_P'])
        self.max_index = len(self.ref_path_east)
        if 's_coord' in self.df.columns:
            s_coord_name = 's_coord' 
        else:
            s_coord_name = 'Road Location'
        self.s_coord = np.array(self.df[s_coord_name])
        self.Min_dist_to_accept = 10 # meter, minial distance to accept the current ego pose
        self.Look_ahead_index_for_heading = 4 # look ahead index number to calculate the road heading angle
        self.Look_ahead_distance_for_lanes = 25 # meter, look ahead distance of the lanes
        self.Max_angle_to_init_heading = 60 # degree
        self.is_along_ref_path = None
        self.current_index = None
    

    def find_all_lanes(self, ego_east, ego_north, ego_heading): 
        ##LUTM X, Y, Heading (rad); Heading: East 0 deg, North 90 deg
        self.ego_east = ego_east
        self.ego_north = ego_north
        self.ego_heading = ego_heading
        
        # if self.is_along_ref_path is None:
        self.init_heading_and_index() # Calculate current index, and judge the direction of path following
        if self.is_along_ref_path is None:
            return np.array([[],[]]).T, np.array([[],[]]).T, np.array([[],[]]).T
        elif self.is_along_ref_path == True:
            left_curb_east = self.left_curb_east
            left_curb_north = self.left_curb_north
            center_line_east = self.center_line_east
            center_line_north = self.center_line_north
            right_curb_east = self.right_curb_east
            right_curb_north = self.right_curb_north
            direction_sign = 1
        else:
            left_curb_east = self.right_curb_east
            left_curb_north = self.right_curb_north
            center_line_east = self.center_line_east
            center_line_north = self.center_line_north
            right_curb_east = self.left_curb_east
            right_curb_north = self.left_curb_north
            direction_sign = -1
            
        look_ahead_index = self.calc_look_ahead_index()
        ret_inds = (np.array( range(look_ahead_index + 1) ) * direction_sign + self.current_index) % self.max_index
        ret_left_curb_en = np.vstack( (left_curb_east[ret_inds], left_curb_north[ret_inds]) ).T
        ret_center_line_en = np.vstack( (center_line_east[ret_inds], center_line_north[ret_inds]) ).T
        ret_right_curb_en = np.vstack( (right_curb_east[ret_inds], right_curb_north[ret_inds]) ).T
        return ret_left_curb_en, ret_center_line_en, ret_right_curb_en
    
    
    def calc_look_ahead_index(self):
        cum_distance = 0
        look_ahead_index = 0
        if self.is_along_ref_path:
            direction_sign = 1
        else:
            direction_sign = -1
        while cum_distance < self.Look_ahead_distance_for_lanes:
            i_diff_east = self.ref_path_east[ (self.current_index + (look_ahead_index + 1) * direction_sign) % self.max_index ] - self.ref_path_east[ (self.current_index + look_ahead_index * direction_sign ) % self.max_index ]
            i_diff_north = self.ref_path_north[ (self.current_index + (look_ahead_index + 1) * direction_sign) % self.max_index ] - self.ref_path_north[ (self.current_index + look_ahead_index * direction_sign) % self.max_index ]
            i_distance = (i_diff_east**2 + i_diff_north**2)**0.5
            cum_distance += i_distance
            look_ahead_index += 1
        return look_ahead_index
        
            
    def find_nearest_index(self):
        diff_e = self.ref_path_east - self.ego_east
        diff_n = self.ref_path_north - self.ego_north
        diff_dist = (diff_e**2 + diff_n**2)**0.5
        min_index = np.argmin(diff_dist)
        min_dist = diff_dist[min_index]
        if min_dist < self.Min_dist_to_accept:
            self.current_index = min_index
        else:
            self.current_index = None
        
    
    def init_heading_and_index(self):
        self.find_nearest_index()
        if self.current_index is not None:
            ego_ind = self.current_index
            next_ind = (ego_ind + self.Look_ahead_index_for_heading)%self.max_index
            cur_ref_east = self.ref_path_east[ego_ind]
            cur_ref_north = self.ref_path_north[ego_ind]
            next_ref_east = self.ref_path_east[next_ind]
            next_ref_north = self.ref_path_north[next_ind]
            
            diff_e = next_ref_east - cur_ref_east
            diff_n = next_ref_north - cur_ref_north
            
            ref_path_heading = np.rad2deg(np.arctan2(diff_e, diff_n))
            ego_heading = self.ego_heading
            diff_angle = ref_path_heading - ego_heading
            diff_angle = diff_angle%360
            diff_angle = min(diff_angle, 360-diff_angle)
            if diff_angle < self.Max_angle_to_init_heading:
                self.is_along_ref_path = True
            elif diff_angle > 180 - self.Max_angle_to_init_heading:
                self.is_along_ref_path = False
            else:
                self.is_along_ref_path = None
                self.current_index = None ### Set index to None to show this is a false initialization
                
            
            

                
        
        
        
        