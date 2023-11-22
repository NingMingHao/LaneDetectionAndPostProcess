#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov  5 16:51:41 2021

@author: minghao
"""


import numpy as np
from RingroadMap import RingroadMap


class Lane_updater(object):   
    def __init__(self, csv_file = 'RingroadWithCenterlineNotGood.csv', is_xlot=False, fitting_degree=2):
        self.roadmap = RingroadMap(csv_file)
        self.roadmap.Look_ahead_index_for_heading = 4 # look ahead index number to calculate the road heading angle
        self.roadmap.Look_ahead_distance_for_lanes = 24 # meter, look ahead distance of the lanes
        
        # HDMAP
        self.Hdmap_points_in_wc_left_global = np.array([[],[]]).T    ## global coordinate
        self.Hdmap_points_in_wc_right_global = np.array([[],[]]).T
        self.Hdmap_points_in_wc_center_global = np.array([[],[]]).T
        self.Hdmap_points_in_wc_left = np.array([[],[]]).T           ## local coordinate
        self.Hdmap_points_in_wc_right = np.array([[],[]]).T
        self.Hdmap_points_in_wc_center = np.array([[],[]]).T
        self.centerlane_f = None                                     ## fitted function
        self.centerline_f = None
        self.rightcurb_f = None
        self.is_xlot = is_xlot
        self.fitting_degree = fitting_degree

    def global2ego(self, global_xy, ego_pos):
        # ego pose: x:east, y:north
        # rslidar: x:front, y:left
        global_e = global_xy[:,0]
        global_n = global_xy[:,1]
        ego_e = ego_pos[0]
        ego_n = ego_pos[1]
        diff_e = global_e - ego_e
        diff_n = global_n - ego_n
        ego_heading = ego_pos[2]
        cos_ego = np.cos(np.deg2rad(ego_heading))
        sin_ego = np.sin(np.deg2rad(ego_heading))
        local_x = diff_e * cos_ego - diff_n * sin_ego
        local_y = diff_e * sin_ego + diff_n * cos_ego
        # change to rslidar frame:
        rslidar_x = local_y
        rslidar_y = -local_x
        return np.array(np.vstack( (rslidar_x,rslidar_y) ).T)
        
        
    def find_lanes(self, current_pos):
        hdmap_left_curb_en, hdmap_center_line_en, hdmap_right_curb_en = self.roadmap.find_all_lanes(current_pos[0], current_pos[1], current_pos[2])
        
        self.Hdmap_points_in_wc_left_global = hdmap_left_curb_en
        self.Hdmap_points_in_wc_center_global = hdmap_center_line_en
        self.Hdmap_points_in_wc_right_global = hdmap_right_curb_en
        self.Hdmap_points_in_wc_left = self.global2ego(self.Hdmap_points_in_wc_left_global, current_pos)
        self.Hdmap_points_in_wc_center = self.global2ego(self.Hdmap_points_in_wc_center_global, current_pos)
        self.Hdmap_points_in_wc_right = self.global2ego(self.Hdmap_points_in_wc_right_global, current_pos)
        
        # np.polyfit(x, y, deg)
        if self.is_xlot:
            s = [0]
            for i in range(len(self.Hdmap_points_in_wc_center)-1):
                xyt = self.Hdmap_points_in_wc_center[i,0]
                xytp1 = self.Hdmap_points_in_wc_center[i+1,0]
                ds = ((xytp1 - xyt)**2).sum()
                s.append(s[-1]+ds**0.5)
            print(s)
            print(self.Hdmap_points_in_wc_center[:,0])
            print(self.Hdmap_points_in_wc_center[:,1])
            centerline_fitx = np.polyfit(s, self.Hdmap_points_in_wc_center[:,0], self.fitting_degree)
            centerline_fity = np.polyfit(s, self.Hdmap_points_in_wc_center[:,1], self.fitting_degree)
            # print(centerline_fitx)
            centerlane_fitx = centerline_fitx.copy()
            centerlane_fity = centerline_fity.copy()
            
            rightcurb_fitx = centerline_fitx.copy()
            rightcurb_fity = centerline_fity.copy()
            
            centerlane_fity[-1] += 2
            rightcurb_fity[-1] -= 2
            
            self.centerlane_f = [np.poly1d(centerlane_fitx), np.poly1d(centerlane_fity)]
            self.centerline_f = [np.poly1d(centerline_fitx), np.poly1d(centerline_fity)]
            self.rightcurb_f = [np.poly1d(rightcurb_fitx), np.poly1d(rightcurb_fity)]
        else:
            centerlane_fit = np.polyfit(self.Hdmap_points_in_wc_center[:,0], self.Hdmap_points_in_wc_center[:,1], self.fitting_degree)
            centerline_fit = centerlane_fit.copy()
            centerline_fit[-1] -= 2
            rightcurb_fit = np.polyfit(self.Hdmap_points_in_wc_right[:,0], self.Hdmap_points_in_wc_right[:,1], self.fitting_degree)
            self.centerlane_f = np.poly1d(centerlane_fit)
            self.centerline_f = np.poly1d(centerline_fit)
            self.rightcurb_f = np.poly1d(rightcurb_fit)
        
        
    def update(self, current_pos):
        self.find_lanes(current_pos)
        