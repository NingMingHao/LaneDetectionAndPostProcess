#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov 25 12:43:16 2021

@author: minghao
"""


import numpy as np
from math import sin,cos,pi
import pyproj
from tf.transformations import euler_from_matrix
utm_proj = pyproj.Proj(proj='utm', zone=17, datum='WGS84')
ref_easting = 537132
ref_northing = 4813391


def matrix2lidar(UTM_T_L):
    translation_x_velodyne = UTM_T_L[0,3]
    translation_y_velodyne = UTM_T_L[1,3]
    translation_z_velodyne = UTM_T_L[2,3]

    roll_alpha_velodyne, pitch_beta_velodyne, yaw_gamma_velodyne = euler_from_matrix(UTM_T_L[0:3,0:3], 'sxyz')
    return (translation_x_velodyne, translation_y_velodyne, 90-np.rad2deg(yaw_gamma_velodyne))
    # return (translation_x_velodyne, translation_y_velodyne, 90-1.571-np.rad2deg(yaw_gamma_velodyne))
