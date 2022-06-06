#!/usr/bin/env python
''' 
    Purpose: Provide the (1) Rotation matrix generating function based on roll,pitch, and yaw angles about fixed axes and
     					 (2) the transform from LIDAR to Applanix

    Project: WATonoBus
    Author: Neel Bhatt
'''

import numpy as np
from math import sin,cos,pi
import geodesy.utm
import getpass

def rot_matrix(roll,pitch,yaw):
	''' Compute Rotation Matrix from {B} to {A} = A_R_B given RPY angles using {A} as fixed axis about which RPY of {B} is given:
		Roll is about x axis, Pitch about y axis, and Yaw about z axis.
		
		Inputs: Roll, pitch, and yaw angles in degrees
		Outputs: A_R_B (3x3)
	'''

	alpha = yaw*pi/180; beta = pitch*pi/180; gamma = roll*pi/180
	Rz = np.array([[cos(alpha), -sin(alpha), 0],[sin(alpha),cos(alpha),0],[0,0,1]])
	Ry = np.array([[cos(beta), 0, sin(beta)],[0,1,0],[-sin(beta),0,cos(beta)]])
	Rx = np.array([[1,0,0],[0,cos(gamma),-sin(gamma)],[0,sin(gamma),cos(gamma)]])
	A_R_B = np.matmul(Rz,np.matmul(Ry,Rx))
	
	# Alternative - To check with final form of principal rotation matrix multiplication
	# R_XYZ = np.array([[cos(alpha)*cos(beta),cos(alpha)*sin(beta)*sin(gamma)-sin(alpha)*cos(gamma),cos(alpha)*sin(beta)*cos(gamma)+sin(alpha)*sin(gamma)],
	# 	[sin(alpha)*cos(beta),sin(alpha)*sin(beta)*sin(gamma)+cos(alpha)*cos(gamma),sin(alpha)*sin(beta)*cos(gamma)-cos(alpha)*sin(gamma)],
	# 	[-sin(beta),cos(beta)*sin(gamma),cos(beta)*cos(gamma)]])
	# print("\nR_XYZ:\n",R_XYZ)
	
	return A_R_B

# Load Transformation Matrix from Lidar to Applanix
# A_T_L = np.load("/home/minghao/Documents/UWaterloo/Rosmsgs/src/transform_publisher/transforms/Applanix/transformation_matrix_A_T_L.npy")
A_T_L = np.load("/home/minghao/Documents/UWaterloo/Rosmsgs/src/transform_publisher/transforms/Applanix/transformation_matrix_A_T_L_jan17.npy")
L_T_BPF = np.load("/home/minghao/Documents/UWaterloo/Rosmsgs/src/transform_publisher/transforms/BP_lidars/transformation_matrix_L_T_BPF.npy")
