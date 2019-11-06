#! /usr/bin/env python
# -*- coding:utf-8 -*-
import os,sys
import numpy as np
import cv2
import time
import glob
import math
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from telnet_connect import TelnetClient

def eulerAnglesToRotationMatrix(theta) :
	R_x = np.array([[1, 0, 0],
					[0, math.cos(theta[0]), -math.sin(theta[0])],
					[0,	math.sin(theta[0]), math.cos(theta[0])]
					])
	R_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])],
					[0, 1, 0],
					[-math.sin(theta[1]), 0, math.cos(theta[1])]
					])
	R_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
					[math.sin(theta[2]), math.cos(theta[2]), 0],
					[0, 0, 1]
					])
	R = np.dot(R_z, np.dot( R_y, R_x ))
	return R

print "MoveIt should be launched!!!\n"
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('hand_eye_calibration', anonymous=True)
robot = moveit_commander.RobotCommander()
group_name = "robot"
move_group = moveit_commander.MoveGroupCommander(group_name)


print "============ Printing robot state"
print robot.get_current_state()
print ""

print "Connecting to Cognex camera...\n"
telnet_client = TelnetClient()
ip = "192.168.2.99"
username = "admin"
password = ""

if telnet_client.login_host(ip, username, password):
	print "Connection to camera built..."
else:
	print "Connection to camera faied..."
	sys.exit(0)

R_gripper2base = []
t_gripper2base = []
R_target2cam = []
t_target2cam = []

trans = np.zeros((3,1))
rota = np.zeros((3,3))

default_orientation = np.array([-math.pi, 0, math.pi])

for i in range(4):
	print ("Please move to the %dth pose" %(i+1))
	print "Press t to take a photo"
	while(1):
		flag = raw_input()
		if flag == 't':
			current_pose = move_group.get_current_pose().pose
			trans[0] = current_pose.position.x
			trans[1] = current_pose.position.y
			trans[2] = current_pose.position.z
			#print(trans)
			t_gripper2base.append(trans)
			rota = eulerAnglesToRotationMatrix(default_orientation)
			#print(rota)
			R_gripper2base.append(rota)
			telnet_client.execute_command('SE8')
			telnet_client.save_img('handeye_pics/handeye'+str(i))
			break
telnet_client.logout_host

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

cameraMatrix = np.loadtxt("Camera Matrix.txt")
distCoeffs = np.loadtxt("Camera Distort.txt")

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*8,3), np.float32)
objp[:,:2] = np.mgrid[0:8,0:6].T.reshape(-1,2)

for i in range(4):
	gray = cv2.imread('calibrate_pics/calibrate'+str(i)+'.bmp', cv2.IMREAD_GRAYSCALE)
	objpoints = [] # 3d point in real world space
	imgpoints = [] # 2d points in image plane.
	ret, corners = cv2.findChessboardCorners(gray, (8,6),None)
	if ret == True:
		objpoints.append(objp)

		corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
		corners2 = np.squeeze(corners2)
		imgpoints.append(corners2)
		objpoints = np.array(objpoints)
		imgpoints = np.array(imgpoints)
		retval, rvec, tvec = cv2.solvePnP(objpoints, imgpoints, cameraMatrix, distCoeffs)
		print(rvec)
		rota, jacobian = cv2.Rodrigues(rvec)
		print(rota)

		gray = cv2.drawChessboardCorners(gray, (8,6), corners2,ret)
        cv2.imshow('gray',gray)
        cv2.waitKey(5000)

cv2.destroyAllWindows()

print("Preparing to calculate the hand-eye matrix...\n")

