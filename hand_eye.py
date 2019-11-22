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
import socket
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from telnet_connect import TelnetClient

def receive_pose(buffersize):
	send_str = 'cart_pos\r'
	conn.sendall(send_str.encode())
	raw_data = conn.recv(buffersize)
	raw_data = raw_data.split(',' , 9)
	return raw_data

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

if __name__ == '__main__':
	ip_port = ('192.168.2.177',8088)
	sk = socket.socket()
	sk.bind(ip_port)
	sk.listen(5)
	print("create server sucessfully!!!")
	while True:
		print('waiting for connection...')
		conn,addr = sk.accept()
		print('...connnecting from:', addr)
		break

	print("Connecting to Cognex camera...\n")
	telnet_client = TelnetClient()
	ip = "192.168.2.99"
	username = "admin"
	password = ""

	if telnet_client.login_host(ip, username, password):
		print("Connection to camera built...")
	else:
		print("Connection to camera faied...")
		sys.exit(0)

	R_gripper2base = []
	t_gripper2base = []
	R_target2cam = []
	t_target2cam = []

	trans = np.zeros((3,1))
	rota = np.zeros((3,3))

	default_orientation = np.array([-math.pi, 0, math.pi])

	for i in range(8):
		print ("Please move to the %dth pose" %(i+1))
		print ("Press t to take a photo")
		while(1):
			flag = raw_input()
			if flag == 't':
				#current_pose = move_group.get_current_pose().pose
				#current_angle = move_group.get_current_rpy()
				pose_data = receive_pose(1024)
				trans[0] = float(pose_data[0])
				trans[1] = float(pose_data[1])
				trans[2] = float(pose_data[2])
				print(trans)
				default_orientation[0] = float(pose_data[3])/180.0*math.pi
				default_orientation[1] = float(pose_data[4])/180.0*math.pi
				default_orientation[2] = float(pose_data[5])/180.0*math.pi
				print(default_orientation)
				t_gripper2base.append(trans)
				rota = eulerAnglesToRotationMatrix(default_orientation)
				#print(rota)
				R_gripper2base.append(rota)
				telnet_client.execute_command('SE8')
				telnet_client.save_img('handeye_pics/handeye'+str(i))
				break
	telnet_client.logout_host()
	sk.shutdown(2)
	sk.close()

	# termination criteria
	criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

	cameraMatrix = np.loadtxt("Camera Matrix.txt")
	distCoeffs = np.loadtxt("Camera Distort.txt")

	# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
	objp = np.zeros((6*8,3), np.float32)
	objp[:,:2] = np.mgrid[0:8,0:6].T.reshape(-1,2)
	objp = objp*29

	for i in range(8):
		gray = cv2.imread('handeye_pics/handeye'+str(i)+'.bmp', cv2.IMREAD_GRAYSCALE)
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
			#print(rvec)
			rota, jacobian = cv2.Rodrigues(rvec)
			#print(rota)
			t_target2cam.append(tvec)
			R_target2cam.append(rota)

			gray = cv2.drawChessboardCorners(gray, (8,6), corners2,ret)
			cv2.imshow('gray',gray)
			cv2.waitKey(500)

	cv2.destroyAllWindows()

	print("Preparing to calculate the hand-eye matrix...\n")

	t_gripper2base = np.array(t_gripper2base)
	R_gripper2base = np.array(R_gripper2base)
	#print(R_gripper2base)
	print(R_gripper2base.shape)
	t_target2cam = np.array(t_target2cam)
	R_target2cam = np.array(R_target2cam)
	#print(R_target2cam.shape)
	np.savetxt("t_gripper2base.txt", t_gripper2base)
	np.savetxt("R_gripper2base.txt", np.reshape(R_gripper2base,(2,36)))
	np.savetxt("t_target2cam.txt", t_target2cam)
	np.savetxt("R_target2cam.txt", np.reshape(R_target2cam,(2,36)))

	'''
	R_cam2gripper, t_cam2gipper = cv2.calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam)
	print(R_cam2gripper)
	np.savetxt("R_cam2gripper.txt", R_cam2gripper)
	np.savetxt("t_cam2gripper.txt", t_cam2gipper)
	'''