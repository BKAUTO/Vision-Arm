#! /usr/bin/env python
# -*- coding:utf-8 -*-
import cv2
import sys
import socket
import time
import numpy as np
from telnet_connect import TelnetClient
from apply_hand_eye import applyHandEye

def detect_circles_demo(image):
	dst = cv2.pyrMeanShiftFiltering(image, 10, 100)   #边缘保留滤波EPF
	cimage = cv2.cvtColor(dst, cv2.COLOR_RGB2GRAY)
	circles = cv2.HoughCircles(cimage, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=0, maxRadius=0)
	circles = np.uint16(np.around(circles)) #把circles包含的圆心和半径的值变成整数
	for i in circles[0, : ]:
		cv2.circle(image, (i[0], i[1]), i[2], (0, 0, 255), 2)  #画圆
		cv2.circle(image, (i[0], i[1]), 2, (0, 0, 255), 2)  #画圆心
		object_point = [i[0], i[1]]
	cv2.imshow("circles", image)
	cv2.waitKey(5000)
	cv2.destroyAllWindows()
	return object_point

def moveToPose(pose):
	print("Start Moving!!!")
	send_str = str(str(pose[0])+','+str(pose[1])+',')
	conn2.sendall(send_str)
	send_str = str(str(pose[2])+'\r')
	conn2.sendall(send_str)
	time.sleep(8)


if __name__ == '__main__':
	cameraMatrix = np.loadtxt("Camera Matrix.txt")
	R_cam2gripper = np.loadtxt("R_cam2gripper.txt")
	t_cam2gripper = np.loadtxt("t_cam2gripper.txt")

	print(cameraMatrix)
	print(R_cam2gripper)
	print(t_cam2gripper)

	ip_port1 = ('192.168.2.177',8088)
	sk1 = socket.socket()
	sk1.bind(ip_port1)
	sk1.listen(5)
	print("status server create sucessfully!!!")
	while True:
		print('waiting for connection...')
		conn1,addr = sk1.accept()
		print('...connnecting from:', addr)
		break

	ip_port2 = ('192.168.2.177',8888)
	sk2 = socket.socket()
	sk2.bind(ip_port2)
	sk2.listen(5)
	print("motion server create sucessfully!!!")
	while True:
		print('waiting for connection...')
		conn2,addr = sk2.accept()
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
	
	#Move to HomePose
	HomePose = [320.0,0.0,353.0]
	moveToPose(HomePose)
	
	#Take a photo
	telnet_client.execute_command('SE8')
	telnet_client.save_img('grasp_test')

	src = cv2.imread('grasp_test.bmp')
	cv2.namedWindow('input_image', cv2.WINDOW_NORMAL) #设置为WINDOW_NORMAL可以任意缩放
	cv2.imshow('input_image', src)
	object_point = detect_circles_demo(src)
	print(object_point)
	target_point = applyHandEye(object_point, R_cam2gripper, t_cam2gripper, cameraMatrix, conn1)
	print(target_point)
	HomePose = [target_point[0],target_point[1],353.0]
	moveToPose(HomePose)
	telnet_client.logout_host()
	sk1.shutdown(2)
	sk1.close()
	sk2.shutdown(2)
	sk2.close()