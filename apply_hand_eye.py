#! /usr/bin/env python
# -*- coding:utf-8 -*-
import sys, os
import numpy as np
import math

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

def applyHandEye(point, R_cam2gripper, t_cam2gripper, cameraMatrix):
	gripper_trans = np.zeros((3,1))
	gripper_orient = np.zeros((3,1))
	gripper_pose = receive_pose(1024)
	gripper_trans[0] = float(gripper_pose[0])
	gripper_trans[1] = float(gripper_pose[1])
	gripper_trans[2] = float(gripper_pose[2])
	gripper_orient[0] = float(gripper_pose[3])/180.0*math.pi
	gripper_orient[1] = float(gripper_pose[4])/180.0*math.pi
	gripper_orient[2] = float(gripper_pose[5])/180.0*math.pi
	rota = np.zeros((3,3))
	point = np.array(point)
	point = point.reshape(-1,1)
	point = np.vstack((point,1))
	#print(point)
	#print(np.linalg.inv(cameraMatrix))
	cam_point = np.dot(np.linalg.inv(cameraMatrix), point)
	cam_point[0] = cam_point[0]*1000
	cam_point[1] = cam_point[1]*1000
	cam_point[2] = gripper_trans[2]+t_cam2gripper[2]
	cam_point = np.vstack((cam_point,1))
	t_cam2gripper = t_cam2gripper.reshape(-1,1)
	cam2gripper = np.hstack((R_cam2gripper,t_cam2gripper))
	cam2gripper = np.vstack((cam2gripper,[0,0,0,1]))
	pose = np.dot(cam2gripper, cam_point)
	
	rota = eulerAnglesToRotationMatrix(gripper_orient)
	gripper2base = np.hstack((rota,gripper_trans))
	gripper2base = np.vstack((gripper2base,[0,0,0,1]))
	world_pose = np.dot(gripper2base, pose)
	world_pose = world_pose[:-1]
	print(cam_point)
	print(pose)
	print(world_pose)
	return world_pose

if __name__ == '__main__':
	ip_port = ('192.168.2.177',8088)
	sk = socket.socket()
	sk.bind(ip_port)
	sk.listen(5)
	print("create server sucessfully!!!")
	cameraMatrix = np.loadtxt("Camera Matrix.txt")
	R_cam2gripper = np.loadtxt("R_cam2gripper.txt")
	t_cam2gripper = np.loadtxt("t_cam2gripper.txt")
	point = [400,400]
	applyHandEye(point, R_cam2gripper, t_cam2gripper, cameraMatrix)


