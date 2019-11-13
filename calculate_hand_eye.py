#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import os,sys
import numpy as np
import cv2

R_gripper2base = np.loadtxt("R_gripper2base.txt")
t_gripper2base = np.loadtxt("t_gripper2base.txt")
R_target2cam = np.loadtxt("R_target2cam.txt")
t_target2cam = np.loadtxt("t_target2cam.txt")
R_gripper2base = np.reshape(R_gripper2base, (4,3,3))
#print(R_gripper2base)
R_target2cam = np.reshape(R_target2cam, (4,3,3))

R_cam2gripper, t_cam2gipper = cv2.calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam)
print(R_cam2gripper)
print(t_cam2gipper)
np.savetxt("R_cam2gripper.txt", R_cam2gripper)
np.savetxt("t_cam2gripper.txt", t_cam2gipper)