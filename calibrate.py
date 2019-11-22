#! /usr/bin/env python
# -*- coding:utf-8 -*-
import numpy as np
import cv2
import glob
import time
from telnet_connect import TelnetClient
'''
print "Connecting to Cognex camera...\n"
telnet_client = TelnetClient()
ip = "192.168.2.99"欧拉角旋转矩阵
username = "admin"
password = ""

if telnet_client.login_host(ip, username, password):
	for i in range(20):
		print("Taking the %dth pic...\n" %(i+1))
		#time.sleep(5)
		telnet_client.execute_command('SE8')
		telnet_client.save_img('calibrate_pics/calibrate'+str(i))
		print("Saving the %d calibrate image...\n" %(i+1))
	print("All calibrate pics acquired.\n")
	telnet_client.logout_host
'''
# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*8,3), np.float32)
objp[:,:2] = np.mgrid[0:8,0:6].T.reshape(-1,2)
objp = objp*29

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob('calibrate_pics/*.bmp')

for fname in images:
    gray = cv2.imread(fname, cv2.IMREAD_GRAYSCALE)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (8,6),None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        gray = cv2.drawChessboardCorners(gray, (8,6), corners2,ret)
        cv2.imshow('gray',gray)
        cv2.waitKey(1000)

cv2.destroyAllWindows()

print("All the points recorded...\n")

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

np.savetxt("Camera Matrix.txt", mtx)
np.savetxt("Camera Distort.txt", dist)
print("Calibration result recorded")

img = cv2.imread('test1.bmp', cv2.IMREAD_GRAYSCALE)
h,  w = img.shape[:2]
newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))

dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

# crop the image
x,y,w,h = roi
dst = dst[y:y+h, x:x+w]
cv2.imwrite('calibresult.png',dst)

tot_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
    tot_error += error

print("total error: ", tot_error/len(objpoints))
print(objp)