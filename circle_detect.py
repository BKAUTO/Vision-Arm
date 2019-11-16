#! /usr/bin/env python
# -*- coding:utf-8 -*-
import cv2
import numpy as np

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

src = cv2.imread('grasp_test.bmp')
cv2.namedWindow('input_image', cv2.WINDOW_NORMAL) #设置为WINDOW_NORMAL可以任意缩放
cv2.imshow('input_image', src)
object_point = detect_circles_demo(src)
print(object_point)