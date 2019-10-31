#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri Jan  5 10:57:34 2018
@author: 晚晴风
"""
import cv2 
import numpy as np

width = 450
height = 350
length = 50

image = np.zeros((width,height),dtype = np.uint8)
print(image.shape[0],image.shape[1])

for j in range(height):
    for i in range(width):
        if((int)(i/length) + (int)(j/length))%2:
            image[i,j] = 255;
cv2.imwrite("chessboard.png",image)
cv2.imshow("chess",image)
cv2.waitKey(0)
