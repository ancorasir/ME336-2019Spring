#!/usr/bin/env python
import cv2
# import numpy as np
# import sys

def DetectForeground(groundImg,currrentImg):
    groundImg_gray = cv2.cvtColor(groundImg,cv2.COLOR_RGB2GRAY)
    groundBlur = cv2.GaussianBlur(groundImg_gray,(5,5),1)
    groundBlur.dtype = 'int16'

    currrentImg_gray = cv2.cvtColor(currrentImg,cv2.COLOR_RGB2GRAY)
    currrentImgBlur = cv2.GaussianBlur(currrentImg_gray,(5,5),1)
    currrentImgBlur.dtype = 'int16'

    dGrayBlur = abs(groundBlur-currrentImgBlur)
    dGrayBlur.dtype = 'uint8'
    dGrayMidBlur=cv2.medianBlur(dGrayBlur,5)

    ret,thresh=cv2.threshold(dGrayMidBlur,10,255,cv2.THRESH_BINARY)
    im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    rect = []
    for i in range(len(contours)):
        area = cv2.contourArea(contours[i])
        if area < 50*50:
            continue
        else:
            temp = cv2.boundingRect(contours[i])
            rect.append(temp)
    return rect
