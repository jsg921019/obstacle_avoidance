#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2

def calibrate_image(frame, mtx, dist, cal_mtx, cal_roi):
    height, width, _ = frame.shape
    tf_image = cv2.undistort(frame, mtx, dist, None, cal_mtx)
    x, y, w, h = cal_roi
    tf_image = tf_image[y:y+h, x:x+w]

    return cv2.resize(tf_image, (width, height))

def detect_stopline_contour(cal_image, low_threshold_value):
    blur = cv2.GaussianBlur(cal_image, (5, 5), 0)
    _, _, B = cv2.split(cv2.cvtColor(blur, cv2.COLOR_BGR2LAB))
    _, lane = cv2.threshold(B, low_threshold_value, 255, cv2.THRESH_BINARY)
    _, contours, _ = cv2.findContours(lane, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    for cont in contours:
        length = cv2.arcLength(cont, True)
        area = cv2.contourArea(cont)

        if not ((area > 3000) and (length > 500)):
            continue

        if len(cv2.approxPolyDP(cont, length*0.02, True)) != 4:
            continue

        (x, y, w, h) = cv2.boundingRect(cont)
        center = (x + int(w/2), y + int(h/2))
        _, width, _ = cal_image.shape

        if 200 <= center[0] <= (width - 200):
            cv2.rectangle(cal_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            print("stopline")

cap = cv2.VideoCapture("Taeback.avi")

Width, Height = 640, 480
mtx = np.array([[422.037858, 0.0, 245.895397], [0.0, 435.589734, 163.625535], [0.0, 0.0, 1.0]])
dist = np.array([-0.289296, 0.061035, 0.001786, 0.015238, 0.0])
cal_mtx, cal_roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (Width, Height), 1, (Width, Height)) 

while cap.isOpened():
    ret, frame = cap.read()
    cal_image = calibrate_image(frame, mtx, dist, cal_mtx, cal_roi)
    detect_stopline_contour(cal_image, 150)
    cv2.imshow("Contours", cal_image)
    cv2.waitKey(1)
