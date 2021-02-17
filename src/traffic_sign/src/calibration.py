#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np

def calibrate_image(frame, mtx, dist, cal_mtx, cal_roi):
    tf_image = cv2.undistort(frame, mtx, dist, None, cal_mtx)
    x, y, w, h = cal_roi
    tf_image = tf_image[y:y+h, x:x+w]
    return cv2.resize(tf_image, (frame.shape[1], frame.shape[0]))

cap = cv2.VideoCapture("Taeback.avi")

mtx = np.array([[422.037858, 0.0, 245.895397], [0.0, 435.589734, 163.625535], [0.0, 0.0, 1.0]])
dist = np.array([-0.289296, 0.061035, 0.001786, 0.015238, 0.0])

Width, Height = 640, 480
cal_mtx, cal_roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (Width, Height), 1, (Width, Height))

while cap.isOpened():
    ret, original = cap.read()
    calibration = calibrate_image(original, mtx, dist, cal_mtx, cal_roi)
    hstack_cat = np.hstack((original, calibration))
    cv2.imshow('original', original)
    cv2.imshow('calibration', calibration)
    cv2.waitKey(1)