#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2

original = cv2.imread('white.png', cv2.IMREAD_UNCHANGED)

H, L, S = cv2.split(cv2.cvtColor(original, cv2.COLOR_BGR2HLS))

_, H = cv2.threshold(H, 148, 255, cv2.THRESH_BINARY)
_, L = cv2.threshold(L, 148, 255, cv2.THRESH_BINARY)
_, S = cv2.threshold(S, 148, 255, cv2.THRESH_BINARY)

cv2.imshow('original', original)
cv2.imshow('H', H)
cv2.imshow('L', L)
cv2.imshow('S', S)

cv2.waitKey(0)
cv2.destroyAllWindows()