
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2

original = cv2.imread('yellow.png', cv2.IMREAD_UNCHANGED)

L, A, B = cv2.split(cv2.cvtColor(original, cv2.COLOR_BGR2LAB))

_, L = cv2.threshold(L, 156, 255, cv2.THRESH_BINARY)
_, A = cv2.threshold(A, 156, 255, cv2.THRESH_BINARY)
_, B = cv2.threshold(B, 156, 255, cv2.THRESH_BINARY)

cv2.imshow('original', original)
cv2.imshow('L', L)
cv2.imshow('A', A)
cv2.imshow('B', B)

cv2.waitKey(0)
cv2.destroyAllWindows()