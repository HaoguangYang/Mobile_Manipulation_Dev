#!/usr/bin/env python
import cv2

img = cv2.imread('/home/cartman/Dev/Maps/p2_village.pgm')
cv2.namedWindow("output", cv2.WINDOW_NORMAL)
cv2.imshow('output', img)
cv2.waitKey(0)
