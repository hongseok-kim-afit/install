import cv2
import numpy as np
import matplotlib.pyplot as plt
import pyrealsense2 as rs

# Author: Michael Aboulhair 

pipe = rs.pipeline()
config = rs.config()
config.enable_device('101622074821')
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
profile = pipe.start()

frameset = pipe.wait_for_frames()
color_frame = frameset.get_color_frame()
color_init = np.asanyarray(color_frame.get_data())

font                   = cv2.FONT_HERSHEY_SIMPLEX
bottomLeftCornerOfText = (10,500)
fontScale              = 1
fontColor              = (255,255,255)
lineType               = 2



color = np.asanyarray(color_frame.get_data())
res = color.copy()

cv2.namedWindow('RBG', cv2.WINDOW_AUTOSIZE)
cv2.imshow('RBG', res)

