## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2
import argparse

parser = argparse.ArgumentParser(description='Process the amount of the devices.')
parser.add_argument('-a', '--amount', type=int,help='the amount for the device', default = 1)
args = parser.parse_args()

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, -1, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, -1, 640, 480, rs.format.bgr8, 30)

profile = config.resolve(pipeline) # does not start streaming
# Start streaming
profile = pipeline.start(config)

print (profile.get_streams())

# the amount of the calibrated images
calibrated_amount = 15
# the size of each square
square_unit = 0.015
w = 7
h = 6

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2) * square_unit

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
index = 0

# Amount of the devices
device_amount = args.amount

try:
    while index <= calibrated_amount:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        #depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # Convert images to numpy arrays
        #depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        cv2.imshow('RealSense', color_image)
        #cv2.imshow('gray', gray)
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (w,h),None)

        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            gray = cv2.drawChessboardCorners(gray, (w,h), corners2,ret)
            cv2.destroyWindow('images with corners:' + str(index-1))
            cv2.imshow('images with corners:' + str(index),gray)
            # Show images
            #cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            
            cv2.imwrite("calibrate\\" + str(index) + ".jpg", color_image)
            index = index+1
        
            cv2.waitKey(1000)
	
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        #depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Stack both images horizontally
        #images = np.hstack((color_image, depth_colormap))

        
        cv2.waitKey(1)
    cv2.destroyAllWindows()

finally:

    # Stop streaming
    pipeline.stop()
