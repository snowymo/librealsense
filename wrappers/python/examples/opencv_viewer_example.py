## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2
import argparse
import pickle

parser = argparse.ArgumentParser(description='Process the amount of the devices.')
parser.add_argument('-a', '--amount', type=int,help='the amount for the device', default = 1)
args = parser.parse_args()

# the amount of the calibrated images
calibrated_amount = 15
# the size of each square
square_unit = 0.015
w = 7
h = 6
# color camera resolution
width = 640
height = 480
# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
stereocalib_criteria = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 100, 1e-5)
flags = 0
flags |= cv2.CALIB_FIX_INTRINSIC
# flags |= cv2.CALIB_FIX_PRINCIPAL_POINT
flags |= cv2.CALIB_USE_INTRINSIC_GUESS
flags |= cv2.CALIB_FIX_FOCAL_LENGTH
# flags |= cv2.CALIB_FIX_ASPECT_RATIO
flags |= cv2.CALIB_ZERO_TANGENT_DIST
# flags |= cv2.CALIB_RATIONAL_MODEL
# flags |= cv2.CALIB_SAME_FOCAL_LENGTH

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((w*h,3), np.float32)
objp[:,:2] = np.mgrid[0:w,0:h].T.reshape(-1,2) * square_unit
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints_s = [[0 for x in range(calibrated_amount)] for y in range(args.amount)] 
index = 0

pipelines = []
configs = []
frames_s = [0] * args.amount
img_shape = (0,0,0)
serial_numbers = ["805212060447", "805212060158"]
for deviceId in range(args.amount):
        # Configure depth and color streams
	pipeline = rs.pipeline()
	config = rs.config()
	# only use serial number if there are more than one device
	if args.amount > 1:
                config.enable_device(serial_numbers[deviceId])
	config.enable_stream(rs.stream.depth, -1, width, height, rs.format.z16, 30)
	config.enable_stream(rs.stream.color, -1, width, height, rs.format.bgr8, 30)

	profile = config.resolve(pipeline) # does not start streaming
	# Start streaming
	profile = pipeline.start(config)

	print (profile.get_streams())
	pipelines.append(pipeline)
	configs.append(config)

try:
        while index < calibrated_amount:
            # Wait for all devices
            allret = True
            grays = []
            corners_s = []
            rets = []
            for deviceId in range(args.amount):
                    print ("index:" + str(index) + " deviceId:" + str(deviceId))
                    frames_s[deviceId] = pipelines[deviceId].wait_for_frames()
                    color_frame = frames_s[deviceId].get_color_frame()
                    if not color_frame:
                            break
                        # Convert images to numpy arrays
                    color_image = np.asanyarray(color_frame.get_data())
                    gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
                    img_shape = gray.shape[::-1]
                    cv2.imshow('RealSense:' + str(deviceId), color_image)

                    # Find the chess board corners
                    ret, corners = cv2.findChessboardCorners(gray, (w,h),None)
                    grays.append(gray)
                    imgpoints_s[deviceId][index] = corners
                    rets.append(ret)
                    allret = allret and ret
                
                    if ret != True:				
                            break
                    # Stack both images horizontally
                    #images = np.hstack((color_image, depth_colormap))
            if allret:
                    # If all found, add object points, image points (after refining them)
                    objpoints.append(objp)

                    for deviceId in range(args.amount):
                        # TODO refine
                            imgpoints_s[deviceId][index] = cv2.cornerSubPix(grays[deviceId],imgpoints_s[deviceId][index],(11,11),(-1,-1),criteria)
			# Draw and display the corners
                            gray = cv2.drawChessboardCorners(grays[deviceId], (w,h), imgpoints_s[deviceId][index],rets[deviceId])
                            cv2.destroyWindow('device ' + str(deviceId) + ' images with corners:' + str(index-1))
                            cv2.imshow('device ' + str(deviceId) + ' images with corners:' + str(index), gray)
			# Show images				#cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)            
                            cv2.imwrite("calibrate\\device" + str(deviceId) + "_imgae" + str(index) + ".jpg", color_image)
                    index = index+1
                    cv2.waitKey(1000)
            cv2.waitKey(1)

        mtxs = []
        dists = []
        for deviceId in range(args.amount):
                ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints_s[deviceId], img_shape,None,None)
                mtxs.append(mtx)
                dists.append(dist)
                print('Individual Intrinsic_mtx', mtxs)
        if args.amount == 2:
                retval, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, R, T, E, F = cv2.stereoCalibrate(objpoints, imgpoints_s[0], imgpoints_s[1], mtxs[0], dists[0], mtxs[1], dists[1], img_shape, criteria=stereocalib_criteria, flags=flags);
                print('\nStereo Intrinsic_mtx', mtxs)
                print('R', R)
                print('T', T)
        cv2.destroyAllWindows()
        with open('calibResult', 'wb') as fp:
                pickle.dump(args.amount,fp)
                pickle.dump(mtxs, fp)
                if args.amount == 2:
                        pickle.dump(R, fp)
                        pickle.dump(T, fp)

finally:
    # Stop streaming
    for deviceId in range(args.amount):
            pipelines[deviceId].stop()
