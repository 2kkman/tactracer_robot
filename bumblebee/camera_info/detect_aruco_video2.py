import numpy as np
from Util import *
from UtilBLB import *
from UtilAruco import *

import argparse
import time
import cv2
import sys
import os
dirPath = os.path.dirname(__file__)
video = cv2.VideoCapture(0)

arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
arucoParams = cv2.aruco.DetectorParameters_create()
calibration_matrix_path = f'{dirPath}/calibration_matrix.npy'
distortion_coefficients_path = f'{dirPath}/distortion_coefficients.npy'
#distortion_coefficients = 'distortion_coefficients.npy'
k = np.load(calibration_matrix_path)
d = np.load(distortion_coefficients_path)
idx = 0
while True:
  ret, frame = video.read()
  idx += 1
  if ret is False:
    break
  h, w, _ = frame.shape

  width=1000
  height = int(width*(h/w))
  frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_CUBIC)
  corners, ids, rejected = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)
  if ids is not None:
    # Estimate pose of each marker
    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.07, k, d)
  else:
    continue
	# Iterate over detected markers
  for i in range(len(ids)):
		# Draw marker borders
    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
    
		# Draw axis for each marker
    #cv2.aruco.drawAxis(frame, k, d, rvecs[i], tvecs[i], 0.03)
    frame = cv2.drawFrameAxes(frame, k, d, rvecs[i], tvecs[i], 0.03)
    degr = Vector2Degree(rvecs[i]) #Z값이 마커 Rotate 각도가 됨
    print(f'{idx} - corners, rvecs, tvecs, length {corners[i]},{degr},{tvecs[i]}')
  
  
  cv2.imshow("Image", frame)
  key = cv2.waitKey(1) & 0xFF
  if key == ord("q"):
    break

cv2.destroyAllWindows()
video.release()