'''
Sample Command:-
python detect_aruco_video.py --type DICT_5X5_100 --camera True
python detect_aruco_video.py --type DICT_5X5_100 --camera False --video test_video.mp4
'''

import numpy as np
from UtilAruco import ARUCO_DICT, aruco_display
import argparse
import time
import cv2
import sys
import os
dirPath = os.path.dirname(__file__)
pngpath = f'{dirPath}/aruco_marker.png'


# ap = argparse.ArgumentParser()
# ap.add_argument("-i", "--camera", required=True, help="Set to True if using webcam")
# ap.add_argument("-v", "--video", help="Path to the video file")
# ap.add_argument("-t", "--type", type=str, default="DICT_ARUCO_ORIGINAL", help="Type of ArUCo tag to detect")
# args = vars(ap.parse_args())
video = cv2.VideoCapture(0)
# if args["camera"].lower() == "true":
# 	video = cv2.VideoCapture(0)
# 	time.sleep(2.0)
	
# else:
# 	if args["video"] is None:
# 		print("[Error] Video file location is not provided")
# 		sys.exit(1)

# 	video = cv2.VideoCapture(args["video"])

# if ARUCO_DICT.get(args["type"], None) is None:
# 	print(f"ArUCo tag type '{args['type']}' is not supported")
# 	sys.exit(0)

#arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[args["type"]])


def pose_esitmation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):

    '''
    frame - Frame from the video stream
    matrix_coefficients - Intrinsic matrix of the calibrated camera
    distortion_coefficients - Distortion coefficients associated with your camera

    return:-
    frame - The frame with the axis drawn on it
    '''

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters_create()
    corners, ids, rejected = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)
    # corners, ids, rejected_img_points = cv2.aruco.detectMarkers(frame, cv2.aruco_dict,parameters=parameters,
    #     cameraMatrix=matrix_coefficients,
    #     distCoeff=distortion_coefficients)

        # If markers are detected
    if len(corners) > 0:
        for i in range(0, len(ids)):
            # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients,
                                                                       distortion_coefficients)
            # Draw a square around the markers
            cv2.aruco.drawDetectedMarkers(frame, corners) 

            # Draw Axis
            cv2.aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)  

    return frame
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
arucoParams = cv2.aruco.DetectorParameters_create()
calibration_matrix_path = f'{dirPath}/calibration_matrix.npy'
distortion_coefficients_path = f'{dirPath}/distortion_coefficients.npy'
#distortion_coefficients = 'distortion_coefficients.npy'
k = np.load(calibration_matrix_path)
d = np.load(distortion_coefficients_path)

while True:
	ret, frame = video.read()
	
	if ret is False:
		break


	h, w, _ = frame.shape

	width=1000
	height = int(width*(h/w))
	frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_CUBIC)
	#corners, ids, rejected = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)

	# detected_markers = aruco_display(corners, ids, rejected, frame)
	# cv2.imshow("Image", detected_markers)
 
	pose_img=pose_esitmation(frame,cv2.aruco.DICT_6X6_250, k,d)
	cv2.imshow("Image", pose_img)
 
	key = cv2.waitKey(1) & 0xFF
	if key == ord("q"):
	    break

cv2.destroyAllWindows()
video.release()