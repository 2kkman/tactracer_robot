import cv2
import cv2.aruco as aruco

# ArUco 마커 생성을 위한 사전 정의
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

# 마커 생성
marker_id = 23  # 마커 ID
marker_size = 200  # 마커 크기 (픽셀 단위)
marker_image = aruco.drawMarker(aruco_dict, marker_id, marker_size)

cv2.imwrite("aruco_marker.png", marker_image)
