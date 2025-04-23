import cv2
import cv2.aruco as aruco
import os

dirPath = os.path.dirname(__file__)
pngpath = f'{dirPath}/aruco_marker.png'

# 이미지 파일 읽기
image = cv2.imread(pngpath)  # 여기에 이미지 파일 경로를 입력하세요.

# 그레이스케일 변환
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#gray = image

# ArUco 마커 사전 정의
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters_create()

# 마커 인식
corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

print(corners, ids, rejectedImgPoints )
# 인식된 마커에 사각형 그리기
if ids is not None:
    aruco.drawDetectedMarkers(image, corners, ids)

# 결과 이미지 보여주기
cv2.imshow('Detected ArUco markers', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
