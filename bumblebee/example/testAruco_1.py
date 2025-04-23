import cv2
import cv2.aruco as aruco
import os

dirPath = os.path.dirname(__file__)
pngpath = f'{dirPath}/arucotest3.jpg'
# 이미지 파일 읽기
image = cv2.imread(pngpath)  # 여기에 이미지 파일 경로를 입력하세요.
# 그레이스케일 변환
#gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

def findArucoMarkers(image, markerSize=6, totalMarkers=250):
    # Convert the image to grayscale
    #gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = image

    # Get the Aruco dictionary based on the marker size and total markers
    dictionary_key = getattr(cv2.aruco, f'DICT_{markerSize}X'
                                        f'{markerSize}_{totalMarkers}')

    aruco_dictionary = cv2.aruco.getPredefinedDictionary(dictionary_key)

    # Set the Aruco detector parameters
    aruco_params = cv2.aruco.DetectorParameters()

    # Detect Aruco markers in the grayscale image
    marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(gray, aruco_dictionary,
                                                            parameters=aruco_params)

    return marker_corners, marker_ids

print(findArucoMarkers(image))