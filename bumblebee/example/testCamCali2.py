import cv2
import numpy as np
import os

# 체스보드 설정
checkerboard_size = (9,6)
square_size = 22  # 체스보드 한 칸의 크기(mm)

# 3D 점 초기화
objp = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
objp[:, :2] = (
    np.mgrid[0 : checkerboard_size[0], 0 : checkerboard_size[1]].T.reshape(-1, 2)
    * square_size
)

# 각 캡처 이미지의 3D 점과 2D 코너를 저장할 리스트
objpoints = []
imgpoints = []

# 종료 기준
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
camID = 0
# 카메라 초기화
cap = cv2.VideoCapture(camID)

if not cap.isOpened():
    print("Cannot open camera")
    exit()

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, checkerboard_size, cv2.CALIB_CB_ADAPTIVE_THRESH)

    # 체스보드 코너가 감지된 경우
    if ret == True:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        # 코너 그리기
        cv2.drawChessboardCorners(frame, checkerboard_size, corners2, ret)
        print(f"Detected corners for image {len(objpoints)}")

    cv2.imshow("Frame", frame)
    if cv2.waitKey(1) == ord("q") or len(objpoints) >= 20:  # 20개 이미지 캡처 후 종료
        print("Exiting and processing calibration...")
        break

cap.release()
cv2.destroyAllWindows()

if len(objpoints) > 0:
    # 카메라 보정 수행
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )
    print("Calibration successful")

    # 결과 파일 저장 경로 설정
    calibration_file = f"calibration_matrix_{camID}.npy"
    distortion_file = f"distortion_coefficients_{camID}.npy"

    # 결과 저장
    np.save(calibration_file, mtx)
    np.save(distortion_file, dist)
    print(f"Calibration matrix saved to {calibration_file}")
    print(f"Distortion coefficients saved to {distortion_file}")
else:
    print("No valid images were captured. Calibration cannot be performed.")
