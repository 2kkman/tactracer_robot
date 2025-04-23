import cv2
import numpy as np
import apriltag
import time

# 📌 카메라 및 태그 설정
TAG_SIZE = 0.15  # AprilTag 실제 크기 (미터) - 측정하여 설정
FX = 1666.67  # 초점 거리 fx (픽셀 단위)
FY = 1666.67  # 초점 거리 fy (픽셀 단위)
CX = 1920 / 2  # 카메라 중심 x
CY = 1080 / 2  # 카메라 중심 y

# 3D 객체 포인트 (태그의 실제 크기)
OBJ_POINTS = np.array([
    [-TAG_SIZE / 2, TAG_SIZE / 2, 0],  # 좌상
    [TAG_SIZE / 2, TAG_SIZE / 2, 0],   # 우상
    [TAG_SIZE / 2, -TAG_SIZE / 2, 0],  # 우하
    [-TAG_SIZE / 2, -TAG_SIZE / 2, 0]  # 좌하
], dtype=np.float32)

def main():
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("웹캠에서 프레임을 읽을 수 없습니다.")
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            results = detector.detect(gray)

            for r in results:
                # 태그 코너 포인트 가져오기
                img_points = np.array(r.corners, dtype=np.float32)

                # PnP (SolvePnP) 를 이용해 3D 좌표 추출
                camera_matrix = np.array([[FX, 0, CX], [0, FY, CY], [0, 0, 1]], dtype=np.float32)
                dist_coeffs = np.zeros((4, 1))  # 왜곡 계수 (기본적으로 없음)
                
                success, rvec, tvec = cv2.solvePnP(OBJ_POINTS, img_points, camera_matrix, dist_coeffs)

                if success:
                    x, y, z = tvec.flatten()

                    # 회전 벡터를 회전 행렬로 변환
                    rotation_matrix, _ = cv2.Rodrigues(rvec)
                    
                    # 수평 회전각 (Yaw) 계산
                    yaw = np.degrees(np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0]))

                    # 태그 정보 출력
                    print(f"\n=== 태그 감지 시간: {time.strftime('%Y-%m-%d %H:%M:%S')} ===")
                    print(f"태그 ID: {r.tag_id}")
                    print(f"x={x:.4f} m, y={y:.4f} m, z={z:.4f} m")
                    print(f"수평 회전 각도 (Yaw): {yaw:.2f}°")
                    print(f"결정 신뢰도: {r.decision_margin:.2f}")

            # 'q' 키를 누르면 종료
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
