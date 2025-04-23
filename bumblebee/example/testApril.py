import cv2
import numpy as np
from dt_apriltags import Detector

# 웹캠 설정 (기본 카메라: 0)
cap = cv2.VideoCapture(0)

# 카메라 기본 파라미터 (초기값 설정)
camera_matrix = np.array([[600, 0, 320],  # 초점 거리 fx
                          [0, 600, 240],  # 초점 거리 fy
                          [0, 0, 1]])     # 중심점 cx, cy

camera_params = (camera_matrix[0, 0], camera_matrix[1, 1], camera_matrix[0, 2], camera_matrix[1, 2])

# AprilTag 탐지기 설정
at_detector = Detector(
    families='tag36h11',
    nthreads=1,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0
)

print("AprilTag 실시간 감지를 시작합니다. 'q'를 눌러 종료하세요.")

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("카메라에서 프레임을 가져올 수 없습니다.")
        break

    # 그레이스케일 변환
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # AprilTag 감지
    tags = at_detector.detect(gray, True, camera_params, tag_size=0.1)  # 태그 크기 (미터 단위, 필요시 조정)

    for tag in tags:
        # 콘솔에 태그 정보 출력
        print(f"Tag ID: {tag.tag_id}")
        print(f"Position: {tag.pose_t.ravel()}")
        print(f"Rotation:\n{tag.pose_R}\n")

    #     # 태그의 꼭짓점 그리기
    #     for idx in range(4):
    #         p1 = tuple(tag.corners[idx - 1, :].astype(int))
    #         p2 = tuple(tag.corners[idx, :].astype(int))
    #         cv2.line(frame, p1, p2, (0, 255, 0), 2)

    #     # 태그 ID 표시
    #     cv2.putText(frame, str(tag.tag_id), 
    #                 (int(tag.corners[0, 0]), int(tag.corners[0, 1]) - 10),
    #                 cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    # # 화면에 결과 표시
    # cv2.imshow('AprilTag Detection', frame)

    # 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 자원 해제
cap.release()
cv2.destroyAllWindows()
