import numpy as np
from sklearn.linear_model import LinearRegression

# 주어진 카메라 좌표와 실제 거리 데이터
camera_coordinates = np.array([-363, -98.4, 432, 537]).reshape(-1, 1)
real_distances = np.array([165, 550, 1220, 1350])

# 선형 회귀 모델을 학습
model = LinearRegression()
model.fit(camera_coordinates, real_distances)

# 새로운 카메라 좌표에 대한 실제 거리 예측 함수
def estimate_real_distance(camera_coordinate):
    estimated_distance = model.predict(np.array([[camera_coordinate]]))
    return estimated_distance[0]

# 테스트
test_coordinate = -353  # 예시 카메라 좌표
predicted_distance = estimate_real_distance(test_coordinate)
print(f"Predicted distance for camera coordinate {test_coordinate}: {predicted_distance:.2f} cm")
