import math

def calculate_robot_translation(current_x, current_y, rotation_deg, target_x=-0.34397, target_y=1.12045):
    """
    현재 카메라 위치와 회전각도를 바탕으로, target_x, target_y로 보정하기 위한 로봇의 이동량 계산.

    :param current_x: 회전된 상태에서의 마커 X (예: -0.599308)
    :param current_y: 회전된 상태에서의 마커 Y (예: -0.0704956)
    :param rotation_deg: 카메라의 반시계 회전 각도 (예: 90, 180 등)
    :param target_x: 기준 각도 0도일 때의 마커 X
    :param target_y: 기준 각도 0도일 때의 마커 Y
    :return: (delta_x, delta_y) – 로봇이 이동해야 할 거리 (X, Y 축 방향)
    """
    # 각도를 라디안으로 변환
    theta = math.radians(rotation_deg)

    # 현재 좌표를 기준 각도(0도)로 되돌리기 위한 역회전
    rotated_x = current_x * math.cos(-theta) - current_y * math.sin(-theta)
    rotated_y = current_x * math.sin(-theta) + current_y * math.cos(-theta)

    # 타겟 좌표까지 이동량 계산
    delta_x = target_x - rotated_x
    delta_y = target_y - rotated_y

    return delta_x, delta_y

# 90도 회전된 상태에서 마커 위치
x, y = -0.599308, -0.0704956
angle = 90

dx, dy = calculate_robot_translation(x, y, angle)
print(f"로봇은 X축 {dx:.4f}m, Y축 {dy:.4f}m 이동해야 함")

import math

# 마커 좌표 (X, Y) at 각 회전각
marker_positions = {
    0: (-0.34397, 1.12045),
    90: (-0.599308, -0.0704956),
    180: (0.57885, -0.323689),
    270: (0.839547, 0.851228)
}

def estimate_rotation_radius(marker_positions):
    # 각도를 라디안으로 변환하여 평균 중심 추정
    angles = []
    xs, ys = [], []

    for deg, (x, y) in marker_positions.items():
        angles.append(math.radians(deg))
        xs.append(x)
        ys.append(y)

    # 중심 추정: 단순 평균
    center_x = sum(xs) / len(xs)
    center_y = sum(ys) / len(ys)

    # 중심에서 각 점까지 거리 = 반지름
    distances = [math.hypot(x - center_x, y - center_y) for x, y in zip(xs, ys)]
    avg_radius = sum(distances) / len(distances)

    return avg_radius, (center_x, center_y)

radius, center = estimate_rotation_radius(marker_positions)
print(f"카메라까지의 거리(회전 반경): {radius:.4f} m")
print(f"추정된 회전 중심 좌표: X={center[0]:.4f}, Y={center[1]:.4f}")
