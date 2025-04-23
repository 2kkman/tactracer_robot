import math

def calculate_coordinates(distance, angle_degrees, x1=0, y1=0):
    """
    주어진 거리와 각도로부터 X, Y 좌표를 계산하는 함수.
    
    :param distance: 거리
    :param angle_degrees: 각도 (도 단위)
    :return: (x, y) 좌표
    """
    # 각도를 라디안 단위로 변환
    angle_radians = math.radians(angle_degrees)
    
    # X, Y 좌표 계산
    x = distance * math.cos(angle_radians) + x1
    y = distance * math.sin(angle_radians) + y1
    
    return round(x), round(y)

# 예제 사용
start_x, start_y = 0, 0       # C의 좌표
radius = 5.0                  # 반지름
#adjusted_initial_angle = 0    # 초기 방향 (기준 방향을 포함한 값)
rotation_angle = 45          # 회전 각도 (시계 방향으로 45도)

final_x, final_y = calculate_coordinates(radius, rotation_angle,start_x,start_y)
print(f"D의 좌표: ({final_x:.2f}, {final_y:.2f})")
