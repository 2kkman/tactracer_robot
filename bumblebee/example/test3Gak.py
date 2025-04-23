import math

def calculate_third_side(side_a, side_b, angle_C):
    """
    두 변의 길이와 그 사이의 각을 입력받아 나머지 한 변의 길이를 구하는 함수.
    
    매개변수:
    side_a (float): 첫 번째 변의 길이
    side_b (float): 두 번째 변의 길이
    angle_C (float): 두 변 사이의 각도 (단위: 도)
    
    반환:
    float: 나머지 한 변의 길이
    """
    # 각도를 라디안으로 변환
    angle_C_rad = math.radians(angle_C)
    
    # 코사인 법칙을 사용하여 나머지 한 변의 길이 계산
    side_c = math.sqrt(side_a**2 + side_b**2 - 2 * side_a * side_b * math.cos(angle_C_rad))
    
    return side_c

# 예시 사용
side_a = 596   # 첫 번째 변의 길이
side_b = 320   # 두 번째 변의 길이
angle_C = 90 # 두 변 사이의 각도

side_c = calculate_third_side(side_a, side_b, angle_C)
print(f"나머지 한 변의 길이: {side_c:.2f}")
