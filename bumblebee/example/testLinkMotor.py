import matplotlib.pyplot as plt
import numpy as np
import math

# 가상의 링크 구조에 따른 각도 변화 모델링
def calculate_lifting_angle(motor_rotation):
    # 초기 급격한 변화 후 완만해지는 비선형 함수 모델링
    rt = 100 * (1 - np.exp(-0.05 * motor_rotation))
    return rt
    # norm_x = (motor_rotation - 0) / (100 - 0)
    # return 100 * 1 * math.log(norm_x * (math.e - 1) + 1)

# 모터 회전수 범위
motor_rotations = np.linspace(0, 100, 400)

# 각도 계산
lifting_angles = calculate_lifting_angle(motor_rotations)

# 그래프 그리기
plt.figure(figsize=(10, 6))
plt.plot(motor_rotations, lifting_angles, label='Lifting Angle', color='b')
plt.title('Motor Rotations vs Lifting Angle')
plt.xlabel('Motor Rotations')
plt.ylabel('Lifting Angle (degrees)')
plt.legend()
plt.grid(True)
plt.show()
