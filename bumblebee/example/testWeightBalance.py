import numpy as np

# 데이터 정의
weightLen = {
    15: 500,
    12: 450,
    10: 410,
    7: 300,
    3: 200,
    1: 100
}

def get_torque(weight, weightLen):
    keys = sorted(weightLen.keys(), reverse=True)
    
    # 입력 값이 데이터의 최소값보다 작을 경우
    if weight <= keys[-1]:
        return weightLen[keys[-1]]
    
    # 입력 값이 데이터의 최대값보다 클 경우
    if weight >= keys[0]:
        return weightLen[keys[0]]

    # 선형 보간법을 통한 토크 계산
    for i in range(len(keys) - 1):
        if keys[i] >= weight >= keys[i + 1]:
            x1, y1 = keys[i], weightLen[keys[i]]
            x2, y2 = keys[i + 1], weightLen[keys[i + 1]]
            return y1 + (weight - x1) * (y2 - y1) / (x2 - x1)

# 예시 사용
weight = 8.3
torque = get_torque(weight, weightLen)
print(f"Weight {weight} requires torque: {torque}")
