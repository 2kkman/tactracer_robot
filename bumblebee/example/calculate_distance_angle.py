#!/usr/bin/env python
import math
from UtilBLB import *
import math

    # return {
    #     "FINAL_X": round(total_x, 4),
    #     "FINAL_Y": round(total_y, 4),
    #     "TOTAL_DISTANCE_m": round(hypotenuse, 4),
    #     "DIRECTION_DEGREES": round(angle_deg, 2)
    # }


    # return {
    #     "HYPOTENUSE": round(hypotenuse, 4),
    #     "ANGLE_DEGREES": round(angle_deg, 2)
    # }

def calculate_robot_movement(target_marker, current_marker):
    """
    기준 마커와 현재 인식된 마커를 기반으로 로봇이 회전하고 이동해야 하는 양을 계산합니다.
    
    Args:
        target_marker (dict): 목표 마커 데이터
        current_marker (dict): 현재 인식된 마커 데이터
    
    Returns:
        dict: 회전 각도와 이동해야 할 거리 정보를 담은 딕셔너리
    """
    # 회전 각도 계산 (각도 차이)
    target_angle = target_marker['ANGLE']
    current_angle = current_marker['ANGLE']
    
    # 최단 회전 방향 계산 (시계 또는 반시계)
    rotation_needed = target_angle - current_angle
    if rotation_needed > 180:
        rotation_needed -= 360
    elif rotation_needed < -180:
        rotation_needed += 360
    
    # 3D 공간에서의 이동 거리 계산
    target_pos = (target_marker['X'], target_marker['Y'], target_marker['Z'])
    current_pos = (current_marker['X'], current_marker['Y'], current_marker['Z'])
    
    # 로봇암 이동 거리 계산 (3D 공간에서의 직선 거리)
    distance = math.sqrt(
        (target_pos[0] - current_pos[0])**2 + 
        (target_pos[1] - current_pos[1])**2 + 
        (target_pos[2] - current_pos[2])**2
    )
    
    # 각 축별 이동 거리 계산
    x_movement = target_pos[0] - current_pos[0]
    y_movement = target_pos[1] - current_pos[1]
    z_movement = target_pos[2] - current_pos[2]
    
    # 크레인 구조를 고려한 이동 (거리 변화를 크레인 팔의 길이 변화로 해석)
    # 현재 위치와 목표 위치 사이의 거리를 계산하여 팔의 길이 변화로 사용
    arm_extension = distance
    
    return {
        "rotation_degrees": rotation_needed,
        "arm_movement": arm_extension,
        "x_movement": x_movement,
        "y_movement": y_movement,
        "z_movement": z_movement,
        "target_position": target_pos,
        "current_position": current_pos
    }

# 예제 사용법
if __name__ == "__main__":
    # 예시 데이터
    target_marker = {
        'MARKER_VALUE': 9, 
        'DIFF_X': 1017.92, 
        'DIFF_Y': 1246.92, 
        'X': 0.0625937, 
        'Y': -0.565568, 
        'Z': 0.624784, 
        'ANGLE': 179.74, 
        'CAM_ID': 2, 
        'cx1': 1059.45, 
        'cy1': 1206.35, 
        'cx2': 974.972, 
        'cy2': 1204.24, 
        'cx3': 975.387, 
        'cy3': 1288.48, 
        'cx4': 1059.62, 
        'cy4': 1288.36
    }
    
    current_marker = {
        'MARKER_VALUE': 9, 
        'DIFF_X': 1051.5, 
        'DIFF_Y': 869.489, 
        'X': 0.982023, 
        'Y': -0.160078, 
        'Z': 0.612972, 
        'ANGLE': 2.75178, 
        'CAM_ID': 2, 
        'cx1': 1006.81, 
        'cy1': 909.781, 
        'cx2': 1092.36, 
        'cy2': 914.717, 
        'cx3': 1096.91, 
        'cy3': 828.555, 
        'cx4': 1010.73, 
        'cy4': 824.361
    }
    
    # print(compute_hypotenuse_and_angle(dx, dy))
    # print(compute_hypotenuse_and_angle(-dx, dy))
    # print(compute_hypotenuse_and_angle(-dx, -dy))
    # print(compute_hypotenuse_and_angle(dx, -dy))
    
    
    # movements = [
    #     (dx, dy),     # 6도 방향으로 1m
    #     (dx1, dy1)     # 15도 방향으로 0.5m
    # ]

    #print(movements)
    # result = compute_final_position_and_angle(ref_dict,current_marker)
    dx1 = target_marker['X']
    dy1 = target_marker['Y']
    dx = -current_marker['X']
    dy = -current_marker['Y']    
    distanceI,angleI = compute_hypotenuse_and_angle(dx, dy)
    distanceII,angleII = compute_hypotenuse_and_angle(dx1, dy1)
    print(compute_hypotenuse_and_angle(dx, dy))
    print(compute_hypotenuse_and_angle(dx1, dy1))
    x1,y1= calculate_coordinates(distanceI,angleI)
    movements = [(distanceI,angleI),(distanceII, angleII)]
    print(movements)
    result = compute_final_position_and_angle(movements)
    print(result)
    
    
    # result = calculate_robot_movement(target_marker, current_marker)
    # print(f"회전해야 할 각도: {result['rotation_degrees']:.2f}도")
    # print(f"로봇 팔 이동 거리: {result['arm_movement']:.4f}")
    # print(f"X축 이동: {result['x_movement']:.4f}")
    # print(f"Y축 이동: {result['y_movement']:.4f}")
    # print(f"Z축 이동: {result['z_movement']:.4f}")