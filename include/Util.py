#!/usr/bin/env python3

# Standard library imports
from datetime import *  # datetime의 모든 클래스/함수를 직접 사용
from itertools import *
from collections import *  # defaultdict, Counter 등을 직접 사용
from enum import Enum, auto
from io import *
from statistics import *  # mean, median 등을 직접 사용
from typing import *
from urllib.parse import *
from pathlib import *
import ast
import copy
import csv
import difflib
import heapq
import inspect
import itertools
import json
import math
import os
import pickle
import re
import socket
import string
import struct
import subprocess
import sys
import threading
import time
import traceback
import psutil
from skimage.metrics import structural_similarity as ssim

# Third-party imports
from flask_socketio import *
from transitions import *
from varname import *
from bitstring import *

import keyboard
import matplotlib
matplotlib.use('Agg')  # GUI 없이 사용
from matplotlib import pyplot as plt
plt.ioff()  # 인터랙티브 모드 비활성화

import networkx as nx
import numpy as np
import pandas as pd
import pcl
#import pcl.pcl_visualization

# ROS related imports
import roslibpy
import rospy
import rosservice
import rosgraph
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from std_msgs.msg import *
import sensor_msgs.point_cloud2 as pc2

# HTTP related imports
import urllib3
import urllib.parse
urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

# Local application imports
from UtilHTTP import *
import logging
import os
import inspect
import math
import numpy as np
import cv2
import numpy as np
import requests
from datetime import datetime
save_dir_download = "/root/Downloads"
machine_running_csv_filename = 'IsCleanTable.csv'
machine_running_csv_filepath = os.path.join(save_dir_download,machine_running_csv_filename)
def getDateTime():
  return datetime.now()

def estimate_backlash_error(current_distance, full_distance=7259818, full_error=89000):
    """
    선형 보간법을 통해 현재 이동 거리에서의 예상 오차를 계산합니다.
    
    Parameters:
    - current_distance: 현재 이동 거리 (정방향 이동한 거리)
    - full_distance: 전체 이동 거리 (기준 거리)
    - full_error: 전체 거리에서 발생한 오차 (기준 오차)
    
    Returns:
    - 예상 오차 (float)
    """
    error = (current_distance / full_distance) * full_error
    return round(error)

# # 예시
# distance = 3000000  # 현재 이동 거리
# estimated_error = estimate_backlash_error(distance)

# print(f"{distance} 펄스 이동 시 예상 오차는 약 {estimated_error:.2f} 펄스입니다.")

def GetResultMessageFromJsonStr(data_str):
    try:
        data = json.loads(data_str)
        if isinstance(data, dict) and len(data) == 1:
            key = next(iter(data))
            value = data[key]
            print("Key:", key)
            print("Value:", value)
            return key,value
        else:
            print("Unexpected data format.")
    except json.JSONDecodeError as e:
        print("Invalid JSON:", e)    
    return None,None

def compare_images_region(img1, img2, x1,y1,x2,y2, method='ssim'):
    """
    img1, img2: 비교할 두 이미지 (numpy 배열, 동일한 크기)
    pt1, pt2: 비교할 직사각형 영역의 좌상단과 우하단 (튜플, 예: (x1, y1), (x2, y2))
    method: 'ssim' 또는 'mse' 선택
    return: 유사도 값 (SSIM은 0~1, MSE는 0 이상)
    """
    # x1, y1 = pt1
    # x2, y2 = pt2

    # 영역 추출
    crop1 = img1[y1:y2, x1:x2]
    crop2 = img2[y1:y2, x1:x2]

    # 그레이스케일 변환 (선택적)
    if len(crop1.shape) == 3:
        crop1 = cv2.cvtColor(crop1, cv2.COLOR_BGR2GRAY)
        crop2 = cv2.cvtColor(crop2, cv2.COLOR_BGR2GRAY)

    if method == 'ssim':
        score, _ = ssim(crop1, crop2, full=True)
        return score
    elif method == 'mse':
        err = np.mean((crop1.astype("float") - crop2.astype("float")) ** 2)
        return err
    else:
        raise ValueError("method must be 'ssim' or 'mse'")
    
def calculate_weight_angle(arm_length_mm, max_length=1300, trigger_distance=500, max_angle=90):
    if arm_length_mm <= trigger_distance:
        return 0
    elif arm_length_mm >= max_length:
        return (max_angle)
    else:
        ratio = (arm_length_mm - trigger_distance) / (max_length - trigger_distance)
        return round(ratio * max_angle)

# # 예시
# length = 290
# angle = calculate_weight_angle(length)
# print(f"Arm length: {length} mm → Weight angle: {angle} degrees")

def capture_frame_from_mjpeg(url='https://172.30.1.8:6001/cam', save_dir=save_dir_download, timeout=5):
    """
    MJPEG 스트림에서 1프레임을 캡처해서 저장하는 함수
    """
    os.makedirs(save_dir, exist_ok=True)

    # SSL 무시하고 MJPEG 스트림 연결
    try:
        session = requests.Session()
        stream = session.get(url, stream=True, verify=False, timeout=timeout)
    except Exception as e:
        print(traceback.format_exc())
        print(f"Failed to connect to stream: {e}")
        return None

    bytes_data = b''
    for chunk in stream.iter_content(chunk_size=1024):
        bytes_data += chunk
        a = bytes_data.find(b'\xff\xd8')  # JPEG 시작
        b = bytes_data.find(b'\xff\xd9')  # JPEG 끝
        if a != -1 and b != -1:
            jpg = bytes_data[a:b+2]
            bytes_data = bytes_data[b+2:]
            img = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)

            # 저장
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            file_name_save = f"captured_{timestamp}.jpg"
            save_path = os.path.join(save_dir,file_name_save )
            cv2.imwrite(save_path, img)
            print(f"Captured and saved: {save_path}")

            stream.close()
            return file_name_save

    print("Failed to capture frame")
    return None
# import csv
# from datetime import datetime

def save_image_with_lidar_data(filename, descendable_distance, tilt_deg, obstacle_thresh=0.4,
                                save_dir=save_dir_download, csv_path=machine_running_csv_filepath):
    """
    이미지 저장 + 라이다 데이터 + 라벨 기록 CSV
    """
    os.makedirs(save_dir, exist_ok=True)

    # # 이미지 파일명 생성
    #timestamp = getDateTime().strftime("%Y%m%d_%H%M%S_%f")
    # filename = f"captured_{timestamp}.jpg"
    #filepath = os.path.join(save_dir, filename)

    # # 저장
    # cv2.imwrite(filepath, img)

    # 라벨 판단
    obstacle = 1 if descendable_distance < obstacle_thresh else 0

    # CSV에 기록
    header = ['timestamp', 'filename', 'tilt_deg', 'descendable_distance', 'obstacle']
    row = [datetime.now().strftime("%Y-%m-%d %H:%M:%S"), filename, tilt_deg, round(descendable_distance, 4), obstacle]

    write_header = not os.path.exists(csv_path)

    with open(csv_path, 'a', newline='') as f:
        writer = csv.writer(f)
        if write_header:
            writer.writerow(header)
        writer.writerow(row)

    print(f"[✔] 저장 완료: {filename} (obstacle={obstacle})")

def estimate_rotation_center(marker_coords_by_angle):
    """
    주어진 회전 각도와 마커 좌표들로부터 회전 중심을 추정
    """
    angles = []
    xs, ys = [], []

    for angle_deg, (x, y) in marker_coords_by_angle.items():
        angles.append(math.radians(angle_deg))
        xs.append(x)
        ys.append(y)

    # 중심점은 평균 좌표 (대략적인 원 중심)
    center_x = np.mean(xs)
    center_y = np.mean(ys)
    return center_x, center_y

def calculate_position_shift(current_angle_deg, marker_coords_by_angle, target_x=0.03, target_y=0.48):
    """
    주어진 회전각에서 마커가 기준 위치(target_x, target_y)에 위치하게 하려면
    로봇을 x, y 방향으로 얼마나 이동시켜야 하는지 계산
    """
    # 회전 중심 추정
    cx, cy = estimate_rotation_center(marker_coords_by_angle)

    # 현재 회전각에서 마커 위치 예측
    radius = math.hypot(marker_coords_by_angle[0][0] - cx, marker_coords_by_angle[0][1] - cy)
    initial_angle_rad = math.atan2(marker_coords_by_angle[0][1] - cy, marker_coords_by_angle[0][0] - cx)

    # 회전 후 예상 마커 위치
    theta = math.radians(current_angle_deg)
    expected_x = cx + radius * math.cos(initial_angle_rad + theta)
    expected_y = cy + radius * math.sin(initial_angle_rad + theta)

    # 기준 위치에 맞추기 위한 로봇 이동량
    dx = target_x - expected_x
    dy = target_y - expected_y

    return dx*1000, dy*1000

def calculate_robot_movement( target_x, target_y, current_length,current_angle=None):
    """
    로봇 팔의 현재 상태에서 목표 위치로 이동하기 위한 길이 변화와 회전 각도를 계산합니다.
    
    Parameters:
    current_length (float): 현재 로봇 팔의 길이 (mm)
    target_x (float): 목표 x 좌표 (mm)
    target_y (float): 목표 y 좌표 (mm)
    current_angle (float, optional): 현재 로봇 팔의 각도 (도). None이면 계산됩니다.
    
    Returns:
    tuple: (필요한 길이, 필요한 각도, 길이 변화, 각도 변화)
    """
    # 목표 위치까지의 거리 계산
    target_distance = math.sqrt(target_x**2 + target_y**2)
    
    # 목표 위치의 각도 계산 (라디안)
    target_angle_rad = math.atan2(target_y, target_x)
    # 라디안에서 도로 변환
    target_angle_deg = math.degrees(target_angle_rad)
    
    # 현재 각도가 주어지지 않았다면, 현재 위치는 원점에서 시작하고 x축 방향으로 뻗어있다고 가정
    if current_angle is None:
        current_angle = 0  # 기본값은 x축 방향 (0도)
    
    # 필요한 각도 변화 계산
    angle_change = target_angle_deg - current_angle
    # 각도를 -180도에서 180도 사이로 조정
    if angle_change > 180:
        angle_change -= 360
    elif angle_change < -180:
        angle_change += 360
    
    # 필요한 길이 변화 계산
    length_change = target_distance - current_length
    
    #return target_distance, target_angle_deg, length_change, angle_change
    return length_change, angle_change

ref_x = 0.03
ref_y = 0.48

# 마커 좌표: 회전각도 → (x, y)
marker_coords_goldsample = {
    0: (ref_x, ref_y),
    90: (0.13, 0.51),
    180: (0.17, 0.41),
    270: (0.06, 0.37)
}

# # 임의 회전 각도에서 보정 이동량 계산
# angle = 45  # 현재 로봇이 45도 회전했다면?
# dx, dy = calculate_position_shift(angle, marker_coords_goldsample)
# print(f"로봇을 X축 {dx:.4f}m, Y축 {dy:.4f}m 이동시키면 마커가 기준 위치에 옴")



def get_camera_offset(armLength_mm, angle_degree):
    angle_rad = math.radians(angle_degree)
    
    # 회전 후 위치
    new_x = armLength_mm * math.cos(angle_rad)
    new_y = armLength_mm * math.sin(angle_rad)
    
    # 원래 위치에서의 변화량 (Δx, Δy)
    delta_x = new_x - armLength_mm
    delta_y = new_y - 0
    
    return delta_x, delta_y

def format_vars(*args):
    frame = inspect.currentframe().f_back
    names = {id(v): k for k, v in frame.f_locals.items()}
    return ','.join(f"{names.get(id(arg), '?')}={arg}" for arg in args)

class APRIL_RESULT(Enum):
    ROTATE_ANGLE = auto()
    DISTANCE_mm = auto()
    MARGIN_mm = auto()

def calculate_relative_extension_and_rotation(target_x, target_y, current_length=100, current_theta_deg=0):
    """
    현재 암의 회전각 current_theta_deg 도에서 current_length 만큼 뻗어 있는 상태에서
    target_x, target_y 지점에 도달하려면 얼마나 더 뻗고 몇 도 상대적으로 회전해야 하는지 계산

    :param target_x: 목표 X 좌표 (mm)
    :param target_y: 목표 Y 좌표 (mm)
    :param current_length: 현재 암 길이 (mm)
    :param current_theta_deg: 현재 회전 각도 (deg, 반시계 기준 0도 = +X방향)
    :return: (추가 뻗기 Δr in mm, 상대 회전 각도 Δθ in degrees)
    """
    # 목표 거리와 각도 계산
    r_target = math.hypot(target_x, target_y)
    theta_target = math.degrees(math.atan2(target_y, target_x))

    # 상대 회전각도 계산 (목표 - 현재)
    delta_theta = theta_target - current_theta_deg

    # -180~180 범위로 정규화
    delta_theta = (delta_theta + 180) % 360 - 180

    # 추가로 뻗어야 하는 길이
    delta_r = r_target - current_length

    return delta_r, delta_theta

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

    return delta_x*1000, delta_y*1000
    
def list_get(lst, index, default=None):
    return lst[index] if index < len(lst) else default

#paramInt1 = strToRoundedInt(list_get(paramArmControl, 0, "0"))

def normalize_angle(angle):
    """-180 ~ 180도로 정규화"""
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle

def compute_crane_movement(reference: dict, current: dict):
    """
    현재 마커를 기준으로 골드샘플처럼 보이게 하려면
    로봇이 어느 방향으로 회전하고, 팔을 얼마나 뻗어야 하는지 계산
    """

    # 회전: 현재 마커 기준에서 기준 마커 방향 바라보기
    dx = reference['X'] - current['X']
    dy = reference['Y'] - current['Y']
    rotate_angle = normalize_angle(math.degrees(math.atan2(dy, dx)))

    # 전후 거리 차이 = 팔 길이 조정
    arm_distance = (reference['Z'] - current['Z']) * 1000  # m → mm

    return {
        "ROTATE_ANGLE": round(rotate_angle),     # 로봇이 회전해야 할 각도 (deg)
        "ARM_DISTANCE_mm": round(arm_distance)   # 팔을 얼마나 뻗거나 접어야 하는지 (mm)
    }

def compute_movement_and_rotation_from_dict(reference: dict, current: dict, mirrorX = False, mirrorY = True):
    """
    기준 위치와 현재 위치의 마커 정보를 받아서
    회전해야 할 각도, 이동 거리, 전후 오차를 계산함
    """
    # 위치 차이 계산
    dx = current['X'] - reference['X']
    dy = current['Y'] - reference['Y']
    dz = current['Z'] - reference['Z']  # 전후 오차 (Z)

    if mirrorX:
        dx = -dx
    if mirrorY:    
        dy = -dy    
        # 이동 거리
    
    distance = math.sqrt(dx**2 + dy**2) * 1000  # m → mm
    rotate_angle = math.degrees(math.atan2(dy, dx))

    return {
        APRIL_RESULT.ROTATE_ANGLE.name: (360+round(rotate_angle))%360,
        APRIL_RESULT.DISTANCE_mm.name: round(distance),
        APRIL_RESULT.MARGIN_mm.name: round(dz * 1000)
    }
   
def compute_distance_and_rotation_from_dict(reference: dict, current: dict, mirrorX = False, mirrorY = True):
    """
    기준 위치와 현재 위치의 마커 정보를 받아서
    회전해야 할 각도, 이동 거리, 전후 오차를 계산함
    """
    # 위치 차이 계산
    dx = current['X'] - reference['X']
    dy = current['Y'] - reference['Y']
    dz = current['Z'] - reference['Z']  # 전후 오차 (Z)

    if mirrorX:
        dx = -dx
    if mirrorY:    
        dy = -dy    
        # 이동 거리
    
    distance = math.sqrt(dx**2 + dy**2) * 1000  # m → mm
    rotate_angle = math.degrees(math.atan2(dy, dx))

    return round(distance),(270+round(rotate_angle))%360
    
# 예제 사용 FHD 기준
#Y
ref_dict = {
    "MARKER_VALUE": 2, "DIFF_X": 1017.92, "DIFF_Y": 1246.92,
    "X": ref_x, "Y": ref_y, "Z": 0.624784,
    "ANGLE": 179.74, "CAM_ID": 2,
    "cx1": 1059.45, "cy1": 1206.35, "cx2": 974.972, "cy2": 1204.24,
    "cx3": 975.387, "cy3": 1288.48, "cx4": 1059.62, "cy4": 1288.36
}

cur1 = {
    "X": -0.27, "Y": 0.69, "Z": 0.79,
}
cur8 = {
    "X": 0.67, "Y": 0.68, "Z": 0.79,
}

cur3 = {
    "X": 0.48, "Y": -0.48, "Z": 0.79,
}


cur4 = {
    "X": -0.51, "Y": -0.05, "Z": 0.61,
}
#print(compute_distance_and_rotation_from_dict(ref_dict, cur3))
# print(compute_distance_and_rotation_from_dict(ref_dict, cur4))
# print(compute_distance_and_rotation_from_dict(ref_dict, cur1))
# print(compute_distance_and_rotation_from_dict(ref_dict, cur8))
# ref_dict = {
#     "MARKER_VALUE": 9, "DIFF_X": 1017.92, "DIFF_Y": 1246.92,
#     "X": 0.0625937, "Y": 0.575568, "Z": 0.624784,
#     "ANGLE": 179.74, "CAM_ID": 2,
#     "cx1": 1059.45, "cy1": 1206.35, "cx2": 974.972, "cy2": 1204.24,
#     "cx3": 975.387, "cy3": 1288.48, "cx4": 1059.62, "cy4": 1288.36
# }

ref_data = """[{"MARKER_VALUE":9,"DIFF_X":1017.92,"DIFF_Y":1246.92,"X":0.0625937,"Y":0.575568,"Z":0.624784,"ANGLE":179.74,"CAM_ID":2,"cx1":1059.45,"cy1":1206.35,"cx2":974.972,"cy2":1204.24,"cx3":975.387,"cy3":1288.48,"cx4":1059.62,"cy4":1288.36}]"""
# cur_data = """[{"MARKER_VALUE":9,"DIFF_X":831.019,"DIFF_Y":979.336,"X":-0.141635,"Y":0.285001,"Z":0.628085,"ANGLE":144.988,"CAM_ID":2,"cx1":841.144,"cy1":921.526,"cx2":772.695,"cy2":969.107,"cx3":820.776,"cy3":1037.82,"cx4":889.469,"cy4":989.587}]"""

# result = compute_movement_and_rotation(ref_data, cur_data)
# print(result)
def is_between(A, B, C):
    """
    함수 설명:
    주어진 값 C가 A와 B 사이에 있는지를 판단합니다.
    A와 B의 크기 관계와 상관없이, C가 그 사이 범위에 포함되는지 확인합니다.
    
    예시:
        is_between(3, 7, 5) => True   (5는 3과 7 사이)
        is_between(7, 3, 5) => True   (역순이어도 5는 그 사이)
        is_between(3, 7, 8) => False  (8은 범위 밖)
    
    매개변수:
        A (숫자): 범위의 한 쪽 끝
        B (숫자): 범위의 다른 쪽 끝
        C (숫자): 검사 대상 값
    
    반환값:
        bool: C가 A와 B 사이에 있으면 True, 그렇지 않으면 False
    """
    return min(A, B) <= C <= max(A, B)


def is_within_range(A, B, C):
    """
    함수 설명:
    A와 B의 절대값 차이가 C 이하인지를 판단합니다.
    즉, A와 B가 C 범위 이내에 있는지를 확인합니다.
    예를 들어 A와 B가 서로 가까운 위치에 있는지 확인할 때 유용합니다.
    
    예시:
        is_within_range(5, 8, 3) => True   (|5 - 8| = 3 ≤ 3)
        is_within_range(-5, 5, 10) => False (|5 - 5| = 0 ≤ 10 → True 이지만 절대값 기준의 의미 고려)
        is_within_range(-5, 5, 5) => True   (|-5| = 5, |5| = 5 → |5 - 5| = 0 ≤ 5)
    
    주의:
        A와 B의 '절대값 차이'를 기준으로 하므로 방향성은 무시되고 크기만 비교됩니다.
    
    매개변수:
        A (숫자): 첫 번째 값
        B (숫자): 두 번째 값
        C (숫자): 허용 거리 범위 (음수일 수 없다고 가정)
    
    반환값:
        bool: A와 B의 절대값 차이가 C 이하이면 True, 그렇지 않으면 False
    """
    return abs(abs(A) - abs(B)) <= C

def is_within_range_signed(A, B, C):
    return abs(A-B) <= abs(C)

def df_to_dict_int_values(df: pd.DataFrame, key_col, value_col):
    return dict(zip(df[key_col], df[value_col].astype(int)))

def df_to_dict(df: pd.DataFrame, key_col, value_col):
    return dict(zip(df[key_col], df[value_col]))

def insert_or_update_row_to_df(df: pd.DataFrame, new_row, unique_key):
    # 모든 컬럼을 일치시킴 (없으면 생성)
    for column in new_row.keys():
        if column not in df.columns:
            df[column] = 'None'
    for column in df.columns:
        if column not in new_row:
            new_row[column] = 'None'

    # unique_key가 DataFrame에 있고, 값이 중복된다면 업데이트
    if unique_key in df.columns and new_row[unique_key] in df[unique_key].values:
        # 중복되는 행의 인덱스 찾기
        idx = df[df[unique_key] == new_row[unique_key]].index[0]
        for col in df.columns:
            df.at[idx, col] = new_row[col]
    else:
        # 신규 행으로 추가
        df.loc[len(df)] = new_row
    return df

def insert_or_update_row_to_csv(strFileEPC_total, strSplitter, new_row, unique_key):
    # 파일이 존재하는지 확인
    if os.path.exists(strFileEPC_total):
        df = pd.read_csv(strFileEPC_total, sep=strSplitter)
    else:
        df = pd.DataFrame()

    dfNew = insert_or_update_row_to_df(df, new_row, unique_key)
    # 파일 저장
    dfNew.to_csv(strFileEPC_total, sep=strSplitter, index=False)
    
    return dfNew

def insert_row_to_df(df: pd.DataFrame, new_row):
    for column in new_row.keys():
        if column not in df.columns:
            df[column] = 'None'
    
    # 새로운 행에 없는 칼럼이 있다면 'None'으로 값 설정
    for column in df.columns:
        if column not in new_row:
            new_row[column] = 'None'
    
    # 새로운 행 추가
    df.loc[len(df)] = new_row
   
    return df

def insert_row_to_csv(strFileEPC_total, strSplitter, new_row):
    # 파일이 존재하는지 확인
    if os.path.exists(strFileEPC_total):
        # 파일이 존재하면 읽어오기
        df = pd.read_csv(strFileEPC_total, sep=strSplitter)
    else:
        # 파일이 없으면 빈 DataFrame 생성
        df = pd.DataFrame()

    dfNew = insert_row_to_df(df, new_row)
    
    # 변경된 DataFrame을 파일에 저장
    dfNew.to_csv(strFileEPC_total, sep=strSplitter, index=False)    
    return dfNew

def find_nearest_pos(dfTemp, pos_target, nearPoints=1, signedSpd=0, onlyRealNode=False):
    try:
        # DataFrame 복사 및 diff 계산
        df = dfTemp.copy()

        if 'POS' not in df.columns:
            return []

        # onlyRealNode 조건에 따라 EPC 필터링
        if onlyRealNode and 'EPC' in df.columns:
            df = df[df['EPC'].astype(str).str.contains('NOTAG')]

        df['diff'] = (df['POS'] - pos_target).abs()

        # 방향 조건에 따른 필터링
        if signedSpd > 0:
            df = df[df['POS'] < pos_target]
        elif signedSpd < 0:
            df = df[df['POS'] > pos_target]

        # 가까운 값 정렬 및 추출
        nearest_rows = df.nsmallest(nearPoints, 'diff')
        result = nearest_rows.to_dict(orient='records')

        return result
    except Exception:
        return []



def GetNodeDicFromPos(dfTemp, pos_target : int, isRealNode = False):
    lsResult = find_nearest_pos(dfTemp,pos_target, nearPoints=1,signedSpd=0,onlyRealNode=isRealNode)
    if lsResult:
        return lsResult[0]
    else:
        return {}

def find_key_by_nested_value(d, target_value):
    for key, value_list in d.items():
        if target_value in value_list:
            return key
    return None  # 못 찾으면 None 반환

def is_equal(a, b):
    return str(a) == str(b)

def add_or_update_row(filepath: str, new_row: dict, sep: str = '\t', tableid_key = 'TABLEID'):
    """
    CSV/TSV 파일에 dict를 추가하거나 업데이트합니다.
    
    - sep: 컬럼 구분자 (기본은 탭 '\t')
    - 파일이 없거나 컬럼이 다르면 새로 생성
    - TABLEID 중복 시 해당 row를 업데이트
    """
    new_row = {tableid_key: new_row[tableid_key].upper(), **{k: v for k, v in new_row.items() if k != tableid_key}}
    bOK = False
    if os.path.exists(filepath):
        try:
            df = pd.read_csv(filepath, sep=sep)
            if set(df.columns) != set(new_row.keys()):
                print(f"[경고] 컬럼 불일치")
                return bOK
                # os.remove(filepath)
                # df = pd.DataFrame([new_row])
            else:
                new_row = {col: new_row[col] for col in df.columns}
                # 대소문자 무시하고 매칭
                idx = df[df[tableid_key].str.upper() == new_row[tableid_key].upper()].index

                if not idx.empty:
                    # 기존 행 업데이트
                    for col in new_row:
                        df.loc[idx, col] = new_row[col]
                else:
                    # 새 행 추가
                    df = pd.concat([df, pd.DataFrame([new_row])], ignore_index=True)
                # # TABLEID 중복 시 해당 row 업데이트
                # if df[tableid_key].str.upper().isin([new_row[tableid_key]]).any():
                #     print(df)
                #     print(new_row)
                #     df.loc[df[tableid_key].str.upper() == new_row[tableid_key], :] = new_row
                #     print(f"[업데이트] TABLEID = {new_row[tableid_key]} 값 업데이트됨.")
                # else:
                #     df = pd.concat([df, pd.DataFrame([new_row])], ignore_index=True)
                #     print(f"[추가] TABLEID = {new_row[tableid_key]} 새로 추가됨.")
            bOK = True
        except Exception as e:
            print(f"[에러] 파일 읽기 실패: {e}.")
            # os.remove(filepath)
            # df = pd.DataFrame([new_row])
    else:
        df = pd.DataFrame([new_row])
        print(f"[생성] 새 파일 생성 및 첫 row 추가.")
        bOK = True
    # 저장
    df.to_csv(filepath, index=False, sep=sep)
    print(f"[완료] 파일에 저장되었습니다: {filepath}")
    return bOK

def update_motor_spd_by_time(df: pd.DataFrame, target_mbid, ref_mbid, cur_pos, pulses_per_rev=10000, marginTime=0):
    """
    DataFrame에서 target_mbid의 SPD를 ref_mbid의 TIME 기준 + marginTime 만큼 조정하여 계산 및 수정합니다.

    Parameters:
        df (pd.DataFrame): 원본 DataFrame
        target_mbid (int): SPD를 수정할 대상 MBID
        ref_mbid (int): 기준이 될 MBID
        cur_pos (int): 대상 MBID의 현재 위치값
        pulses_per_rev (int): 한 바퀴당 펄스 수
        marginTime (float): 기준 시간에 더하거나 빼는 여유 시간 (초 단위, 기본값 0)

    Returns:
        pd.DataFrame: SPD와 TIME이 수정된 새로운 DataFrame (예외 시 원본 그대로 반환)
    """
    try:
        df_updated = df.copy()

        # 참조 TIME 추출 및 변환
        ref_time_series = df_updated[df_updated['MBID'] == ref_mbid]['TIME']
        if ref_time_series.empty:
            return df
        ref_time = float(ref_time_series.values[0])

        # marginTime 고려한 동작 시간 계산 (최소 0.001초로 제한)
        target_time = max(ref_time + marginTime, 0.001)

        # 대상 POS 추출 및 변환
        target_pos_series = df_updated[df_updated['MBID'] == target_mbid]['POS']
        if target_pos_series.empty:
            return df
        target_pos = int(target_pos_series.values[0])

        # 이동 펄스 계산
        delta_pulse = abs(target_pos - cur_pos)

        # RPM 계산
        new_spd = int((delta_pulse * 60) / (pulses_per_rev * target_time))

        # 값 수정
        df_updated.loc[df_updated['MBID'] == target_mbid, 'SPD'] = new_spd
        df_updated.loc[df_updated['MBID'] == target_mbid, 'TIME'] = float(target_time)

        return df_updated

    except Exception:
        return df

def remove_consecutive_duplicates(arr):
    if not arr:
        return []
    
    result = [arr[0]]  # 첫 번째 값은 항상 포함
    for i in range(1, len(arr)):
        if arr[i] != arr[i - 1]:
            result.append(arr[i])
    return result

# def get_logger(func_name):
#     """함수 이름을 기반으로 개별 로그 파일을 설정하는 함수"""
#     log_dir = "/root/Downloads/"
#     os.makedirs(log_dir, exist_ok=True)  # logs 디렉터리 생성

#     logger = logging.getLogger(func_name)
#     logger.setLevel(logging.INFO)

#     # 로그 파일 핸들러 설정
#     log_file = os.path.join(log_dir, f"{func_name}.log")
#     file_handler = logging.FileHandler(log_file, encoding='utf-8')
#     file_handler.setLevel(logging.INFO)

#     # 로그 포맷 설정
#     formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
#     file_handler.setFormatter(formatter)

#     # 중복 핸들러 방지 (이미 추가된 경우 추가하지 않음)
#     if not logger.handlers:
#         logger.addHandler(file_handler)

#     return logger
# import os
# import logging
# import json
class UbuntuEnv(Enum):
    ROS_MASTER_URI = auto()
    CONFIG = auto()
    SCR_DIR = auto()
    QBI = auto()
    ITX = auto()
    COMMON = auto()
    ROS_HOSTNAME = auto()


def GetUbutuParam(variable_name):
    if variable_name in os.environ:
        my_variable = os.environ[variable_name]
        return my_variable
    else:
        return None


def get_logger(func_name):
    """함수 이름을 기반으로 개별 로그 파일을 설정하고, pretty 로그 메서드 포함"""
    log_dir = f"{GetUbutuParam(UbuntuEnv.SCR_DIR.name)}/"
    os.makedirs(log_dir, exist_ok=True)

    logger = logging.getLogger(func_name)
    logger.setLevel(logging.INFO)

    log_file = os.path.join(log_dir, f"{func_name}.log")
    file_handler = logging.FileHandler(log_file, encoding='utf-8')
    file_handler.setLevel(logging.INFO)

    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
    file_handler.setFormatter(formatter)

    if not logger.handlers:
        logger.addHandler(file_handler)

    # 🔽 pretty 메서드 추가
    def pretty(url, result, response):
        logger.info("=" * 100)
        logger.info(f"URL: {url}")
        logger.info(f"RESULT: {result}")
        if isinstance(response, (dict, list)):
            pretty_response = json.dumps(response, indent=4, ensure_ascii=False)
            logger.info("RESPONSE:\n" + pretty_response)
        else:
            logger.info(f"RESPONSE: {response}")
        logger.info("=" * 100 + "\n")

    logger.pretty = pretty  # 메서드 바인딩

    return logger

logger_ard = get_logger(f'blb_ard')
logger_local = get_logger(f'blb_local')
logger_android = get_logger(f'blb_android')
logger_svr = get_logger(f'blb_svr')
logger_api = get_logger(f'blb_api')
logger_motor = get_logger(f'blb_motor')

def get_missing_or_next(lst):
    # 리스트가 비어있으면 1 반환
    if not lst:
        return 1
    
    # 리스트 정렬
    lst = sorted(lst)
    
    # 최소값이 1보다 크면 1 반환
    if lst[0] > 1:
        return 1
    
    # 1부터 최대값까지 확인하여 누락된 값 찾기
    for i in range(len(lst) - 1):
        if lst[i + 1] - lst[i] > 1:
            return lst[i] + 1
    
    # 누락된 값이 없으면 최대값 + 1 반환
    return lst[-1] + 1

def get_key_by_value(d, target_value):
    
    """
    주어진 딕셔너리 d에서 특정 값(target_value)에 해당하는 첫 번째 키를 반환합니다.

    매개변수:
        d (dict): 검색할 딕셔너리.
        target_value: 찾고자 하는 값.

    반환값:
        해당 값에 대응되는 첫 번째 키를 반환하고,
        값이 존재하지 않으면 None을 반환합니다.

    예:
        >>> get_key_by_value({'a': 1, 'b': 2, 'c': 1}, 1)
        'a'
        >>> get_key_by_value({'a': 1, 'b': 2}, 3)
        None
    """
    try :
        # 딕셔너리의 모든 키-값 쌍을 순회하며 값이 target_value인 키를 찾습니다.
        return next(k for k, v in d.items() if v == target_value)
    except Exception as e:
        return None


def get_value_by_key_fromDF(df : pd.DataFrame , key_column, key_value, target_column):
    row = df[df[key_column] == key_value]
    if not row.empty:
        value = row.iloc[0][target_column]
        # numpy 타입이면 Python 기본 타입으로 캐스팅
        if isinstance(value, (np.integer, np.int64)):
            return int(value)
        elif isinstance(value, (np.floating, np.float64)):
            return float(value)
        else:
            return value
    else:
        return None

def rate_limited(interval):
    def decorator(func):
        last_called = [0]  # 리스트로 감싸서 클로저에서 값 변경 가능

        def wrapper(*args, **kwargs):
            current_time = time.time()
            if current_time - last_called[0] >= interval:  # 지정된 시간 경과 여부 확인
                last_called[0] = current_time
                return func(*args, **kwargs)
        return wrapper
    return decorator

def log_arguments(func):
    def wrapper(*args, **kwargs):
        print(f"Calling {func.__name__} with:")
        if args:
            for i, arg in enumerate(args, start=1):
                print(f"  Arg {i}: {arg} (Type: {type(arg).__name__})")
        if kwargs:
            for key, value in kwargs.items():
                print(f"  {key}: {value} (Type: {type(value).__name__})")
        result = func(*args, **kwargs)
        return result
    return wrapper

def GetCounterDirection(direction):
    dirCur = str(direction).upper()
    if dirCur == 'N':
        return 'S'
    if dirCur == 'S':
        return 'N'
    if dirCur == 'E':
        return 'W'
    if dirCur == 'W':
        return 'E'
    return None

def GetNewPos(old_X, old_Y,directionNew,old_distance):
    #   old_X = dicRecord[APIBLB_FIELDS_INFO.st_xval.name]
    #   old_Y = dicRecord[APIBLB_FIELDS_INFO.st_yval.name]
    if directionNew == 'N':
        old_Y = old_Y+old_distance
    if directionNew == 'S':
        old_Y = old_Y-old_distance
    if directionNew == 'E':
        old_X = old_X+old_distance
    if directionNew == 'W':
        old_X = old_X-old_distance
    return old_X,old_Y

def isAlmostSame(valueA, valueB, thresholdPercentage=0.1):
    # valueA와 valueB의 절대값
    abs_valueA = abs(valueA)
    abs_valueB = abs(valueB)

    # valueA와 valueB 중 더 작은 값을 기준으로 오차범위 계산
    max_value = min(abs_valueA, abs_valueB)
    threshold = max_value * thresholdPercentage

    # 두 값의 절대 차이 계산
    difference = abs(abs_valueA - abs_valueB)

    # 차이가 오차범위 이내에 있으면 True, 아니면 False 리턴
    return difference <= threshold

def count_dicts(lst):
    return sum(isinstance(item, dict) for item in lst)
  
def getEnumInstanceFromValue(cls, value):
    # 만약 value가 문자열이라면 정수로 변환 시도
    if isinstance(value, str):
        try:
            value = int(value)
        except ValueError:
            raise ValueError(f"Invalid literal for int() with base 10: '{value}'")
            
    for member in cls:
        if member.value == value:
            return member
    raise ValueError(f"No member found for value: {value}")

def getEnumInstanceFromName(cls, nameStr):
    for member in cls:
        if member.name == str(nameStr):
            return member
    raise ValueError(f"No member found for name: {nameStr}")

def get_list_depth(lst):
    # 만약 입력값이 리스트가 아니면 중첩이 없다고 간주하고 깊이를 0으로 반환
    if not isinstance(lst, list):
        return 0
    # 리스트 안의 모든 요소에 대해 깊이를 계산하고, 그 중 가장 깊은 값을 반환
    else:
        return 1 + max(get_list_depth(item) for item in lst) if lst else 1  # 빈 리스트는 깊이가 1
  
def log_all_frames(logmsg='',max_frames=3):
    if logmsg is None:
        logmsg = 'None'
    # 현재 스택의 모든 프레임 정보를 가져옵니다.
    stack = inspect.stack()
    
    # 첫 번째 프레임(현재 함수 자신)을 제외합니다.
    stack = stack[1:]
    
    # 프레임 정보를 역순으로 순회합니다 (가장 최근의 호출부터).
    frame_names = [frame.function for frame in reversed(stack)]
    
    # max_frames가 지정되었다면, 해당 수만큼만 프레임을 사용합니다.
    if max_frames >= 1:
        frame_names = frame_names[-max_frames:]
    
    # 함수 이름들을 콜론으로 구분하여 연결합니다.
    log_message = ":".join(frame_names)
    # rospy를 사용하여 로그를 출력합니다.
    returnMsg = ''
    if logmsg == '':
        returnMsg = f'{log_message}'        
    else:
        returnMsg=f'{logmsg}:{log_message}'
    rospy.loginfo(returnMsg)
    return returnMsg

def control_system_sound(mute: bool):
    """
    Mute or unmute the system sound using PulseAudio based on the mute flag.
    
    Args:
    mute (bool): If True, mutes the sound. If False, unmutes the sound.
    """
    try:
        # Determine the mute state based on the boolean input
        mute_state = '1' if mute else '0'
        
        # Run the pactl command to set the mute state
        subprocess.run(['pactl', 'set-sink-mute', '@DEFAULT_SINK@', mute_state], check=True)
        
        if mute:
            print("System sound muted.")
        else:
            print("System sound unmuted.")
    except subprocess.CalledProcessError as e:
        print(f"An error occurred while controlling the sound: {e}")

# # Example usage
# control_system_sound(True)  # To mute the sound
# control_system_sound(False) # To unmute the sound


def LoadJsonFile(strFilePath):
  dicCaliFinalPos = None
  if isFileExist(strFilePath):
    try:    
      with open(strFilePath, "r") as f:
        dicCaliFinalPos = json.load(f)
    except Exception as e:
        print(e)
  return dicCaliFinalPos

def visualize_point_cloud(cloud):
    visual = pcl.pcl_visualization.CloudViewing()
    visual.ShowMonochromeCloud(cloud)
    visual.Spin()
    
def get_connected_monitors():
    drm_dir = "/sys/class/drm/"
    connected_monitors = []

    for entry in os.listdir(drm_dir):
        if "card" in entry and "-DP-" in entry or "-HDMI-" in entry:
            status_path = os.path.join(drm_dir, entry, "status")
            if os.path.isfile(status_path):
                with open(status_path, 'r') as f:
                    status = f.read().strip()
                    if status == "connected":
                        connected_monitors.append(entry)

    return connected_monitors

MAX_INT = sys.maxsize
MIN_INT = -sys.maxsize - 1


def convert_angle(angle):
    """
    각도를 -180~180 범위에서 0~360 범위로 변환하는 함수

    :param angle: 변환할 각도 (단위: 도)
    :return: 변환된 각도 (단위: 도)
    """
    if angle < 0:
        return 360 + angle
    else:
        return angle

def has_common_element(arr1, arr2):
    """
    두 배열에 공통 원소가 하나라도 있으면 True 를 리턴하는 함수
    """
    # 두 배열을 집합으로 변환
    set1 = set(arr1)
    set2 = set(arr2)
    
    # 공통 원소가 있는지 확인
    return not set1.isdisjoint(set2)

def calculate_distance_and_angle_from_currentCoordinates(distance, angle_degrees, target_x, target_y):
    """
    현재 위치와 목표 지점을 기준으로 이동할 거리와 각도를 계산하는 함수.
    
    :param distance: 현재 위치의 distance
    :param angle_degrees: 현재 위치의 각도
    :param target_x: 목표 지점의 x 좌표
    :param target_y: 목표 지점의 y 좌표
    :return: (거리, 각도) 각도는 도 단위로 반환
    """
    current_x, current_y = calculate_coordinates(distance, angle_degrees)
    return calculate_distance_and_angle_from_currentXY(current_x, current_y, target_x, target_y)


def calculate_distance_and_angle_from_currentXY(current_x, current_y, target_x, target_y):
    """
    현재 위치와 목표 지점을 기준으로 이동할 거리와 각도를 계산하는 함수.
    
    :param current_x: 현재 위치의 x 좌표
    :param current_y: 현재 위치의 y 좌표
    :param target_x: 목표 지점의 x 좌표
    :param target_y: 목표 지점의 y 좌표
    :return: (거리, 각도) 각도는 도 단위로 반환
    """
    # x와 y의 차이 계산
    delta_x = target_x - current_x
    delta_y = target_y - current_y
    
    # 거리 계산 (피타고라스의 정리)
    distance = math.sqrt(delta_x**2 + delta_y**2)
    
    # 각도 계산 (atan2 함수 사용)
    angle_radians = math.atan2(delta_y, delta_x)
    angle_degrees = math.degrees(angle_radians)
    
    return distance, angle_degrees

def calculate_coordinates(distance, angle_degrees, x1=0,y1=0):
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

def calculate_length_and_angle(r, x, y):
    """
    r: Initial length of the rod in mm
    x: Relative x coordinate to move to in mm
    y: Relative y coordinate to move to in mm
    
    Returns the adjusted length of the rod and the rotation angle in degrees.
    """
    # Calculate the new length required to reach (x, y)
    new_length = math.sqrt(x**2 + y**2)
    
    # Calculate the angle in radians
    angle_rad = math.atan2(y, x)
    
    # Convert angle to degrees
    angle_deg = math.degrees(angle_rad)
    
    return new_length, angle_deg

# # Example usage:
# calculate_length_and_angle(100, 50, 50)  # r=100mm, move to (50, 50) mm


def calculate_distance_and_angle(x, y):
    """
    주어진 x, y 좌표로 이동할 때 필요한 거리와 각도를 계산하는 함수.
    
    :param x: x 좌표
    :param y: y 좌표
    :return: (거리, 각도) 각도는 라디안 단위와 도 단위 모두 반환
    """
    # 거리 계산 (피타고라스의 정리)
    distance = round(math.sqrt(x**2 + y**2))
    
    # 각도 계산 (atan2 함수 사용)
    angle_radians = math.atan2(y, x)
    angle_degrees = convert_angle(round(math.degrees(angle_radians)))
    return distance, angle_degrees
    #return distance, angle_radians, angle_degrees

# # 예제 사용
# x = 3
# y = 4
# distance, angle_radians, angle_degrees = calculate_distance_and_angle(x, y)
# print(f"Distance: {distance}")
# print(f"Angle (radians): {angle_radians}")
# print(f"Angle (degrees): {angle_degrees}")

def extract_hostname_from_uri(uri):
    # URI 파싱
    parsed_uri = urlparse(uri)

    # 네트워크 위치 부분(호스트네임 또는 IP) 추출
    hostname = parsed_uri.hostname
    return hostname


def check_ros_master():
    try:
        rosgraph.Master("/rosout").getPid()  # '/rostopic'은 임의의 노드 이름입니다.
        print("ROS Master is running.")
        return True
    except Exception as e:
        print("Failed to contact ROS Master.")
        print(e)
        return False


# from SPG_Keys import *
# from tta_spg.srv import *

sDivFieldColon = ":"
sDivItemComma = ","
sDivEmart = "`"
sDivSlash = "/"
sDivSemiCol = ";"
sDivTab = '\t'
roundPulse = 10000
ZERO_ALMOST = 1e-6
""" 각 노드에 내리는 명령어 """


class COMMON_CMD(Enum):
    ID = auto()
    CMD = auto()
    VALUE = auto()


class Config_Common(Enum):
    DEVICE_ID = auto()


class RFID_RESULT(Enum):
    EPC = auto()
    TID = auto()
    SEQ = auto()
    DEVID = auto()
    RSSI = auto()
    TIMESTAMP = auto()
    inventoryMode = auto()
    status = auto()


class StrParser(Enum):
    sDivColon = sDivFieldColon
    sDivComma = sDivItemComma
    sDivEmart = sDivEmart
    sDivSlash = sDivSlash
    sDivSemiCol = sDivSemiCol
    sKey = "CMD"
    sValue = "PARAM"


def GetLastString(dataStr: str, splitter):
    callData = None
    callData = dataStr.split(splitter)[-1]
    return callData


def GetROSString(dataStr):
    callData = String()
    callData.data = dataStr
    return callData


def calculate_offset(image_width, image_height, points):
    # 이미지 중심 좌표 계산
    image_center_x = image_width / 2
    image_center_y = image_height / 2

    # 사각형의 중심 좌표 계산
    rect_center_x = sum(point[0] for point in points) / len(points)
    rect_center_y = sum(point[1] for point in points) / len(points)

    # 중심 사이의 거리 차이 계산
    diff_x = rect_center_x - image_center_x
    diff_y = rect_center_y - image_center_y

    # 백분율로 변환
    percent_diff_x = (diff_x / image_width) * 100
    percent_diff_y = (diff_y / image_height) * 100

    return percent_diff_x, percent_diff_y


def GetMasterIP():
  ipAddr = extract_hostname_from_uri(GetUbutuParam(UbuntuEnv.ROS_MASTER_URI.name))
  return ipAddr

def GetMachineStr():
    return GetUbutuParam(UbuntuEnv.CONFIG.name)


def CheckAllKeysExist(className, dicTmp: dict) -> bool:
    for s in className:
        sCurrentTmp = s.name
        if dicTmp.get(sCurrentTmp, None) is None:
            return False
    return True


def is_json(myjson):
    try:
        json.loads(myjson)
    except Exception as e:
        return False
    return True

def get_hostname(ip=None):
    if ip is None:
      host = os.uname()[1]
      # print('HOST:' + host)
      return host
    else:
      slave_hostname = socket.gethostbyaddr(ip)[0]
      return slave_hostname
  
def isFileExist(filePath):
    file_path = Path(filePath)
    return file_path.is_file()

def isServiceExist(serviceName):
    service_list = rosservice.get_service_list()
    return serviceName in service_list


""" 리턴값을 2개가지는 함수 예제"""


def splitSignedTest(value: int):
    # strHex = TwosComp32(value).rjust(8,'0')
    # strUpperHex = strHex[0:4]
    # strBottomHex = strHex[4:]
    signed32Hex = signedIntToHex(value, 32).rjust(8, "0")
    strUpperHex = signed32Hex[0:4]
    strBottomHex = signed32Hex[4:]
    return int(strUpperHex, 16), int(strBottomHex, 16)


def getCMDDictFromParams(strID: str, strCMD: str, strVALUE: str):
    dictReturn = {}
    dictReturn[COMMON_CMD.ID.name] = strID
    dictReturn[COMMON_CMD.CMD.name] = strCMD
    dictReturn[COMMON_CMD.VALUE.name] = strVALUE
    return dictReturn


def getCMDStrFromParams(
    strID: str, strCMD: str, strVALUE: str, keyValueSpliter=":", itemSpliter=","
):
    dictReturn = getCMDDictFromParams(strID, strCMD, strVALUE)
    strReturn = getStr_fromDic(dictReturn, keyValueSpliter, itemSpliter)
    return strReturn


def getCMDDictFromStr(dataStr: str, keyValueSpliter=":", itemSpliter=","):
    try:
        dicResultRaw = getDic_strArr(dataStr, keyValueSpliter, itemSpliter)
        if set(COMMON_CMD.__members__.keys()) == set(dicResultRaw.keys()):
            return dicResultRaw
        else:
            rospy.loginfo(dataStr)
            return None
    except Exception as e:
        rospy.loginfo(e)


def getDictFromDF(df: pd.DataFrame, key: str, value: str):
    strExp = f"{key} == '{value}'"
    returnDict = df.query(strExp).to_dict("list")
    finalDict = {}
    for key, value in returnDict.items():
        if len(value) > 0:
            finalDict[key] = value[0]
    return finalDict


def try_parse_int(text):
    try:
        return int(text)
    except:
        return None


def getROS_Header(frame_id_str):
    msgTmp = Header()
    msgTmp.frame_id = frame_id_str
    return msgTmp


def getROS_Param(param_name):
    if rospy.has_param(param_name):
        return rospy.get_param(param_name)
    return None


def getLines_FromFile(filename):
    file_list = getStr_FromFile(filename).splitlines()
    return file_list


def getStr_FromFile(filename):
    with open(filename, "r") as f:
        data = f.read()
    return data


def getDic_FromFile(filename, spliter=sDivTab):
    file_list = getLines_FromFile(filename)
    dicReturn = {}
    if len(file_list) == 1 and is_json(file_list[0]):
      dicReturn = json.loads(file_list[0])
    else:
      for i in file_list:
          iCurrent = i.split(spliter)
          if len(iCurrent) > 1:
              dicReturn[iCurrent[0]] = iCurrent[1]
              # print(iCurrent[0])
    return dicReturn

def save_data_pickle(dataTmp, file_path):
    with open(file_path, 'wb') as file:
        pickle.dump(dataTmp, file)
    rospy.loginfo(f'Data saved to {file_path}')

def load_data_pickle(file_path):
    if os.path.exists(file_path):
        with open(file_path, 'rb') as file:
            return pickle.load(file)
    else:
        return None


""" 2022-10-30 개발중 , Config 저장용도 """


def saveDic_ToFile(dic, filename, spliter, isJson = False):
    saveStr = ''
    if isJson:
      saveStr = json.dumps(dic)
    else:
      saveStr = getStr_fromDicPrivate(dic, spliter, "\n")
    
    with open(filename, "w") as file:  # hello.txt 파일을 쓰기 모드(w)로 열기
        file.write(saveStr)


def getDic_strArr(strTmp, spliterItemValue, spliterLine):
    dicReturn = {}
    # if spliterItemValue == None and is_json(strTmp):
    #     dicReturn = json.loads(strTmp)
    # elif spliterItemValue != None and spliterLine != None:
    file_list = strTmp.split(sep=spliterLine)
    for i in file_list:
        iCurrent = i.split(spliterItemValue, 1)
        if len(iCurrent) > 1:
            dicReturn[iCurrent[0]] = iCurrent[1]
            # print(iCurrent[0])
    return dicReturn


def getStr_fromDicPrivate(dicTmp, spliterItemValue, spliterLine):
    strReturn = ""
    for sKey in dicTmp.keys():
        sValue = dicTmp[sKey]
        strReturn += f"{sKey}{spliterItemValue}{sValue}{spliterLine}"

    return strReturn[0:-1]

def dictAppend(dictOrg, mbid, k,v):
  dictTmp = dictOrg.get(mbid, None)
  if dictTmp == None:
    dictOrg[mbid] = {}
  dictOrg[mbid][k] = v

def get_last_field_value(df : pd.DataFrame, field_name : str):
    if df.empty:
        return None  # 또는 예외 처리
    try:
        return df.iloc[-1][field_name]  
    except:
        return None    
    

def get_last_value_for_key(df: pd.DataFrame, key1_col, key1_value, key2_col):
    filtered = df[df[key1_col] == key1_value]
    if filtered.empty:
        return None
    return filtered[key2_col].iloc[-1]
  
def isTrue(val):
    if val is None:
        return False
    valueStr = str(val).strip().upper()
    return valueStr.startswith(('T', '1', 'Y','ON'))

def getStr_fromDic(
    dicTmp, spliterItemValue, spliterLine, head_or_tail=None, padString=None
):
    strReturn = ""
    if head_or_tail is None:
        return getStr_fromDicPrivate(dicTmp, spliterItemValue, spliterLine)

    for sKey in dicTmp.keys():
        sValue = dicTmp[sKey]
        sKeyCurrent = ""
        if head_or_tail:
            sKeyCurrent = sKey + padString
        else:
            sKeyCurrent = padString + sKey
        strReturn += f"{sKeyCurrent}{spliterItemValue}{sValue}{spliterLine}"

    return strReturn[0:-1]

def getMergedDic(dic1, dic2):
    dic3 = {}
    if len(dic1) > 0:
        dic3.update(dic1)
    if len(dic2) > 0:
        dic3.update(dic2)
    return dic3

def getListedDic(dic1):
    lsTmp = []
    lsTmp.append(dic1)
    return lsTmp

def getValueFromMap(mapTmp, sKey):
    chkHasKey = sKey in mapTmp.keys()
    if chkHasKey == False:
        return ""
    else:
        return mapTmp[sKey]


def getValueFromMapEx(mapTmp, sKeyList):
    lsReturn = []
    for sKey in sKeyList:
        chkHasKey = sKey in mapTmp.keys()
        if chkHasKey == False:
            lsReturn.append(None)
        else:
            lsReturn.append(mapTmp[sKey])
    return lsReturn

def getCurrentTime(spliter=":", includeDate=False):
    now = getDateTime()
    strTime = now.strftime("%H:%M:%S.%f")
    milliseconds = strTime[-6:-3].zfill(3)  # 밀리초를 3자리로 맞춤
    strTime = f"{strTime[:-7]}.{milliseconds}"
    if includeDate:
        strTime = f"{getCurrentDate(spliter)} {strTime}"
    if spliter != None:
        return strTime.replace(":", spliter)
    return strTime


def getCurrentDate(spliter="-"):
    strDate = getDateTime().strftime(f"%Y{spliter}%m{spliter}%d")
    return strDate


"""
def dijkstraEx(dicMap : Dict[str,int], total_length : int, start : str, end : str):
  if dicMap == None:
    dicMap = {'0' : 0, '8':150, '44':450, '57':650}
  dicReverse = {}
  for sKey in dicMap.keys():
    dicReverse[sKey] = dicMap[sKey] - total_length
  dicReverse['0'] = 0
  cw_length = dicMap[end] - dicMap[start]
  ccw_length = dicReverse[end] - dicReverse[start]
  returnVal = cw_length if abs(cw_length) < abs(ccw_length) else ccw_length
  return returnVal

def dijkstra(graph, start, end):
    if graph == None:
      graph = {
    '0': {'8': 150, '57': 200},
    '8': {'0': 150, '44': 300},
    '44': {'8': 300, '57': 200},
    '57': {'0': 200, '44': 200}
}
    distances = {node: [float('inf'), start] for node in graph}
    distances[start] = [0, start]

    queue = []

    heapq.heappush(queue, [distances[start][0], start])

    while queue:
        cur_route_distance, cur_node = heapq.heappop(queue)

        if distances[cur_node][0] < cur_route_distance:
            continue

        for adjacent, weight in graph[cur_node].items():
            distance = cur_route_distance + weight

            if distance < distances[adjacent][0]:
                distances[adjacent] = [distance, cur_node]
                heapq.heappush(queue, [distance, adjacent])

    path = end
    path_output = end + '->'
    while distances[path][1] != start:
        path_output += distances[path][1] + '->'
        path = distances[path][1]
    path_output += start
    print(path_output)
    return distances

# def dijkstra(graph, start):
#     distances = {node: float('inf') for node in graph}  # start로 부터의 거리 값을 저장하기 위함
#     distances[start] = 0  # 시작 값은 0이어야 함
#     queue = []
#     heapq.heappush(queue, [distances[start], start])  # 시작 노드부터 탐색 시작 하기 위함.

#     while queue:  # queue에 남아 있는 노드가 없으면 끝
#         current_distance, current_destination = heapq.heappop(queue)  # 탐색 할 노드, 거리를 가져옴.

#         if distances[current_destination] < current_distance:  # 기존에 있는 거리보다 길다면, 볼 필요도 없음
#           continue

#     for new_destination, new_distance in graph[current_destination].items():
#         distance = current_distance + new_distance  # 해당 노드를 거쳐 갈 때 거리
#         if distance < distances[new_destination]:  # 알고 있는 거리 보다 작으면 갱신
#             distances[new_destination] = distance
#             heapq.heappush(queue, [distance, new_destination])  # 다음 인접 거리를 계산 하기 위해 큐에 삽입

#     return distances
"""

""" 인자로 밀리초 및 datetime 개체 넘기면 현재 시간에서 해당시간을 넘겼으면 TRUE, 아직 안되었으면 FALSE """

def strToRoundedInt(strFloatValue):
    try:
        paramInt1 = int(round(float(strFloatValue)))
        return paramInt1
    except:
        return MIN_INT    

def isTimeExceeded(lastLogTime: datetime, iMilliSeconds: int):
    td = getDateTime() - lastLogTime
    tds = td.total_seconds() * 1000
    if tds > iMilliSeconds:
        return True
    else:
        return False


""" 10진수 혹은 16진수인지 판단 """


def if_Number(s):
    return if_integer(s) or if_hex(s)


def if_integer(s):
    reg_exp = "[-+]?\d+$"
    return re.match(reg_exp, s) is not None


def if_hex(s):
    s = s.upper()
    if s.startswith("0X"):
        s = s[2:]
    return all(c in string.hexdigits for c in s)


""" 숫자인지 판단 (float 로 변환시도하여 확인)"""


def is_digit(str):
    try:
        tmp = float(str)
        return True
    except ValueError:
        return False


def mapRangeExp(y, in_min, in_max, out_min, out_max, k):
    """
    지수 매핑 함수로 입력 범위 내의 값을 출력 범위로 변환한다.
    
    Parameters:
    y (float): 변환하려는 원래 값
    in_min (float): 입력 범위의 최소값
    in_max (float): 입력 범위의 최대값
    out_min (float): 출력 범위의 최소값
    out_max (float): 출력 범위의 최대값
    k (float): 지수 변환의 강도를 조절하는 계수
    
    Returns:
    float: 변환된 값
    """    
    # y의 범위를 0에서 1 사이로 정규화
    norm_y = (y - in_min) / (in_max - in_min)
    
    # 정규화된 값을 지수 함수에 적용하고 정규화된 입력 범위로 변환
    #exp_value = (math.exp(norm_y * k * math.log(math.e)) - 1) / (math.e - 1)
    exp_value = (math.exp(norm_y * k) - 1) / (math.exp(k) - 1)
    # 입력 범위로 매핑
    output_value = exp_value * (out_max - out_min) + out_min
    
    return output_value

def mapRangeLog(x, input_min, input_max, output_min, output_max, k):
    # x의 범위를 0에서 1 사이로 정규화
    norm_x = (x - input_min) / (input_max - input_min)
    
    # 정규화된 값을 로그 함수에 적용하고 정규화된 출력 범위로 변환
    log_value = k * math.log(norm_x * (math.e - 1) + 1)
    
    # 출력 범위로 매핑
    output = (log_value - k * math.log(1)) / (k * math.log(math.e)) * (output_max - output_min) + output_min
    
    return output


""" 아두이노의 맵 함수를 파이썬으로 구현
iRPM = int(map(iSpd,0,100, 0,3000))
0~100 으로 iSpd 입력이 들어가면 0~3000 사이의 값으로 리턴한다.
"""
def mapRange(x, input_min, input_max, output_min, output_max):
    try:
        return (x - input_min) * (output_max - output_min) / (
            input_max - input_min
        ) + output_min  # map()함수 정의
    except:
        return 0

def twosComplement_hex(hexval):
    bits = 16  # Number of bits in a hexadecimal number format
    val = int(hexval, bits)
    if val & (1 << (bits - 1)):
        val -= 1 << bits
    return val

#이 코드는 new_rpm 값을 spd_cur_arm1 절대값에서 adjust_rate 이상 벗어나지 않게 조정합니다. adjust_rpm 함수는 new_rpm과 spd_cur_arm1 값을 입력받아 조정된 new_rpm 값을 반환합니다.
def adjust_rpm(new_rpm, spd_cur_arm1, adjust_rate):
    spd_cur_arm1_abs = abs(spd_cur_arm1)
    max_variation = adjust_rate * spd_cur_arm1_abs

    # If new_rpm is more than 50% variation from spd_cur_arm1, adjust it
    if abs(new_rpm - spd_cur_arm1_abs) > max_variation:
        if new_rpm > spd_cur_arm1_abs:
            new_rpm = spd_cur_arm1_abs * (1+adjust_rate)
        else:
            new_rpm = spd_cur_arm1_abs * adjust_rate

    return abs(round(new_rpm))
  

def unsigned32(n):
    return n & 0xFFFFFFFF


"""
unsignedInt32 를 부호있는 32비트 숫자로 변환한다
"""


def signed32(lv):
    return TwosComp32(lv)
    # return unpack('l', pack('L', lv & 0xffffffff))[0]
    # return -(0xffffffff+1 - lv)


def TwosComp32(n):
    return n - 0x100000000 if n & 0x80000000 else n


"""
정수값을 16/32/64 비트 부호 있는 정수로 변환한다.
"""


def signedIntToHex(value: int, nbit: int):
    conv = (value + (1 << nbit)) % (1 << nbit)
    convHex = hex(conv)[2:]
    return convHex


"""
try_parse_int, try_parse_float 
문자열을 정수 혹은 실수로 파싱해보고 실패한 경우 ReturnDefaultValue 를 반환.
try_parse_int 의 경우 X 란 문자열이 감지되면 16진수로 try 함.
"""


def try_parse_int(text2, baseReturn=0):
    text = str(text2)
    try:
        if str(text).upper().find("X") > 0:
            return int(f"{text}", 16)
        else:
            return int(text)
    except:
        return baseReturn


def try_parse_float(text2, baseReturn=0.0):
    text = str(text2)
    try:
        return float(text)
    except:
        return baseReturn


def splitSignedInt(value: int):
    """32비트 정수를 16비트 두개로 쪼갠다

    Args:
        value (int): 32비트 큰 부호있는 정수

    Returns:
        _type_: 16비트 정수 2개
    """
    signed32Hex = signedIntToHex(value, 32).rjust(8, "0")
    strUpperHex = signed32Hex[0:4]
    strBottomHex = signed32Hex[4:]
    return int(strUpperHex, 16), int(strBottomHex, 16)


def dicKeysMerge(dicTmp: Dict[str, str]):
    """MAP의 KEY 중에서 _1 과 _2 로 끝나는 값을 하나로 합쳐준다.
    즉 KEY 중에 LOC_1 과 LOC_2 가 있으면 두 키에 해당하는 값을 합친 후 LOC 란 키로 변환한다.
    모드버스 WORD가 16비트이기 때문에 32비트의 큰 숫자를 입출력하기 위해 변환필요.
    Args:
        dicTmp (Dict[str,str]): 일반적인 dic 변수

    Returns:
        _type_: _description_
    """
    # dicTmp = dicTmp2
    headerPart1 = "_1"
    headerPart2 = "_2"
    lsMergeKeyTmp = []
    for k, v in dicTmp.items():
        if k.endswith(headerPart1) or k.endswith(headerPart2):
            mergeKey = k[:-2]
            if mergeKey not in lsMergeKeyTmp:
                lsMergeKeyTmp.append(mergeKey)
    for merged in lsMergeKeyTmp:
        mergeH = f"{merged}{headerPart1}"
        mergeL = f"{merged}{headerPart2}"
        if mergeH in dicTmp.keys() and mergeL in dicTmp.keys():
            loc_cmd1 = hex(dicTmp[mergeH])
            loc_cmd2 = hex(dicTmp[mergeL])
            loc_cmdHex = f"{loc_cmd1[2:]}{loc_cmd2[2:]}"
            longHex = signed32(int(loc_cmdHex, 16))
            dicTmp[merged] = longHex
            del dicTmp[mergeH]
            del dicTmp[mergeL]
    return dicTmp


def getMulti(dicTmp: Dict[str, str], strKey):
    ret1 = getValueFromMap(dicTmp["H"], strKey)
    ret2 = getValueFromMap(dicTmp["V"], strKey)
    return ret1, ret2


def getMultiEx(dicTmp: Dict[str, str], strKey):
    """스파이더고에서 원하는 키값의 2개 모터 정보를 한꺼번에 리턴함.
    범블비에서는 6~7개의 모터 좌표를 모두 리턴하는 함수로 변경 필요.

    Args:
        dicTmp (Dict[str,str]): 현재 운행정보를 포함하는 dic 변수. (스파이더고에서는 dic_485 )
        strKey (_type_): 조회하고자 하는 값의 키.

    Returns:
        _type_: dic485, TARG 입력시 TARG_H, TARG_V 의 value 값을 리턴
    """
    ret1 = getValueFromMap(dicTmp, f"{strKey}_H")
    ret2 = getValueFromMap(dicTmp, f"{strKey}_V")
    return ret1, ret2


def str2Dic(estr, sep=",", lineterm="\n"):
    """_summary_

    Args:
        dicMergedStr (_type_): _description_
        sep (str, optional): _description_. Defaults to ','.
        lineterm (str, optional): _description_. Defaults to '\n'.

    Returns:
        _type_: _description_1`
    """
    dicReturn = {}
    for x in estr.split(lineterm):
        key = x.split(sep)[0]
        val = x.split(sep)[1]
        dicReturn[key] = val
    return dicReturn


def str2frame(estr, sep=",", lineterm="\n", set_header=True):
    df = None
    if set_header:
        df = pd.DataFrame(
            [x.split(sep) for x in estr.split(lineterm)[1:]],
            columns=[x for x in estr.split(lineterm)[0].split(sep)],
        )
    else:
        df = pd.DataFrame([x.split(sep) for x in estr.split(lineterm)])
    return df
    # dat = [x.split(sep) for x in estr.split(lineterm)][1:-1]
    # cdf = pd.DataFrame(dat)
    # if set_header:
    #     cdf = cdf.T.set_index(0, drop = True).T # flip, set ix, flip back
    # return cdf

def flipBoolan(anything):
  return False if isTrue(anything) else True

def intToBit(value, dicTmp: Dict[str, str]):
    strBit = bin(int(value))
    strBMSDataPartBit = ConstBitStream(strBit)
    #print(strBMSDataPartBit)


dicTmp = {
    1: "일",
    2: "이",
    3: "삼",
    4: "사",
    5: "오",
    6: "육",
    7: "칠",
    8: "팔",
    9: "구",
    10: "십,",
    98: "주방,",
    99: "충전소,",
}
# dicTmp = { 1:'일',2:'이이',3:'삼',4:'사아',5:'오오',6:'유욱',7:'치일',8:'파알',9:'구우',10:'시입'}


def GetKoreanFromNumber(strNum):
    if if_Number(strNum):
        sTmp = (int)(strNum)
        return dicTmp.get(sTmp, strNum)
    else:
        return strNum


def getTimestamp():
    current_time = rospy.get_rostime()

    # 년-월-일 시:분:초 포맷 예시
    # timestamp_str = datetime.fromtimestamp(current_time.to_sec()).strftime('%Y-%m-%d %H:%M:%S')

    # ISO 8601 확장형 포맷 예시
    # timestamp_str = datetime.fromtimestamp(current_time.to_sec()).isoformat(timespec='milliseconds')

    # 년월일_시분초 포맷 예시
    timestamp_str = datetime.fromtimestamp(current_time.to_sec()).strftime(
        "%Y%m%d_%H%M%S"
    )
    # timestamp_str = datetime.fromtimestamp(current_time.to_sec()).strftime('%Y-%m-%d %H:%M:%S')
    return timestamp_str


def setTimestamp():
    pass
    # sResult['time'] = getTimestamp()

def remove_dict_by_value(dict_list, key, value):
    # 결과를 저장할 빈 리스트 생성
    filtered_list = []

    # 리스트의 각 원소를 순회
    for item in dict_list:
        # 원소가 딕셔너리인지 확인
        if isinstance(item, dict):
            # 지정된 키의 값이 주어진 값과 다르면 리스트에 추가
            if item.get(key) != value:
                filtered_list.append(item)
        else:
            # 원소가 딕셔너리가 아니면 그대로 리스트에 추가
            filtered_list.append(item)

    return filtered_list

def removeDictFromList(key, val, orderList: dict):
    """주어진 key value 를 가진 원소를 orderList 에서 제거한다

    Args:
        key (_type_): _description_
        val (_type_): _description_
        orderList (dict): _description_

    Returns:
        _type_: _description_
    """
    for i in range(len(orderList) - 1, -1, -1):
        if orderList[i][key] == val:
            del orderList[i]
    return orderList


def get_valueDict_from_keys(source_dict, keys):
    """my_dict = {"A": 10, "B": 20, "C": 20}
        로 선언되어 있을때 내가 지정한 배열을 파라미터로 넘기면 각 배열 원소를
        KEY 값으로 갖는 VALUE 를 dict 형태로 리턴하는 함수를 만들어줘. 예를 들면
        파라미터로 my_dict, ['A','B'] 를 넘기면 return 값은 {"A": 10, "B": 20} 이
        되는거야. 단 내가 넘긴 파라미터의 배열에서 my_dict 에 없는 key 값이 있다면 무시해줘

    Args:
        source_dict (_dict_): _description_
        keys (_list_): _description_

    Returns:
        _type_: 주어진 키들에 대해 딕셔너리를 필터링하여 새 딕셔너리 생성
    """
    #

    return {key: source_dict[key] for key in keys if key in source_dict}


def get_valuelist_from_keys(input_dict, keys_list):
    """주어진 키 리스트에 해당하는 값들을 찾아서 리스트로 반환
    my_dict = {"A": 10, "B": 20, "C": 20}
    로 선언되어 있을때 내가 지정한 배열을 파라미터로 넘기면 각 배열 원소를
    KEY 값으로 갖는 VALUE 를 리턴하는 함수를 만들어줘. 예를 들면 파라미터로
    my_dict, ['A','B'] 를 넘기면 return 값은 [10,20] 이 되는거야

    Args:
        input_dict (_type_): _description_
        keys_list (_type_): _description_

    Returns:
        _type_: _description_
    """
    return [input_dict[key] for key in keys_list if key in input_dict]


def getPercentBatteryRemain(
    current_voltage, full_charge_voltage=58.8, cut_off_voltage=43.0
):
    """_summary_
        만충전압 58.8V 이고 전압컷이 43.0V 인 배터리가 있을때 현재 전압을 입력하면
        백분율과 남은전류를 와트값으로 리턴하는 파이썬 코드는?

    Args:
        current_voltage (_float_): _현재 전압_
        full_charge_voltage (float, optional): 만충전압. Defaults to 58.8.
        cut_off_voltage (float, optional): 전압컷. Defaults to 43.0.

    Returns:
        _float_: (배터리 잔량 백분율)
    """
    # 백분율 계산
    percentage = (
        (current_voltage - cut_off_voltage) / (full_charge_voltage - cut_off_voltage)
    ) * 100

    # 백분율이 0보다 작으면 0으로, 100보다 크면 100으로 조정
    percentage = max(0, min(percentage, 100))

    # 남은 전류를 와트값으로 변환하는 계산은 배터리의 용량(암페어시)과 장치의 소비 전력(와트) 정보가 필요함.
    # 이 예제에서는 이러한 정보 없이 백분율만 반환합니다.

    return percentage


# 두 변의 길이와 끼인 각도를 받아 나머지 한 변의 길이를 계산
# a = 5
# b = 7
# gamma_degrees = 60
# third_side = calculate_third_side(a, b, gamma_degrees)
# print(f"나머지 한 변의 길이: {third_side}")
def calculate_third_side(a, b, gamma_degrees):
    # 각도를 라디안으로 변환
    gamma_radians = math.radians(gamma_degrees)
    
    # 코사인 법칙을 사용하여 세 번째 변의 길이를 계산
    c = math.sqrt(a**2 + b**2 - 2 * a * b * math.cos(gamma_radians))
    
    return c

# # 사용 예시
# a = 5
# b = 7
# c = 8
# angles = calculate_angles(a, b, c)
# print(f"내부 각도: α = {angles[0]:.2f}°, β = {angles[1]:.2f}°, γ = {angles[2]:.2f}°")
# 삼각형의 세 변의 길이를 입력받았을때 각도를 계산.
def calculate_angles(a, b, c):
    alpha_degrees= beta_degrees= gamma_degrees = -1
    try:
        # 코사인 법칙을 사용하여 각도를 계산
        # alpha는 변 b와 변 c 사이의 각도
        alpha = math.acos((b**2 + c**2 - a**2) / (2 * b * c))
        # beta는 변 a와 변 c 사이의 각도
        beta = math.acos((a**2 + c**2 - b**2) / (2 * a * c))
        # gamma는 변 a와 변 b 사이의 각도
        gamma = math.acos((a**2 + b**2 - c**2) / (2 * a * b))
        
        # 각도를 라디안에서 도 단위로 변환
        alpha_degrees = math.degrees(alpha)
        beta_degrees = math.degrees(beta)
        gamma_degrees = math.degrees(gamma)
    except Exception as e:
        message = traceback.format_exc()
        rospy.loginfo(message)    
        rospy.loginfo(f'Param a, b, c : {a},{b},{c}')    
    return alpha_degrees, beta_degrees, gamma_degrees

lastperiod_secs = getDateTime()

def loginfo_throttle_identical(period_secs, logStr):
    global lastperiod_secs
    td = getDateTime() - lastperiod_secs
    if td.total_seconds() >= period_secs:
        lastperiod_secs = getDateTime()
        print(logStr)
  
def count_elements(element_list, findStr):
  count = sum(1 for element in element_list if element.startswith(findStr))
  return count

def calculate_triangle_sides(a, angle1, angle2):
    """
    주어진 변 a와 그 양쪽의 두 각도를 이용하여 삼각형의 나머지 두 변을 계산하는 함수.

    Parameters:
        a (float): 주어진 변의 길이
        angle1 (float): 첫 번째 각도 (도 단위)
        angle2 (float): 두 번째 각도 (도 단위)

    Returns:
        tuple: 나머지 두 변의 길이 (b, c)
    """
    # 세 번째 각도 계산
    angle3 = 180 - (angle1 + angle2)
    
    # 각도를 라디안으로 변환
    angle1_rad = math.radians(angle1)
    angle2_rad = math.radians(angle2)
    angle3_rad = math.radians(angle3)
    
    # 사인 법칙을 이용해 나머지 두 변 계산
    b = a * (math.sin(angle2_rad) / math.sin(angle3_rad))
    c = a * (math.sin(angle1_rad) / math.sin(angle3_rad))
    
    return b, c

def calculate_triangle(X1, Y1, X2, Y2, X3, Y3):
    # 세 변의 길이 계산
    side_a = math.sqrt((X2 - X3)**2 + (Y2 - Y3)**2)
    side_b = math.sqrt((X1 - X3)**2 + (Y1 - Y3)**2)
    side_c = math.sqrt((X1 - X2)**2 + (Y1 - Y2)**2)
    
    # 세 각의 크기 계산 (라디안)
    angle_A = math.acos((side_b**2 + side_c**2 - side_a**2) / (2 * side_b * side_c))
    angle_B = math.acos((side_a**2 + side_c**2 - side_b**2) / (2 * side_a * side_c))
    angle_C = math.acos((side_a**2 + side_b**2 - side_c**2) / (2 * side_a * side_b))
    
    # 라디안을 도로 변환
    angle_A_deg = math.degrees(angle_A)
    angle_B_deg = math.degrees(angle_B)
    angle_C_deg = math.degrees(angle_C)
    
    return round(side_a), round(side_b),round( side_c),round(angle_A_deg),round( angle_B_deg),round( angle_C_deg)

def rotate_point_cloud(points, axis, angle):
    if angle == 0:
        return points
    """
    Rotate the point cloud around the specified axis by the given angle.
    
    :param points: List of points to rotate
    :param axis: Axis to rotate around ('x', 'y', or 'z')
    :param angle: Angle in degrees to rotate
    :return: List of rotated points
    """
    # Convert angle from degrees to radians
    theta = np.radians(angle)
    
    if axis == 'x':
        rotation_matrix = np.array([[1, 0, 0],
                                    [0, np.cos(theta), -np.sin(theta)],
                                    [0, np.sin(theta), np.cos(theta)]])
    elif axis == 'y':
        rotation_matrix = np.array([[np.cos(theta), 0, np.sin(theta)],
                                    [0, 1, 0],
                                    [-np.sin(theta), 0, np.cos(theta)]])
    elif axis == 'z':
        rotation_matrix = np.array([[np.cos(theta), -np.sin(theta), 0],
                                    [np.sin(theta), np.cos(theta), 0],
                                    [0, 0, 1]])
    else:
        raise ValueError("Axis must be 'x', 'y', or 'z'")
    
    # Rotate each point in the point cloud
    rotated_points = []
    for point in points:
        rotated_point = np.dot(rotation_matrix, np.array([point[0], point[1], point[2]]))
        rotated_points.append((rotated_point[0], rotated_point[1], rotated_point[2]))
    
    return rotated_points

def get_closest_value(data_dict, target_key):
    # dict가 비어 있는지 확인
    if not data_dict:
        return None
    
    # 가장 가까운 key를 찾기 위해 초기화
    closest_key = None
    closest_diff = float('inf')

    # 모든 key 값을 순회하며 가장 가까운 key를 찾기
    for key in data_dict:
        diff = abs(key - target_key)
        if diff < closest_diff:
            closest_diff = diff
            closest_key = key

    # 가장 가까운 key에 해당하는 value를 반환
    return data_dict[closest_key]

def find_closest_value(arr, target):
    return min(arr, key=lambda x: (abs(x - target), x))

def safe_read_json(json_str):
  try:
    #   # json_str 이 None 이거나 유효하지 않으면 빈 DataFrame 반환
    #   if not json_str:
    #       return pd.DataFrame()
      
    #   # 유효한 JSON 문자열이면 DataFrame으로 변환
      return pd.read_json(json_str, orient='records')
  
  except Exception as e:
      # JSON 변환이 실패하면 빈 DataFrame 반환
      print(e)
      return pd.DataFrame()
    
def is_valid_python_dict_string(myjson):
    try:
        # 안전하게 문자열을 Python dict로 변환 시도
        result = ast.literal_eval(myjson)
        # 변환 결과가 dict인지 확인
        return isinstance(result, dict)
    except (SyntaxError, ValueError):
        return False  # 변환 실패 시 False 반환

# # 예제 JSON 문자열 (단일 인용부호 사용)
# myjson = "{'timestamp': '2025-02-12T14:37:04.950', 'AndroidIP': '172.30.1.5', 'TAGSCAN': False, 'INTERVAL_DATA': 1000, 'x': 0, 'y': 0.0003054324, 'z': 0, 'ax': 0.30089745, 'ay': -0.1788635, 'az': 9.886545, 'mx': -16.325, 'my': -34.9375, 'mz': -45.225002, 'Angle_X': -1, 'Angle_Y': -2, 'Angle_Z': 113, 'proximity': 0, 'lightlux': 85, 'batteryPercentage': 98, 'isCharging': False, 'batteryHealth': 'Good', 'batteryTechnology': 'Li-poly', 'Fx': '190.65', 'Fy': '190.65', 'IMG_WIDTH': '640', 'IMG_HEIGHT': '480', 'ACTIVATED_CAM': '2', 'CPU_USAGE': 2, 'MEMORY_USED': 3104.88, 'MEMORY_TOTAL': 5606.72, 'MEMORY_USAGE': 55, 'NETWORK_TX': 8702.99, 'NETWORK_RX': 3836.47, 'TEMPERATURE_DEGREE': 37}"

# # 유효성 검사 실행
# if is_valid_python_dict_string(myjson):
#     print("✅ myjson은 유효한 Python dict 문자열입니다. ast.literal_eval()을 호출할 수 있습니다.")
# else:
#     print("❌ myjson은 유효하지 않은 문자열입니다. ast.literal_eval()을 호출하면 오류가 발생할 수 있습니다.")

def nameof(var):
    return [k for k, v in globals().items() if v is var][0]

def get_cpu_temperature_wsl():
    """
    Uses PowerShell to retrieve the CPU temperature from the Windows host in WSL.
    Returns:
        float: CPU temperature in Celsius, or None if not available.
    """
    try:
        # PowerShell 명령어 실행
        result = subprocess.check_output([
            "powershell.exe",
            "-Command",
            "Get-WmiObject MSAcpi_ThermalZoneTemperature -Namespace root/wmi | Select -Expand CurrentTemperature"
        ], stderr=subprocess.DEVNULL)

        # 결과 처리
        lines = result.decode("utf-8").strip().splitlines()
        if not lines:
            return None

        # ACPI 온도는 Kelvin * 10으로 출력됨 → 섭씨로 변환
        temps_celsius = [(int(t) / 10.0) - 273.15 for t in lines if t.isdigit()]
        return round(sum(temps_celsius) / len(temps_celsius), 2) if temps_celsius else None

    except Exception as e:
        print(f"WSL CPU 온도 확인 실패: {e}")
        return None

def get_cpu_temperature_ubuntu():
    """
    Reads the CPU temperature from the thermal zone file in Ubuntu.
    Returns:
        float: The CPU temperature in degrees Celsius.
    """
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as file:
            temp_milli = int(file.read().strip())
            return temp_milli / 1000.0  # Convert to Celsius
    # except FileNotFoundError:
    #     raise RuntimeError("Could not find the thermal zone file. Ensure this is a Linux system with proper drivers.")
    # except ValueError:
    #     raise RuntimeError("Unexpected data format in the thermal zone file.")
    except Exception as e:
        #raise RuntimeError(f"An error occurred while reading the CPU temperature: {e}")
        return get_cpu_temperature_wsl()

def save_dict_to_csv(file_path: str, data: dict, limit : int):
    """
    파일 경로와 데이터를 받아 CSV 파일로 저장한다. (탭 구분자)
    최대 100개의 레코드만 유지하도록 조정한다.
    """
    df = pd.DataFrame(list(data.items()), columns=["timestamp", "value"])
    existing_df = pd.DataFrame()
    if os.path.exists(file_path):
        try:
            existing_df = pd.read_csv(file_path, delimiter='\t')
            df = pd.concat([existing_df, df])
        except Exception as e:
            print(f"Error reading existing CSV file: {existing_df}")
    
    df = df.sort_values(by='timestamp')  # 오래된 순으로 정렬
    df = df.iloc[-limit:]  # 100개 초과 시 오래된 항목 삭제
    
    df.to_csv(file_path, index=False, sep='\t')

def load_csv_to_dict(file_path: str, sort_ascending: bool = True) -> dict:
    """
    CSV 파일을 불러와 정렬 후 dict 형태로 반환한다.
    """
    if not os.path.exists(file_path):
        return {}
    
    try:            
        df = pd.read_csv(file_path, delimiter='\t')
        df = df.sort_values(by='timestamp', ascending=sort_ascending)
        
        df['timestamp'] = df['timestamp'].astype(str)  # timestamp를 문자열로 변환
        return dict(zip(df['timestamp'], df['value']))
    except Exception as e:
        print(f"Error loading CSV file: {e}")
        return {}

def get_script_directory() -> str:
    """
    현재 실행 중인 파이썬 파일의 디렉토리 경로를 반환한다.
    """
    return str(Path(__file__).resolve().parent)

def get_timestampNow():
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")

def calculate_average_keys(data):
    value_to_keys = defaultdict(list)
    
    # Value별로 Key를 그룹화
    for key, value in data.items():
        value_to_keys[value].append(key)
    
    # 각 Value별 Key 값 평균 계산
    result = {value: round(sum(keys) / len(keys)) for value, keys in value_to_keys.items()}
    return result

def calculate_relative_height_difference(crop_data, lidar_tilt_deg=0):
    """
    기울어진 LiDAR가 장애물을 감지했을 때,
    LiDAR 원점과 장애물 꼭대기 간의 상대 높이 차이를 계산하는 함수.

    Parameters:
        crop_data (sensor_msgs.PointCloud2): 크롭된 포인트 클라우드 데이터
        lidar_tilt_deg (float): LiDAR의 기울어진 각도 (기본값: 60°)

    Returns:
        float: LiDAR 원점과 장애물 꼭대기 간의 상대 높이 차이 (m)
    """
    if crop_data is None:
      return None
    
    # LiDAR의 기울어진 각도 (라디안 변환)
    tilt_rad = np.radians(lidar_tilt_deg)

    # PointCloud2 데이터를 numpy 배열로 변환 (x, y, z)
    points = np.array(list(pc2.read_points(crop_data, field_names=("x", "y", "z"), skip_nans=True)))

    if points.size < 10:
        return None  # 장애물 없음

    # 장애물 중 가장 큰 x 값 찾기 (가장 먼 장애물)
    max_x = np.max(points[:, 0])
    min_x = np.min(points[:, 0])

    # LiDAR 원점과 장애물 꼭대기 간의 상대 높이 차이 (x 좌표 보정)
    height_difference = min_x * np.cos(tilt_rad)

    return height_difference

def transform_marker_coordinates(x, y, z, pitch):
    """
    pitch(라디안) 만큼 기울어진 카메라에서 마커의 상대 좌표(x, y, z)를 변환.

    :param x: 마커의 원래 x 좌표
    :param y: 마커의 원래 y 좌표
    :param z: 마커의 원래 z 좌표
    :param pitch: 카메라의 pitch 각도 (라디안)
    :return: 변환된 (x', y', z') 좌표
    """
    # x축을 기준으로 회전 변환 행렬 생성
    rotation_matrix = np.array([
        [1, 0, 0],  
        [0, np.cos(pitch), -np.sin(pitch)],  
        [0, np.sin(pitch), np.cos(pitch)]
    ])

    # 원래 좌표 벡터
    original_coords = np.array([x, y, z])

    # 변환된 좌표 계산
    transformed_coords = np.dot(rotation_matrix, original_coords)

    return tuple(transformed_coords)

def immutable_multi_dict_to_dict(immutable_multi_dict, suffix = ''):
    return {key+suffix: immutable_multi_dict.getlist(key)[-1] for key in immutable_multi_dict}

def compute_final_position_and_angle(movements):
    total_x = 0.0
    total_y = 0.0

    for angle_deg, distance in movements:
        # Y축 기준 0도 → 수학적 X축 기준으로 변환
        angle_rad = math.radians((angle_deg - 90) % 360)
        dx = math.cos(angle_rad) * distance
        dy = math.sin(angle_rad) * distance
        total_x += dx
        total_y += dy

    hypotenuse = math.sqrt(total_x**2 + total_y**2)
    angle_deg = (math.degrees(math.atan2(total_y, total_x)) + 270) % 360

    # return {
    #     "X": round(total_x, 4),
    #     "Y": round(total_y, 4),
    #     "DISTANCE": round(hypotenuse, 4),
    #     "ANGLE": round(angle_deg, 2)
    # }
    return round(hypotenuse*1000),round(angle_deg)
    # """
    # movements: 리스트 형태로 [ (각도_deg, 거리_m), ... ]
    # 각도는 Y축 기준 시계방향 0도 기준
    # """
    # #movements = [(-current_dict['X'],-current_dict['Y']),(refloc_dict['X'], refloc_dict['Y'])]
    # total_x = 0.0
    # total_y = 0.0
    # print(movements)

    # for distance,angle_deg in movements:
    #     # Y축 기준 각도를 수학적 X축 기준으로 변환
    #     # 수학 기준 0도 = 오른쪽 → 우리는 0도 = 위쪽(Y축)
    #     angle_rad = math.radians((angle_deg + 360) % 360)

    #     dx = math.cos(angle_rad) * distance
    #     dy = math.sin(angle_rad) * distance

    #     total_x += dx
    #     total_y += dy

     # 최종 거리와 각도(Y축 기준)
    # hypotenuse = math.sqrt(total_x**2 + total_y**2)
    # angle_deg = (math.degrees(math.atan2(total_y, total_x)) + 360) % 360
    # return round(hypotenuse*1000),round(angle_deg)


def compute_hypotenuse_and_angle(x, y):
    hypotenuse = math.sqrt(x**2 + y**2)
    angle_deg = (math.degrees(math.atan2(y, x)) + 360) % 360
    return (hypotenuse),(angle_deg)

def compute_position(target_marker,current_marker):
    dx1 = target_marker['X']
    dy1 = target_marker['Y']
    dx = -current_marker['X']
    dy = -current_marker['Y']    
    #print(dx, dy, dx1, dy1)
    print(current_marker, target_marker)
    distanceI,angleI = compute_hypotenuse_and_angle(dx, dy)
    distanceII,angleII = compute_hypotenuse_and_angle(dx1, dy1)
    print(compute_hypotenuse_and_angle(dx, dy))
    print(compute_hypotenuse_and_angle(dx1, dy1))
    #x1,y1= calculate_coordinates(distanceI,angleI)
    movements = [(angleI,distanceI),(angleII,distanceII)]
    print(movements)
    distance, angleD = compute_final_position_and_angle(movements)
    print(distance, angleD)
    return distance, (angleD+270)%360
    
def ConvertArucoSizeToReal(arucoDistance):    
    pulse_value = (arucoDistance / 58) * 32
    return round(pulse_value)

def ConvertRealToArucoSize(realDistance):    
    aruco_value = (realDistance / 32) * 58
    return (aruco_value)
CAMERA_DISTANCE_FROM_CENTER = ConvertRealToArucoSize(0.32)
print(os.path.splitext(os.path.basename(__file__))[0],getDateTime())
