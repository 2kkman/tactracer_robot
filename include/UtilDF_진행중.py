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

def get_logger(func_name):
    """함수 이름을 기반으로 개별 로그 파일을 설정하고, pretty 로그 메서드 포함"""
    log_dir = "/root/Downloads/"
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

def getDateTime():
  return datetime.now()

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


class UbuntuEnv(Enum):
    ROS_MASTER_URI = auto()
    CONFIG = auto()
    SCR_DIR = auto()
    QBI = auto()
    ITX = auto()
    COMMON = auto()
    ROS_HOSTNAME = auto()


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


def GetUbutuParam(variable_name):
    if variable_name in os.environ:
        my_variable = os.environ[variable_name]
        return my_variable
    else:
        return None

def GetMasterIP():
  return GetUbutuParam(UbuntuEnv.ROS_HOSTNAME.name)

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


def getDic_FromFile(filename, spliter):
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
    print(strBMSDataPartBit)


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
    
    if os.path.exists(file_path):
        existing_df = pd.read_csv(file_path, delimiter='\t')
        df = pd.concat([existing_df, df])
    
    df = df.sort_values(by='timestamp')  # 오래된 순으로 정렬
    df = df.iloc[-limit:]  # 100개 초과 시 오래된 항목 삭제
    
    df.to_csv(file_path, index=False, sep='\t')

def load_csv_to_dict(file_path: str, sort_ascending: bool = True) -> dict:
    """
    CSV 파일을 불러와 정렬 후 dict 형태로 반환한다.
    """
    if not os.path.exists(file_path):
        return {}
    
    df = pd.read_csv(file_path, delimiter='\t')
    df = df.sort_values(by='timestamp', ascending=sort_ascending)
    
    df['timestamp'] = df['timestamp'].astype(str)  # timestamp를 문자열로 변환
    return dict(zip(df['timestamp'], df['value']))

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