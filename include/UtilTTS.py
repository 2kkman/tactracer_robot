from enum import Enum

class TTSMessage(Enum):
    ARUCO_FOUND_OK = '테이블 마커를 찾았습니다'
    ARUCO_FOUND_FAIL = '테이블 마커를 찾지 못했습니다.'
    ARUCO_CORRECTED = '테이블 위치 인식을 완료했습니다.'
    ARUCO_FINE_TUNING = '서빙 위치를 세부적으로 조절합니다.'
    ARUCO_FOLD_TUNING = '서빙 위치가 변경되었습니다.'
    ARUCO_NOT_CORRECTED = '테이블 감지 시도 횟수를 초과했습니다.'
    ALARM_DETECTED = '알람이 감지되어 긴급정지 합니다'
    REMAIN_30S = '주문하신 음식이 나왔습니다.'
    PAY_30S = '현금을 담아주세요.'
    RETURN_30S = '빈그릇을 담아주세요'
    ALARM_BATTERY = '배터리를 충전해주세요.'
    OBSTACLE_SLOW = '장애물이 감지되어 서행합니다'
    OBSTACLE_STOP = '장애물이 감지되어 정지합니다'
    SHAKE_TRAY_DETECTED = '트레이 흔들림이 감지되었습니다'
    SHAKE_TRAY_OK = '트레이 흔들림이 사라졌습니다'
