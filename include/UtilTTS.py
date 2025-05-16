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
    REQUEST_TABLECLEAR = '서빙 위치에 장애물을 치워주세요'
# services/tts_service.py

import os
from gtts import gTTS
from playsound import playsound
from threading import Thread

audio_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../audio"))

class TtsService:
    def __init__(self):
        self.is_speaking = False

    def handle_tts(self, text: str, queuing: bool = False) -> str:
        if self.is_speaking and not queuing:
            return "Error: Already speaking"

        thread = Thread(target=self.speak, args=(text,))
        thread.start()
        return "Success"

    def speak(self, text: str):
        self.is_speaking = True
        os.makedirs(audio_path, exist_ok=True)
        audio_file = os.path.join(audio_path, f"{text}.mp3")

        if not os.path.isfile(audio_file):
            tts = gTTS(text=text, lang='ko', slow=True)
            tts.save(audio_file)

        playsound(audio_file)
        self.is_speaking = False

tts_service = TtsService()