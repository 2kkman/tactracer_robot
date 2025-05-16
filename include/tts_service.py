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
