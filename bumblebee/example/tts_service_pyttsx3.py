#!/usr/bin/env python
import rospy
from tta_blb.srv import *
import pyttsx3

class TTSService:
    def __init__(self):
        self.engine = pyttsx3.init()
                
        self.voices = self.engine.getProperty('voices')
        self.rate = self.engine.getProperty('rate')
        self.engine.startLoop(False)
        self.is_speaking = False

    def handle_tts(self, req):
        if self.is_speaking and not req.queuing:
            return TTSResponse("Error: Already speaking")


        self.is_speaking = True
        
        self.engine.setProperty('rate', self.rate-30)
        self.engine.say(req.text)
        
        self.engine.iterate()
        while self.engine.isBusy():
            pass
        self.is_speaking = False
        
        return ("Success")

    def tts_service(self):
        rospy.init_node('tts_service')
        s = rospy.Service('tts', TTS, self.handle_tts)
        # for voice in self.voices:
        #     print(f"Voice[]:{voice.name, voice.id}")
        print("Ready to convert text to speech.")
        rospy.spin()

if __name__ == "__main__":
    tts_service = TTSService()
    tts_service.tts_service()
