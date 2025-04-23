#!/usr/bin/env python3
"""Text to speech service using gTTS"""
import os
import rospy
from gtts import gTTS
from playsound import playsound
from tta_blb.srv import TTS, TTSResponse

# standard imports first, then third-party libraries, then local imports



audio_path = os.path.dirname(os.path.realpath(__file__)) + "/../audio/"

class TssService:
    """Class for text to speech service using gTTS"""
    def __init__(self):
        self.is_speaking = False

    def handle_tts(self, req):
        """_summary_

        Args:
            req (dictionary): service call request
                text (string): text to be converted to speech
                queuing (bool): whether to queue the request or not

        Returns:
            TTSResponse(string): message
        """
        if self.is_speaking and not req.queuing:
            return TTSResponse("Error: Already speaking")
        
        self.is_speaking = True
        if not os.path.isfile(f"{audio_path}{req.text}.mp3"):
            tts = gTTS(
                text=req.text,
                lang='ko',
                slow=True)
            tts.save(f"{audio_path}{req.text}.mp3")
        playsound(f"{audio_path}{req.text}.mp3")
        self.is_speaking = False
        
        return TTSResponse("Success")

    def tts_service(self):
        """Text to speech service using gTTS"""
        rospy.init_node('tts_service')
        rospy.Service('tts', TTS, self.handle_tts)
        rospy.loginfo("Ready to convert text to speech.")
        rospy.spin()

if __name__ == "__main__":
    tts_service = TssService()
    tts_service.tts_service()
