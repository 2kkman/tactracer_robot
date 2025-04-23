#!/usr/bin/env python3
"""Text to speech service using gTTS"""
import os
import rospy
from gtts import gTTS
from playsound import playsound
from tta_blb.srv import TTS, TTSResponse
from threading import Thread
from UtilBLB import *

# standard imports first, then third-party libraries, then local imports
audio_path = os.path.dirname(os.path.realpath(__file__)) + "/../audio/"

class TssService:
    """Class for text to speech service using gTTS"""
    def __init__(self):
        self.is_speaking = False

    def handle_tts(self, req):
        """Handles the TTS service request in a non-blocking way using threads.

        Args:
            req (dictionary): service call request
                text (string): text to be converted to speech
                queuing (bool): whether to queue the request or not

        Returns:
            TTSResponse(string): message
        """
        if self.is_speaking and not req.queuing:
            return TTSResponse("Error: Already speaking")
        
        # Start the TTS in a separate thread to avoid blocking
        thread = Thread(target=self.speak, args=(req.text,))
        thread.start()
        
        return TTSResponse("Success")

    def speak(self, text):
        """Handles the actual speaking process in a separate thread."""
        self.is_speaking = True
        audio_file = f"{audio_path}{text}.mp3"
        
        # Check if the file already exists, if not, create it
        if not os.path.isfile(audio_file):
            tts = gTTS(text=text, lang='ko', slow=True)
            tts.save(audio_file)
        
        # Play the sound
        playsound(audio_file)
        self.is_speaking = False

    def tts_service(self):
        """Text to speech service using gTTS"""
        rospy.init_node('tts_service')
        machineName = GetMachineStr()
        serviceTTS = ServiceBLB.TTS_ITX.name
        if machineName == UbuntuEnv.QBI.name:
            serviceTTS = ServiceBLB.TTS_QBI.name
        rospy.Service(serviceTTS, TTS, self.handle_tts)
        rospy.loginfo("Ready to convert text to speech.")
        rospy.spin()

if __name__ == "__main__":
    tts_service = TssService()
    tts_service.tts_service()
