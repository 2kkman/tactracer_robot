#!/usr/bin/env python3
from UtilBLB import *

import rospy
import cv2
import UtilBLB
from datetime import datetime
from std_srvs.srv import *

class VideoRecorder:
    def __init__(self):
        self.recording = False
        self.out = None
        self.cap = None
        rospy.init_node(f'node_{ServiceBLB.REC_CAM_QBI.name}')
        self.service = rospy.Service(ServiceBLB.REC_CAM_QBI.value, SetBool, self.handle_record_video)
        rospy.loginfo(f"{ServiceBLB.REC_CAM_QBI.value} Service Ready")

    def handle_record_video(self, req):
        if req.data:
            if self.recording:
                rospy.logwarn("Already recording!")
                return False
            else:
                self.start_recording()
                return True
        else:
            if self.recording:
                self.stop_recording()
                return True
            else:
                rospy.logwarn("Not currently recording!")
                return False

    def start_recording(self):
        self.filename = self.generate_filename()
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            rospy.logerr("Cannot open webcam")
            return False
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out = cv2.VideoWriter(self.filename, fourcc, 20.0, (640, 480))
        self.recording = True
        rospy.loginfo(f"Recording started, saving to {self.filename}")
        rospy.Timer(rospy.Duration(0.05), self.record)
    
    def generate_filename(self):
        now = datetime.now()
        return PATH_RECORDING+now.strftime("%Y%m%d%H%M%S") + ".mp4"

    def record(self, event):
        if self.recording and self.cap.isOpened():
            ret, frame = self.cap.read()
            frame = cv2.rotate(frame, cv2.ROTATE_180)
            if ret:
                self.out.write(frame)
            else:
                rospy.logwarn("Failed to capture frame")
    
    def stop_recording(self):
        self.recording = False
        if self.cap:
            self.cap.release()
        if self.out:
            self.out.release()
        rospy.loginfo(f"Recording stopped, file saved: {self.filename}")

if __name__ == '__main__':
    try:
        VideoRecorder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
