#!/usr/bin/env python

import rospy
import rosnode
from std_srvs.srv import Trigger, TriggerResponse

import os
import subprocess


class DataRecorder():
    def __init__(self):
        self.start_recording_service = rospy.Service('/spg_recording/start', Trigger, self.start_recording)
        self.stop_recording_service = rospy.Service('/spg_recording/stop', Trigger, self.stop_recording)
        self.stop_recording_service = rospy.Service('/spg_recording/toggle', Trigger, self.toggle_recording)

        self.process = None
        self.recording = False

        self.output_directory = rospy.get_param('/tta_blb/output_directory', '~/rosbags/')
        
        self.topics = rospy.get_param('/tta_blb/topics', [])
        if not self.topics:
            rospy.logerr('No Topics Specified.')

        self.command = ['rosrun', 'rosbag', 'record', '-e'] + self.topics + ['__name:=data_recording_tta_blb']
        print(self.command)
        rospy.loginfo('SPG Recorder Started')

    def toggle_recording(self, req):
        if self.recording:
            return self.stop_recording(req)
        else:
            return self.start_recording(req)

    def start_recording(self, req):
        if self.recording:
            rospy.logerr('Already Recording')
            return TriggerResponse(False, 'Already Recording')

        self.process = subprocess.Popen(self.command, cwd=self.output_directory)
        self.recording = True
        rospy.loginfo('Started recorder, PID %s' % self.process.pid)
        return TriggerResponse(True, 'Started recorder, PID %s' % self.process.pid)

    def stop_recording(self, req):
        if not self.recording:
            rospy.logerr('Not Recording')
            return TriggerResponse(False, 'Not Recording')

        rosnode.kill_nodes(['/data_recording_tta_blb'])

        self.process = None
        self.recording = False

        rospy.loginfo('Stopped Recording')
        return TriggerResponse(True, 'Stopped Recording')


if __name__ == "__main__":
    rospy.init_node('spg_recording')
    DataRecorder()
    rospy.spin()
