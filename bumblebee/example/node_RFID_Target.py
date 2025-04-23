#!/usr/bin/env python3
import rospy

from std_msgs.msg import String
from UtilBLB import *
from MotorControl import MOTOR_CONTROL_CMD, MOTOR_DRIVER


class RFIDTarget:
    def __init__(self, currentTag, targetTag, tagOnPath, mbid: ModbusID = ModbusID.MOTOR_H.value):
        self._current = currentTag
        self._target = targetTag
        self._tags = tagOnPath
        self._mbid = mbid
        self._pos = None
        self._target_index = tagOnPath.index(targetTag)
         
         
    def matchTag(self, currentTag: str):
        try:
            motor_driver = MOTOR_DRIVER()
            current_index = self._tags.index(currentTag)
            if current_index == self._target_index:
                rospy.loginfo(f"Arrived at the targeting. Stopping.")
                motor_driver.EmergencyStop(self._mbid)
            elif current_index == self._target_index-1:
                rospy.logdebug(f"Targeted the tag just before the targeting")
                motor_driver.SetSpeedTarget(self._mbid)
            else:
                rospy.logdebug(f'just passing by the tag: {currentTag}')
                
        except ValueError as e:
            rospy.logerr(f"Error in matchRFID: {e}")


    def start(self):
        rospy.init_node("RFIDTarget", anonymous=True, log_level=rospy.DEBUG)
        rospy.loginfo("RFIDTarget node started.")
        self.subscriber_RFID_ = rospy.Subscriber(TopicName.RFID.name, String, self.callback_RFID)
        self.subscriber_BLB_CMD_ = rospy.Subscriber(TopicName.BLB_CMD.name, String, self.callback_BLBCMD)
        self.subscriber_STATE = rospy.Subscriber(f'{TopicName.MB_.name}{ModbusID.MOTOR_H.value}', String, self.callbackCmd_STATE)
        rospy.spin()


    def callback_RFID(self, data):
        recvDataMap = getDic_strArr(data.data, sDivFieldColon, sDivItemComma)
        self.matchTag(recvDataMap[RFID_RESULT.EPC.name])
        
        
    def callback_BLBCMD(self, data):
        recvDataMap = getDic_strArr(data.data, sDivFieldColon, sDivItemComma)
        rospy.logdebug(f"Received BLB_CMD: {recvDataMap}")
        

    def callbackCmd_STATE(self, data):
        recvDataMap = getDic_strArr(data.data, sDivFieldColon, sDivItemComma)
        
        cur_pos = recvDataMap.get(MonitoringField.CUR_POS.name)
        cmd_pos = recvDataMap.get(MonitoringField.CMD_POS.name)
        if cmd_pos is None:
            return
        stop_pos = str(round(int(cmd_pos)*0.8, -4))
        rospy.logdebug(f'cur_pos: {cur_pos}, cmd_pos: {cmd_pos}, stop_pos: {stop_pos}')
        # print(f"stop_pos: {stop_pos} {type(stop_pos)}")
        # print(f"{stop_pos}")
        
            # if cmd_pos is None:
            #     return
            # if cur_pos < cmd_pos:

if __name__ == "__main__":
    tags_on_path = ['start_tag', '000000000000201111180168','3005FB63AC1F3681EC880468', '303443D11C0001C000000001']
    current_tag = "start_tag"
    target_tag = "303443D11C0001C000000001"
    rfid_target = RFIDTarget(currentTag = current_tag, targetTag = target_tag, tagOnPath = tags_on_path)
    rfid_target.start()
