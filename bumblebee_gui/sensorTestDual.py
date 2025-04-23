
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import QDate
from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo
from PyQt5 import uic
import rospy
from std_msgs.msg import String, Header
from sensor_msgs.msg import Imu, Temperature, RelativeHumidity
# from ui.sensorTest_ui import Ui_MainWindow
from ui.sensorTest_dual_ui import Ui_MainWindow

import threading
from enum import Enum, auto
from tf.transformations import quaternion_from_euler


sDivFieldColon = ':'
sDivItemComma = ','
sDivEmart ='`'
sDivSlash ='/'
sDivSemiCol = ';'
seq = 0

class TRAY_ARD_Field(Enum):
    GLA_SVN = auto()  #linear_acceleration
    GAV_SVN = auto() #angular_velocity
    TEMPN = auto() #mpu9250 온도
    GAV_TUN = auto()   #가속도 3축 합산 절대값
    GOR_SVN = auto()  #orientation -> roll,pitch,yaw
    TEMPAHT_N = auto()  #AHT20 온도
    HUMIDAHT_N = auto() #AHT20 습도
    PARTICLEN = auto()  #미세먼지 (RED/IR/GREEN)
    RESULTN = auto()   #명령어처리결과

class MOTOR_CONTROL_CMD():
    def __init__(self):
        self.MBID = None
        self.MODE = None
        self.POS = None
        self.SPD = None
        self.ACC = None
        self.DECC = None
        self.CMD = None
        
        self.publisher = rospy.Publisher('/CMD_DEVICE', String, queue_size=10)
        
    def MakeCommand(self):
        self.cmd = []
        self.cmd.append('MBID:' + self.MBID)
        if self.CMD == 'WMOVE':
            self.cmd.append('MODE:' + self.MODE)
            self.cmd.append('POS:' + self.POS)
            self.cmd.append('SPD:' + self.SPD)
            self.cmd.append('ACC:' + self.ACC)
            self.cmd.append('DECC:' + self.DECC)
                
        self.cmd.append('CMD:' + self.CMD)
        self.cmd = sDivItemComma.join(self.cmd)
        # print(self.cmd)
        self.MotorTopic(self.cmd)     
           
    def calculateSpeed(self, angle):
        base_speed = 100
        max_speed = 1500
        speed_increment = 100
        
        speed = base_speed + (abs(angle) * speed_increment)
        if speed > max_speed:
            speed = max_speed

        # 최소 속도 제한 (예: 0)
        min_speed = 0
        if speed < min_speed:
            speed = min_speed

        return speed    
       
    def MotorRightFastMove(self):
        self.MBID = '30'
        self.MODE = '65'
        self.POS = '100000'
        self.SPD = '1000'
        self.ACC = '50'
        self.DECC = '100'
        self.CMD = 'WMOVE'
        self.MakeCommand()

    def MotorRightMove(self, angle):
        self.MBID = '30'
        self.MODE = '65'
        self.POS = '50000'
        self.SPD = str(self.calculateSpeed(angle))
        self.ACC = '40'
        self.DECC = '1000'
        self.CMD = 'WMOVE'
        # print(type())
        self.MakeCommand()
        
    def MotorRightSlowMove(self):
        self.MBID = '30'
        self.MODE = '65'
        self.POS = '100000'
        self.SPD = '100'
        self.ACC = '10'
        self.DECC = '100'
        self.CMD = 'WMOVE'
        self.MakeCommand()
        
    def MotorLeftFastMove(self):
        self.MBID = '29'
        self.MODE = '65'
        self.POS = '100000'
        self.SPD = '1000'
        self.ACC = '50'
        self.DECC = '100'
        self.CMD = 'WMOVE'
        self.MakeCommand()
        
    def MotorLeftMove(self, angle):
        self.MBID = '29'
        self.MODE = '65'
        self.POS = '50000'
        self.SPD = str(self.calculateSpeed(angle))
        self.ACC = '40'
        self.DECC = '1000'
        self.CMD = 'WMOVE'
        self.MakeCommand()
        
    def MotorLeftSlowMove(self):
        self.MBID = '29'
        self.MODE = '65'
        self.POS = '100000'
        self.SPD = '100'
        self.ACC = '50'
        self.DECC = '100'
        self.CMD = 'WMOVE'
        self.MakeCommand()         
           
    def MotorRightStop(self):
        self.MBID = '30'
        self.CMD = 'WSTOP'
        self.MakeCommand()
        
    def MotorLeftStop(self):
        self.MBID = '29'
        self.CMD = 'WSTOP'
        self.MakeCommand()
        
    def MotorAllStop(self):
        self.MotorRightStop()
        self.MotorLeftStop()
    
    def MotorDualFastMove(self):
        self.MotorRightFastMove()
        self.MotorLeftFastMove()
        
    def MotorDualMove(self):
        self.MotorRightMove()
        self.MotorLeftMove()   
                
    def MotorDualSlowMove(self):
        self.MotorRightSlowMove()
        self.MotorLeftSlowMove()   
            
    def MotorTopic(self, message):
        # 발행할 메시지 생성
        # message.data = 'MBID:30,MODE:65,POS:-1000000,SPD:10,ACC:20,DECC:20,CMD:Wmove'
        print(message)
        # 메시지 발행
        self.publisher.publish(message)    
        
class MyWidget(QMainWindow, Ui_MainWindow):
    
    def __init__(self):
        super(MyWidget, self).__init__()
        self.date = QDate.currentDate()
        self.setupUi(self)
        self.InitSlot()
        self.InitButton()
        self.InitCommand()
        self.InitFlag()

    def InitSlot(self):
        self.lineEdit_DI1_1.textChanged.connect(self.getSensor)  # DI_1
        self.lineEdit_DI2_1.textChanged.connect(self.getSensor)  # DI_2     
        self.lineEdit_DI3_1.textChanged.connect(self.getSensor)  # DI_3
        self.lineEdit_DI4_1.textChanged.connect(self.getSensor)  # DI_4
        self.lineEdit_DI5_1.textChanged.connect(self.getSensor)  # DI_5        
        self.lineEdit_DI6_1.textChanged.connect(self.getSensor)  # DI_6
        self.lineEdit_DI7_1.textChanged.connect(self.getSensor)  # DI_7
        
        self.lineEdit_DI1_2.textChanged.connect(self.getSensor)  # DI_1
        self.lineEdit_DI2_2.textChanged.connect(self.getSensor)  # DI_2         
        self.lineEdit_DI3_2.textChanged.connect(self.getSensor)  # DI_3
        self.lineEdit_DI4_2.textChanged.connect(self.getSensor)  # DI_4
        self.lineEdit_DI5_2.textChanged.connect(self.getSensor)  # DI_5        
        self.lineEdit_DI6_2.textChanged.connect(self.getSensor)  # DI_6
        self.lineEdit_DI7_2.textChanged.connect(self.getSensor)  # DI_7
        
        self.lineEdit_state0.textChanged.connect(self.getState) # STATE_0
        self.lineEdit_state1.textChanged.connect(self.getState) # STATE_1
        self.lineEdit_state2.textChanged.connect(self.getState) # STATE_2
        self.lineEdit_state4.textChanged.connect(self.getState) # STATE_4
        self.lineEdit_state5.textChanged.connect(self.getState) # STATE_5
        self.lineEdit_state6.textChanged.connect(self.getState) # STATE_6
        
        self.lineEdit_state0_2.textChanged.connect(self.getState) # STATE_0
        self.lineEdit_state1_2.textChanged.connect(self.getState) # STATE_1
        self.lineEdit_state2_2.textChanged.connect(self.getState) # STATE_2
        self.lineEdit_state4_2.textChanged.connect(self.getState) # STATE_4
        self.lineEdit_state5_2.textChanged.connect(self.getState) # STATE_5
        self.lineEdit_state6_2.textChanged.connect(self.getState) # STATE_6
 
        self.pushButton_START.clicked.connect(self.startCommand) # START
        self.pushButton_STOP.clicked.connect(self.stopCommand)   # STOP
        self.pushButton_READ.clicked.connect(self.readCommand)   # READ
        self.pushButton_RESET.clicked.connect(self.resetCommand) # RESET
        
        self.pushButton_START_2.clicked.connect(self.startCommand) # START
        self.pushButton_STOP_2.clicked.connect(self.stopCommand2)   # STOP
        self.pushButton_READ_2.clicked.connect(self.readCommand2)   # READ
        self.pushButton_RESET_2.clicked.connect(self.resetCommand2) # RESET
           
        self.comboBox_MBID.currentTextChanged.connect(self.comboBox_MBID_currentTextChanged)    # MBID
        self.comboBox_MBID_2.currentTextChanged.connect(self.comboBox_MBID_currentTextChanged2) # MBID
        
        self.pushButton.clicked.connect(self.alarmClearcommand)  # DI_1
    def InitButton(self):
        # button disable
        self.pushButton_READ.setEnabled(True)
        self.pushButton_START.setEnabled(False)
        self.pushButton_STOP.setEnabled(False)
        self.pushButton_RESET.setEnabled(False)
        self.pushButton_READ_2.setEnabled(True)
        self.pushButton_START_2.setEnabled(False)
        self.pushButton_STOP_2.setEnabled(False)
        self.pushButton_RESET_2.setEnabled(False)
    
    def InitCommand(self):
        self.lineEdit_MODE.setText('1')
        self.lineEdit_POS.setText('0')
        self.lineEdit_SPD.setText('500')
        self.lineEdit_ACC.setText('6000')
        self.lineEdit_DECC.setText('6000')
        self.comboBox_CMD.setCurrentIndex(0)
        self.comboBox_MBID.setCurrentIndex(0)
        self.lineEdit_MODE_2.setText('1')
        self.lineEdit_POS_2.setText('-2600000')
        self.lineEdit_SPD_2.setText('700')
        self.lineEdit_ACC_2.setText('6000')
        self.lineEdit_DECC_2.setText('6000')
        self.comboBox_CMD_2.setCurrentIndex(0)
        self.comboBox_MBID_2.setCurrentIndex(0)
        self.comboBox_MBID_2.setMaxVisibleItems(20)
        self.groupBox.setTitle(self.comboBox_MBID.currentText()) 
        self.groupBox_2.setTitle(self.comboBox_MBID_2.currentText())
        self.lineEdit_MBID.setText(self.comboBox_MBID.currentText().split('_')[1])
        self.lineEdit_MBID_2.setText(self.comboBox_MBID_2.currentText().split('_')[1])  
    
    def InitFlag(self):
        self.onFlag = False
        self.subscriber = None
        self.subscriber2 = None   
               
    def comboBox_MBID_currentTextChanged(self):
        self.lineEdit_MBID.setText(self.comboBox_MBID.currentText().split('_')[1])
        self.groupBox.setTitle(self.comboBox_MBID.currentText())
        
    def comboBox_MBID_currentTextChanged2(self):
        self.lineEdit_MBID_2.setText(self.comboBox_MBID_2.currentText().split('_')[1])    
        self.groupBox_2.setTitle(self.comboBox_MBID_2.currentText())
        
    def getDic_strArr(self, strTmp, spliterItemValue, spliterLine):
        dicReturn = {}
        file_list = strTmp.split(sep=spliterLine)
        for i in file_list:
            iCurrent = i.split(spliterItemValue, 1)
            if len(iCurrent) > 1:
                dicReturn[iCurrent[0]] = iCurrent[1]
        return dicReturn
    
    def getROS_Header(self, frame_id_str):
        msgTmp = Header()
        msgTmp.frame_id = frame_id_str
        return msgTmp
            
    def publish_ImuData(self, dicARD : dict, frame_id_Range):
        global seq
        
        Imu_msg = Imu()
        Imu_msg.header = self.getROS_Header(frame_id_Range)
        GAV_SVN = dicARD.get(TRAY_ARD_Field.GAV_SVN.name, None)
        GLA_SVN = dicARD.get(TRAY_ARD_Field.GLA_SVN.name, None)
        GOR_SVN = dicARD.get(TRAY_ARD_Field.GOR_SVN.name, None)
        if GAV_SVN == None or GLA_SVN == None or GOR_SVN == None:
            return
        
        GAV_SVNarr = GAV_SVN.split(sDivItemComma) #angular_velocity
        GLA_SVNarr = GLA_SVN.split(sDivItemComma) #linear_acceleration
        GOR_SVNarr = GOR_SVN.split(sDivItemComma) #orientation , roll,pitch,yaw

        roll = float(GOR_SVNarr[0])
        pitch = float(GOR_SVNarr[1])
        yaw = float(GOR_SVNarr[2])
        quaternion = quaternion_from_euler(roll, pitch, yaw)
        
        # 쿼터니언 설정
        Imu_msg.orientation.x = quaternion[0]
        Imu_msg.orientation.y = quaternion[1]
        Imu_msg.orientation.z = quaternion[2]
        Imu_msg.orientation.w = quaternion[3]

        Imu_msg.linear_acceleration.x = float(GLA_SVNarr[0])
        Imu_msg.linear_acceleration.y = float(GLA_SVNarr[1])
        Imu_msg.linear_acceleration.z = float(GLA_SVNarr[2])
        
        Imu_msg.angular_velocity.x = float(GAV_SVNarr[0])
        Imu_msg.angular_velocity.y = float(GAV_SVNarr[1])
        Imu_msg.angular_velocity.z = float(GAV_SVNarr[2])
        Imu_msg.header.stamp = rospy.Time.now()
        Imu_msg.header.seq = seq
        
        self.pub_IMU.publish(Imu_msg)
        motorctl = MOTOR_CONTROL_CMD()
        
        intPitch = int(pitch)
        if intPitch > 0:
            print(intPitch)
            # if intPitch > 10:
            #     motorctl.MotorLeftFastMove()
            # elif intPitch > 5:
            #     motorctl.MotorLeftMove()
            # else:
            #     motorctl.MotorLeftSlowMove()
            
            motorctl.MotorLeftMove(intPitch)
            
            motorctl.MotorRightStop()
            
        elif intPitch < 0:
            print(intPitch)
            # if intPitch < -10:
            #     motorctl.MotorRightFastMove()
            # elif intPitch < -5:
            #     motorctl.MotorRightMove()
            # else:
            #     motorctl.MotorRightSlowMove()
            motorctl.MotorRightMove(intPitch)
            
            motorctl.MotorLeftStop()
            
        elif intPitch == 0:
            print(intPitch)
            motorctl.MotorAllStop()
            # motorctl.MotorDualMove()
        print("------------------")
        seq += 1

    def alarmClearcommand(self):
        clearCommand = MOTOR_CONTROL_CMD()
        clearCommand.MBID = self.lineEdit_MBID.text()
        clearCommand.CMD = 'WALM_C'
        clearCommand.MakeCommand() 
            
    def startCommand(self):
        # 신호를 보낸 위젯 확인
        motorctl = MOTOR_CONTROL_CMD()
        motorctl.MBID = self.lineEdit_MBID.text()

        sender = self.sender()
        if sender is None:
            return
        if sender.objectName() == 'pushButton_START':
            if self.comboBox_CMD.currentText() == 'WMOVE':
                motorctl.MODE = self.lineEdit_MODE.text()
                motorctl.POS = self.lineEdit_POS.text()
                motorctl.SPD = self.lineEdit_SPD.text()
                motorctl.ACC = self.lineEdit_ACC.text() 
                motorctl.DECC = self.lineEdit_DECC.text()
                motorctl.CMD = self.comboBox_CMD.currentText()    
            elif self.comboBox_CMD.currentText() == 'WALM_C':
                motorctl.CMD = self.comboBox_CMD.currentText()
            elif self.comboBox_CMD.currentText() == 'WOFF':
                motorctl.CMD = self.comboBox_CMD.currentText()
            elif self.comboBox_CMD.currentText() == 'WZERO':
                motorctl.CMD = self.comboBox_CMD.currentText()
            elif self.comboBox_CMD.currentText() == 'WSPD':
                motorctl.CMD = self.comboBox_CMD.currentText()
                motorctl.SPD = self.lineEdit_SPD.text()
                motorctl.ACC = self.lineEdit_ACC.text()
                motorctl.DECC = self.lineEdit_DECC.text()
            elif self.comboBox_CMD.currentText() == 'WINIT':
                motorctl.CMD = self.comboBox_CMD.currentText()
            elif self.comboBox_CMD.currentText() == 'WLIMIT_OFF':
                motorctl.CMD = self.comboBox_CMD.currentText()
            elif self.comboBox_CMD.currentText() == 'WSTOP':
                motorctl.CMD = self.comboBox_CMD.currentText()
        elif sender.objectName() == 'pushButton_START_2':
            print(self.comboBox_MBID_2.currentText())
            if self.comboBox_CMD_2.currentText() == 'WMOVE':
                motorctl.MODE = self.lineEdit_MODE_2.text()
                motorctl.POS = self.lineEdit_POS_2.text()
                motorctl.SPD = self.lineEdit_SPD_2.text()
                motorctl.ACC = self.lineEdit_ACC_2.text()
                motorctl.DECC = self.lineEdit_DECC_2.text()
                motorctl.CMD = self.comboBox_CMD_2.currentText()
            elif self.comboBox_CMD_2.currentText() == 'WALM_C':
                motorctl.CMD = self.comboBox_CMD_2.currentText()
            elif self.comboBox_CMD_2.currentText() == 'WOFF':
                motorctl.CMD = self.comboBox_CMD_2.currentText()
            elif self.comboBox_CMD_2.currentText() == 'WZERO':
                motorctl.CMD = self.comboBox_CMD_2.currentText()
            elif self.comboBox_CMD_2.currentText() == 'WSPD':
                motorctl.CMD = self.comboBox_CMD_2.currentText()
                motorctl.SPD = self.lineEdit_SPD_2.text()
                motorctl.ACC = self.lineEdit_ACC_2.text()
                motorctl.DECC = self.lineEdit_DECC_2.text()
            elif self.comboBox_CMD_2.currentText() == 'WINIT':
                motorctl.CMD = self.comboBox_CMD_2.currentText()
            elif self.comboBox_CMD_2.currentText() == 'WLIMIT_OFF':
                motorctl.CMD = self.comboBox_CMD_2.currentText()
            elif self.comboBox_CMD_2.currentText() == 'WSTOP':
                motorctl.CMD = self.comboBox_CMD_2.currentText()
            else:
                return
        
        
        motorctl.MakeCommand()    
        # cmd = sDivItemComma.join(cmd)
        # self.publisherTopic(cmd)     
           
    def startCommand2(self):
        cmd = []
        cmd.append('MBID:' + self.lineEdit_MBID_2.text())

        if self.comboBox_CMD_2.currentText() == 'WMOVE':
            cmd.append('MODE:' + self.lineEdit_MODE_2.text())
            cmd.append('POS:' + self.lineEdit_POS_2.text())
            cmd.append('SPD:' + self.lineEdit_SPD_2.text())
            cmd.append('ACC:' + self.lineEdit_ACC_2.text())
            cmd.append('DECC:' + self.lineEdit_DECC_2.text())
            cmd.append('CMD:' + self.comboBox_CMD_2.currentText())
        elif self.comboBox_CMD_2.currentText() == 'WALM_C':
            cmd.append('CMD:' + self.comboBox_CMD_2.currentText())
        elif self.comboBox_CMD_2.currentText() == 'WOFF':
            cmd.append('CMD:' + self.comboBox_CMD_2.currentText())
        elif self.comboBox_CMD_2.currentText() == 'WZERO':
            cmd.append('CMD:' + self.comboBox_CMD_2.currentText())
        elif self.comboBox_CMD_2.currentText() == 'WSPD':
            cmd.append('CMD:' + self.comboBox_CMD_2.currentText())
            cmd.append('SPD:' + self.lineEdit_SPD_2.text())
            cmd.append('ACC:' + self.lineEdit_ACC_2.text())
            cmd.append('DECC:' + self.lineEdit_DECC_2.text())
        elif self.comboBox_CMD_2.currentText() == 'WINIT':
            cmd.append('CMD:' + self.comboBox_CMD_2.currentText())
        elif self.comboBox_CMD_2.currentText() == 'WLIMIT_OFF':
            cmd.append('CMD:' + self.comboBox_CMD_2.currentText())
        elif self.comboBox_CMD_2.currentText() == 'WSTOP':
            cmd.append('CMD:' + self.comboBox_CMD_2.currentText())
            
        cmd = sDivItemComma.join(cmd)
        self.publisherTopic(cmd)         
            
    def stopCommand(self):
        cmd = []
        cmd.append('MBID:' + self.lineEdit_MBID.text())
        cmd.append('CMD:Wstop')
        cmd = sDivItemComma.join(cmd)
        self.publisherTopic(cmd)
            
    def stopCommand2(self):
        cmd = []
        cmd.append('MBID:' + self.lineEdit_MBID_2.text())
        cmd.append('CMD:Wstop')
        cmd = sDivItemComma.join(cmd)
        self.publisherTopic(cmd)   
             
    def resetCommand(self):
        if self.subscriber is not None:
            self.subscriber.unregister()
            rospy.loginfo("Subscription stopped.")
            self.subscriber = None
        self.pushButton_READ.setEnabled(True)
        self.pushButton_RESET.setEnabled(False)
        self.resetState()
        
    def resetCommand2(self):
        if self.subscriber2 is not None:
            self.subscriber2.unregister()
            rospy.loginfo("Subscription2 stopped.")
            self.subscriber2 = None
        self.pushButton_READ_2.setEnabled(True)
        self.pushButton_RESET_2.setEnabled(False)
        self.resetState2()        
        
    def resetState(self):
        self.lineEdit_state0.setText('')
        self.lineEdit_state1.setText('')
        self.lineEdit_state2.setText('')
        self.lineEdit_state4.setText('')
        self.lineEdit_state5.setText('')
        self.lineEdit_state6.setText('')
        self.lineEdit_DI1_1.setText('')
        self.lineEdit_DI2_1.setText('')
        self.lineEdit_DI3_1.setText('')
        self.lineEdit_DI4_1.setText('')
        self.lineEdit_DI5_1.setText('')
        self.lineEdit_DI6_1.setText('')
        self.lineEdit_DI7_1.setText('')       
        
    def resetState2(self):
        self.lineEdit_state0_2.setText('')
        self.lineEdit_state1_2.setText('')
        self.lineEdit_state2_2.setText('')
        self.lineEdit_state4_2.setText('')
        self.lineEdit_state5_2.setText('')
        self.lineEdit_state6_2.setText('')
        self.lineEdit_DI1_2.setText('')
        self.lineEdit_DI2_2.setText('')
        self.lineEdit_DI3_2.setText('')
        self.lineEdit_DI4_2.setText('')
        self.lineEdit_DI5_2.setText('')
        self.lineEdit_DI6_2.setText('')
        self.lineEdit_DI7_2.setText('')      
                
    def readCommand(self):
        self.init_ros_node()
        self.pushButton_START.setEnabled(True)
        self.pushButton_STOP.setEnabled(True)
        self.pushButton_RESET.setEnabled(True)
        self.pushButton_READ.setEnabled(False)
        
    def readCommand2(self):
        self.init_ros_node2()
        self.pushButton_START_2.setEnabled(True)
        self.pushButton_STOP_2.setEnabled(True)
        self.pushButton_RESET_2.setEnabled(True)
        self.pushButton_READ_2.setEnabled(False)     
           
    def getState(self):
        # 신호를 보낸 위젯 확인
        sender = self.sender()
        if sender is None:
            return
        
        if sender.text() == '0':
            sender.setStyleSheet('background-color: red; color: red')
        elif sender.text() == '1':
            sender.setStyleSheet('background-color: green; color: green')
        else:
            sender.setStyleSheet('')
        
    def getSensor(self):
        # 신호를 보낸 위젯 확인
        sender = self.sender()
        if sender is None:
            return
        
        if sender.text() == '0':
            sender.setStyleSheet('background-color: green; color: green')
        elif sender.text() == '1':
            sender.setStyleSheet('background-color: red; color: red')
        else:
            sender.setStyleSheet('')
            
    def init_ros_node(self):
        # ROS 노드 초기화
        self.stopSubscribeTopic()
        sub_thread = threading.Thread(target=self.subscribeTopic, daemon=True)
        sub_thread.start()
        self.publisher = rospy.Publisher('/CMD_DEVICE', String, queue_size=10)
        
    def init_ros_node2(self):
        # ROS 노드 초기화
        self.stopSubscribeTopic2() 
        sub_thread = threading.Thread(target=self.subscribeTopic2, daemon=True)
        sub_thread.start()
        self.publisher = rospy.Publisher('/CMD_DEVICE', String, queue_size=10)      
                                
    def subscribeTopic(self):
        # 구독자(Subscriber) 생성
        self.subscriber = rospy.Subscriber(self.comboBox_MBID.currentText(), String, self.callbackCmd)
        # 루프 유지
        rospy.spin()   
        
    def subscribeTopic2(self):
        # 구독자(Subscriber) 생성
        self.subscriber2 = rospy.Subscriber(self.comboBox_MBID_2.currentText(), String, self.callbackCmd2)
        # 루프 유지
        rospy.spin()   
            
    def stopSubscribeTopic(self):
        if self.subscriber is not None:
            self.subscriber.unregister()
            rospy.loginfo("Subscription stopped.")
            self.subscriber = None
            
    def stopSubscribeTopic2(self):        
        if self.subscriber2 is not None:
            self.subscriber2.unregister()
            rospy.loginfo("Subscription stopped.")
            self.subscriber2 = None
   
    def publisherTopic(self, message):
        # 발행할 메시지 생성
        print(message)
        # message.data = 'MBID:30,MODE:65,POS:-1000000,SPD:10,ACC:20,DECC:20,CMD:Wmove'
        # 메시지 발행
        self.publisher.publish(message)
        
    def callbackCmd(self, data):
        di_1 = None
        di_2 = None
        di_3 = None
        di_4 = None
        di_5 = None
        di_6 = None
        di_7 = None

        for key in data.data.split(sDivItemComma):
            if key.find('DI_01') != -1:
                di_1 = key.split(sDivFieldColon)[1]
                self.lineEdit_DI1_1.setText(di_1)
            if key.find('DI_NOT') != -1:
                di_2 = key.split(sDivFieldColon)[1]
                self.lineEdit_DI2_1.setText(di_2)
            if key.find('DI_POT') != -1:
                di_3 = key.split(sDivFieldColon)[1]
                self.lineEdit_DI3_1.setText(di_3)
            if key.find('DI_ESTOP') != -1:
                di_4 = key.split(sDivFieldColon)[1]
                self.lineEdit_DI4_1.setText(di_4)
            if key.find('DI_HOME') != -1:
                di_5 = key.split(sDivFieldColon)[1]
                self.lineEdit_DI5_1.setText(di_5)
            if key.find('DI_06') != -1:
                di_6 = key.split(sDivFieldColon)[1]
                self.lineEdit_DI6_1.setText(di_6)
            if key.find('DI_07') != -1:
                di_7 = key.split(sDivFieldColon)[1]
                self.lineEdit_DI7_1.setText(di_7)
        
            if key.find('ST_FAULTY') != -1:
                state_0 = key.split(sDivFieldColon)[1]
                self.lineEdit_state0.setText(state_0)
            if key.find('ST_ENABLE') != -1:
                state_1 = key.split(sDivFieldColon)[1]
                self.lineEdit_state1.setText(state_1)
            if key.find('ST_RUNNING') != -1:
                state_2 = key.split(sDivFieldColon)[1]
                self.lineEdit_state2.setText(state_2)
            if key.find('ST_CMD_FINISH') != -1:
                state_4 = key.split(sDivFieldColon)[1]
                self.lineEdit_state4.setText(state_4)
            if key.find('ST_PATH_FINISH') != -1:
                state_5 = key.split(sDivFieldColon)[1]
                self.lineEdit_state5.setText(state_5)
            if key.find('ST_HOME_FINISH') != -1:
                state_6 = key.split(sDivFieldColon)[1]                      
                self.lineEdit_state6.setText(state_6)
        
        self.groupBox.setTitle(self.comboBox_MBID.currentText())
        self.lineEdit_MBID.setText(self.comboBox_MBID.currentText().split('_')[1])
        
        rospy.loginfo("%s",data.data)
        
    def callbackCmd2(self, data):
        di_1_2 = None
        di_2_2 = None
        di_3_2 = None
        di_4_2 = None
        di_5_2 = None
        di_6_2 = None
        di_7_2 = None
        state_0_2 = None
        state_1_2 = None
        state_2_2 = None
        state_4_2 = None
        state_5_2 = None
        state_6_2 = None
        for key in data.data.split(sDivItemComma):
            if key.find('DI_01') != -1:
                di_1_2 = key.split(sDivFieldColon)[1]
            if key.find('DI_NOT') != -1:
                di_2_2 = key.split(sDivFieldColon)[1]
            if key.find('DI_POT') != -1:
                di_3_2 = key.split(sDivFieldColon)[1]
            if key.find('DI_ESTOP') != -1:
                di_4_2 = key.split(sDivFieldColon)[1]
            if key.find('DI_HOME') != -1:
                di_5_2 = key.split(sDivFieldColon)[1]
            if key.find('DI_06') != -1:
                di_6_2 = key.split(sDivFieldColon)[1]
            if key.find('DI_07') != -1:
                di_7_2 = key.split(sDivFieldColon)[1]
            if key.find('ST_FAULTY') != -1:
                state_0_2 = key.split(sDivFieldColon)[1]
            if key.find('ST_ENABLE') != -1:
                state_1_2 = key.split(sDivFieldColon)[1]
            if key.find('ST_RUNNING') != -1:
                state_2_2 = key.split(sDivFieldColon)[1]
            if key.find('ST_CMD_FINISH') != -1:
                state_4_2 = key.split(sDivFieldColon)[1]
            if key.find('ST_PATH_FINISH') != -1:
                state_5_2 = key.split(sDivFieldColon)[1]
            if key.find('ST_HOME_FINISH') != -1:
                state_6_2 = key.split(sDivFieldColon)[1]                   
                    
            self.lineEdit_DI1_2.setText(di_1_2)
            self.lineEdit_DI2_2.setText(di_2_2)
            self.lineEdit_DI3_2.setText(di_3_2)
            self.lineEdit_DI4_2.setText(di_4_2)
            self.lineEdit_DI5_2.setText(di_5_2)
            self.lineEdit_DI6_2.setText(di_6_2)
            self.lineEdit_DI7_2.setText(di_7_2)
            
            self.lineEdit_state0_2.setText(state_0_2)
            self.lineEdit_state1_2.setText(state_1_2)
            self.lineEdit_state2_2.setText(state_2_2)
            self.lineEdit_state4_2.setText(state_4_2)
            self.lineEdit_state5_2.setText(state_5_2)
            self.lineEdit_state6_2.setText(state_6_2)
            
            self.groupBox_2.setTitle(self.comboBox_MBID_2.currentText())
            self.lineEdit_MBID_2.setText(self.comboBox_MBID_2.currentText().split('_')[1])
            
            rospy.loginfo("%s",data.data)        
if __name__ == '__main__':
    rospy.init_node('sensorTest_frank')
    app = QApplication(sys.argv)
    widget = MyWidget()
    widget.show()
    
    sys.exit(app.exec_())
    
    
