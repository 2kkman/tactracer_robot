
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import QDate
from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo
from PyQt5 import uic
import rospy
from std_msgs.msg import String, Header
from sensor_msgs.msg import Imu, Temperature, RelativeHumidity
# from ui.sensorTest_ui import Ui_MainWindow
from ui.sensorTest_big_ui import Ui_MainWindow

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
        
        self.com = QSerialPort()
        self.com.readyRead.connect(self.Com_Receive_Data)
        self.CreateSignalSlot()
        
        self.pushButton_START.setEnabled(False)
        self.pushButton_STOP.setEnabled(False)
        self.pushButton_RESET.setEnabled(False)
        
        self.lineEdit_MBID.text() == self.groupBox.title()
        
        self.lineEdit_MODE.setText('65')
        self.lineEdit_POS.setText('-1000000')
        self.lineEdit_SPD.setText('100')
        self.lineEdit_ACC.setText('100')
        self.lineEdit_DECC.setText('100')
        self.comboBox_CMD.itemText(0)
        self.comboBox_MBID.itemText(0)
        
        self.lineEdit_MBID.setText(self.comboBox_MBID.currentText().split('_')[1])
        self.comboBox_MBID.currentTextChanged.connect(self.comboBox_MBID_currentTextChanged)
        self.onFlag = False
        self.subscriber = None
        self.Com_Refresh_Button_Clicked()
        
    def CreateSignalSlot(self):
        self.lineEdit_DI_1.textChanged.connect(self.getSensor)
        self.lineEdit_NOT.textChanged.connect(self.getSensor)        
        self.lineEdit_POT.textChanged.connect(self.getSensor)
        self.lineEdit_STOP.textChanged.connect(self.getSensor)
        self.lineEdit_HOME.textChanged.connect(self.getSensor)        
        self.lineEdit_DI_6.textChanged.connect(self.getSensor)
        self.lineEdit_DI_7.textChanged.connect(self.getSensor)
        
        self.lineEdit_state_0.textChanged.connect(self.getState)
        self.lineEdit_state_1.textChanged.connect(self.getState)
        self.lineEdit_state_2.textChanged.connect(self.getState)
        self.lineEdit_state_4.textChanged.connect(self.getState)
        self.lineEdit_state_5.textChanged.connect(self.getState)
        self.lineEdit_state_6.textChanged.connect(self.getState)
        
        self.Com_Open_Button.clicked.connect(self.Com_Open_Button_clicked)
        self.Com_Close_Button.clicked.connect(self.Com_Close_Button_clicked)
        self.Com_Refresh_Button.clicked.connect(self.Com_Refresh_Button_Clicked)
        self.Com_Clear_Button.clicked.connect(self.Com_Clear_Button_clicked)
        
        self.pushButton_START.clicked.connect(self.startCommand)
        self.pushButton_STOP.clicked.connect(self.stopCommand)
        self.pushButton_READ.clicked.connect(self.readCommand)
        self.pushButton_RESET.clicked.connect(self.resetCommand) 
    
    def comboBox_MBID_currentTextChanged(self):
        self.lineEdit_MBID.setText(self.comboBox_MBID.currentText().split('_')[1])
               
    def Com_Clear_Button_clicked(self):
        self.Com_Receive_PlainTextEdit.clear()
        
    def Com_Open_Button_clicked(self):
        # print('serial open')
                #### com Open Code here ####
        # comName = '/dev/ttyACM0'
        comName = self.Com_Name_Combo.currentText()
        comBaud = int(self.Com_Baud_Combo.currentText())
        self.pub_IMU = rospy.Publisher('TRAY_IMU',Imu,queue_size=10)   
        
        self.com.setPortName(comName)
        self.com.setBaudRate(comBaud)
        
        self.com.setDataBits(QSerialPort.Data8)
        self.com.setParity(QSerialPort.NoParity)
        self.com.setStopBits(QSerialPort.OneStop)
        
        if not self.com.open(QSerialPort.ReadWrite):
            print("시리얼 포트 열기 실패")
            return

        print("시리얼 포트 설정 완료")
        
        self.Com_Close_Button.setEnabled(True)
        self.Com_Open_Button.setEnabled(False)
        self.Com_Refresh_Button.setEnabled(False)
        self.Com_Name_Combo.setEnabled(False)
        self.Com_Baud_Combo.setEnabled(False)
        self.Com_isOpenOrNot_Label.setText('Open')
        
    def Com_Close_Button_clicked(self):
        self.com.close()
        self.Com_Close_Button.setEnabled(False)
        self.Com_Open_Button.setEnabled(True)
        self.Com_Refresh_Button.setEnabled(True)
        self.Com_Name_Combo.setEnabled(True)
        self.Com_Baud_Combo.setEnabled(True)
        self.Com_isOpenOrNot_Label.setText('Close')
        
    def Com_Refresh_Button_Clicked(self):
        
        self.Com_Baud_Combo.clear()
        baud_list = [50, 75, 110, 134, 150, 200, 300, 600, 1200, 1800, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 500000, 576000, 921600]
        baud_list=list(map(str, baud_list))        
        self.Com_Baud_Combo.addItems(baud_list)
        self.Com_Baud_Combo.setCurrentText("115200")
        
        self.Com_Name_Combo.clear()
        com = QSerialPort()
        
        
        # com.setBaudRate(QSerialPort.Baud115200)
        com_list = QSerialPortInfo.availablePorts()
        # self.Com_Baud_Combo.addItems(str(info.standardBaudRates()))
        # print(info.standardBaudRates())
        for info in com_list:
            com.setPort(info)
            if com.open(QSerialPort.ReadWrite):
                self.Com_Name_Combo.addItem(info.portName())
                com.close()    
                
    def Com_Receive_Data(self):
        try:
            incoming_data = self.com.readLine().data().decode('ascii').rstrip()
        except:
            return
        
        # sensor_data_parts = incoming_data.split('`')
            
        # for sensor_data in sensor_data_parts:
        #     if sensor_data:
        #         name, values = sensor_data.split(':')
        #         if name == 'GOR_SVN':                    
        #             values = values.split(',')
        #             print(f"{name}: {values}")
        
        
                    # if values[0] > '0':
                    #     self.startCommand()
                    # else:
                    #     self.stopCommand()
                    
        # self.Com_Receive_PlainTextEdit.appendPlainText(incoming_data)
        dicTmp = self.getDic_strArr(incoming_data,sDivFieldColon,sDivEmart)
        self.publish_ImuData(dicTmp,'map')
        
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
    
    def startCommand(self):
        cmd = []
        cmd.append('MBID:' + self.lineEdit_MBID.text())

        if self.comboBox_CMD.currentText() == 'WMOVE':
            cmd.append('MODE:' + self.lineEdit_MODE.text())
            cmd.append('POS:' + self.lineEdit_POS.text())
            cmd.append('SPD:' + self.lineEdit_SPD.text())
            cmd.append('ACC:' + self.lineEdit_ACC.text())
            cmd.append('DECC:' + self.lineEdit_DECC.text())
            cmd.append('CMD:' + self.comboBox_CMD.currentText())
        elif self.comboBox_CMD.currentText() == 'WALM_C':
            cmd.append('CMD:' + self.comboBox_CMD.currentText())
        elif self.comboBox_CMD.currentText() == 'WOFF':
            cmd.append('CMD:' + self.comboBox_CMD.currentText())
        elif self.comboBox_CMD.currentText() == 'WZERO':
            cmd.append('CMD:' + self.comboBox_CMD.currentText())
        elif self.comboBox_CMD.currentText() == 'WSPD':
            cmd.append('CMD:' + self.comboBox_CMD.currentText())
            cmd.append('SPD:' + self.lineEdit_SPD.text())
            cmd.append('ACC:' + self.lineEdit_ACC.text())
            cmd.append('DECC:' + self.lineEdit_DECC.text())
        elif self.comboBox_CMD.currentText() == 'WINIT':
            cmd.append('CMD:' + self.comboBox_CMD.currentText())
        elif self.comboBox_CMD.currentText() == 'WLIMIT_OFF':
            cmd.append('CMD:' + self.comboBox_CMD.currentText())
        elif self.comboBox_CMD.currentText() == 'WSTOP':
            cmd.append('CMD:' + self.comboBox_CMD.currentText())
            
        cmd = sDivItemComma.join(cmd)
        self.publisherTopic(cmd)        
        
    def stopCommand(self):
        cmd = []
        cmd.append('MBID:' + self.lineEdit_MBID.text())
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
        
    def resetState(self):
        self.lineEdit_state_0.setText('')
        self.lineEdit_state_1.setText('')
        self.lineEdit_state_2.setText('')
        self.lineEdit_state_4.setText('')
        self.lineEdit_state_5.setText('')
        self.lineEdit_state_6.setText('')
        self.lineEdit_NOT.setText('')
        self.lineEdit_POT.setText('')
        self.lineEdit_HOME.setText('')
        self.lineEdit_STOP.setText('')
        self.lineEdit_DI_1.setText('')
        self.lineEdit_DI_6.setText('')
        self.lineEdit_DI_7.setText('')       
        
    def readCommand(self):
        self.init_ros_node()
        self.pushButton_START.setEnabled(True)
        self.pushButton_STOP.setEnabled(True)
        self.pushButton_RESET.setEnabled(True)
        self.pushButton_READ.setEnabled(False)
        
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
        if self.subscriber is not None:
            self.subscriber.unregister()
            rospy.loginfo("Subscription stopped.")
            self.subscriber = None
            
        sub_thread = threading.Thread(target=self.subscribeTopic, daemon=True)
        sub_thread.start()
        self.publisher = rospy.Publisher('/CMD_DEVICE', String, queue_size=10)
                               
    def subscribeTopic(self):
        # 구독자(Subscriber) 생성
        self.subscriber = rospy.Subscriber(self.comboBox_MBID.currentText(), String, self.callbackCmd)
        # 루프 유지
        rospy.spin()   
        
    def stopSubscribeTopic(self):
        if self.subscriber is not None:
            self.subscriber.unregister()
            rospy.loginfo("Subscription stopped.")
            self.subscriber = None
   
    def publisherTopic(self, message):
        # 발행할 메시지 생성
        print(message)
        # message.data = 'MBID:30,MODE:65,POS:-1000000,SPD:10,ACC:20,DECC:20,CMD:Wmove'
        # 메시지 발행
        self.publisher.publish(message)
        
    def callbackCmd(self, data):
        di_1 = None
        di_pot = None
        di_home = None
        di_not = None
        di_stop = None
        di_6 = None
        di_7 = None

        for key in data.data.split(sDivItemComma):
            if key.find('DI_POT') != -1:
                di_pot = key.split(sDivFieldColon)[1]
            if key.find('DI_HOME') != -1:
                di_home = key.split(sDivFieldColon)[1]
            if key.find('DI_NOT') != -1:
                di_not = key.split(sDivFieldColon)[1]
            if key.find('DI_ESTOP') != -1:
                di_stop = key.split(sDivFieldColon)[1]
            if key.find('DI_01') != -1:
                di_1 = key.split(sDivFieldColon)[1]
            if key.find('DI_06') != -1:
                 di_6 = key.split(sDivFieldColon)[1]
            if key.find('DI_07') != -1:
                 di_7 = key.split(sDivFieldColon)[1]
            if key.find('ST_FAULTY') != -1:
                 state_0 = key.split(sDivFieldColon)[1]
            if key.find('ST_ENABLE') != -1:
                 state_1 = key.split(sDivFieldColon)[1]
            if key.find('ST_RUNNING') != -1:
                 state_2 = key.split(sDivFieldColon)[1]
            if key.find('ST_CMD_FINISH') != -1:
                  state_4 = key.split(sDivFieldColon)[1]
            if key.find('ST_PATH_FINISH') != -1:
                state_5 = key.split(sDivFieldColon)[1]
            if key.find('ST_HOME_FINISH') != -1:
                state_6 = key.split(sDivFieldColon)[1]                   
                
        self.lineEdit_NOT.setText(di_not)
        self.lineEdit_POT.setText(di_pot)
        self.lineEdit_HOME.setText(di_home)
        self.lineEdit_STOP.setText(di_stop)
        self.lineEdit_DI_1.setText(di_1)
        self.lineEdit_DI_6.setText(di_6)
        self.lineEdit_DI_7.setText(di_7)
        
        self.lineEdit_state_0.setText(state_0)
        self.lineEdit_state_1.setText(state_1)
        self.lineEdit_state_2.setText(state_2)
        self.lineEdit_state_4.setText(state_4)
        self.lineEdit_state_5.setText(state_5)
        self.lineEdit_state_6.setText(state_6)
        
        self.groupBox.setTitle(self.comboBox_MBID.currentText())
        self.lineEdit_MBID.setText(self.comboBox_MBID.currentText().split('_')[1])
        
        rospy.loginfo("%s",data.data)
        
if __name__ == '__main__':
    rospy.init_node('sensorTest_frank')
    app = QApplication(sys.argv)
    widget = MyWidget()
    widget.show()
    
    sys.exit(app.exec_())
    
    
