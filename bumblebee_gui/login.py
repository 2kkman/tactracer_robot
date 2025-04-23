import os
import sys

from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtGui import *
from PyQt5.QtCore import *

import rospy
from std_msgs.msg import String, Time
from importlib import reload
from ui.KeypadDialog import KeypadDialog
from testgui.ui.mainwindow_ui import Ui_MainWindow
from ui.moving import Ui_Form
from datetime import datetime
from ttacer.parser import *

import threading

class WindowClass(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.counter = 0

        # self.subscriber = rospy.Subscriber('DeliveryTable_topic', String, self.callback)
        self.pushButton.clicked.connect(self.evt_send_message)

        # self.label_4.mousePressEvent = self.mousePressEvent
    def resetCount(self):
        self.count = 0

    # def mousePressEvent(self, e):
    #     self.count += 1
    #     if self.count >= 10:
    #         qApp.quit()
    #     self.timer.start(500) # 500 milliseconds

    def evtmousePress(self, e):
        self.count += 1
        if self.count >= 7:
            qApp.quit()
        self.timer.start(500) # 500 milliseconds

    def btn_main_to_second(self):
        self.hide()                     # 메인윈도우 숨김
        self.second = secondwindow()    #
        self.second.exec()              # 두번째 창을 닫을 때 까지 기다림
        self.showFullScreen()           # 두번째 창을 닫으면 다시 첫 번째 창이 보여짐짐

    def btn_main_to_movewindow(self):
        self.hide()                     # 메인윈도우 숨김
        self.second = secondwindow()    #
        self.second.exec()              # 두번째 창을 닫을 때 까지 기다림
        self.showFullScreen()           # 두번째 창을 닫으면 다시 첫 번째 창이 보여짐짐

    def evt_send_message(self):
        global sResult
        global send_data
        global pub
        sResult= 'RESULT'+ chSplit1 +'OK'+ chSplit3 +'CMDN'+ chSplit1 +'U'+ chSplit3 +'MSGN'+ chSplit1 +'SPG2060N'+ chSplit3 +'NESTCHARGE_NC'+ chSplit1 +'0'+ chSplit3 +'BREAK_V_NC'+ chSplit1 +'0'+ chSplit3 +'USBHUB_TXRX_NC'+ chSplit1 +'0'+ chSplit3 +'USBHUB_VCC_NC'+ chSplit1 +'0'+ chSplit3 + chSplit3 +'GAV_TUN'+ chSplit1 +'0.00'+ chSplit3 + 'TEMPN'+ chSplit1 +'30.67'+ chSplit3 +'TEMPAHT_N'+ chSplit1 +'27.06'+ chSplit3 +'HUMIDAHT_N'+ chSplit1 +'18.88'
        #sResult = 'ID' + chSplit1 + 'BumbleBee1'+ chSplit3 + 'trayA' + chSplit1 + self.spinbox1.text() + chSplit3 + 'trayB' + chSplit1 + self.spinbox2.text() + chSplit3 + 'state' + chSplit1 + 'move1' 
        # for key, value in sResult.items():
        #     print(f'{key}층 : {value}번 테이블')
        msgDeliveryTable = QMessageBox()
        msgDeliveryTable.setFixedSize(500, 500)
        msgDeliveryTable.setText(f"Tray A : {self.spinbox1.text()}번 Table\nTray B : {self.spinbox2.text()}번 Table\n배달 확인")
        msgDeliveryTable.setWindowTitle("테이블 배달")
        msgDeliveryTable.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)

        button_list = msgDeliveryTable.findChildren(QPushButton)
        for button in button_list:
            button.setFixedSize(100, 50)
        msgDeliveryTable.setStyleSheet("QMessageBox{background-color: #FDDE45; border-radius: 5px; font-size: 24px; padding: 10px; text-align: center;} QPushButton {font-size: 20px;}")
        bOK = msgDeliveryTable.exec_()

        pub = rospy.Publisher('DeliveryTable_topic', String, queue_size=10)

        if bOK == QMessageBox.Ok:
            # getTimestamp_str()
            # send_data = String(f'DeliveryTable:{sResult}')
            print(sResult)
            pub.publish(sResult)
            self.spinbox1.setValue(0)
            self.spinbox2.setValue(0)
            # self.btn_main_to_movewindow()
            # self.spinbox3.setValue(0)
        else :
            send_data = String('no')
            
        
class secondwindow(QDialog,QWidget, Ui_Form):
    def __init__(self, parent=None):
        super(secondwindow,self).__init__(parent)
        global send_data
        global table_Text
        global sResult
        global received_message

        table_Text = ''
        received_message = ''
        
        self.parent = parent
        self.initUi()
        self.showFullScreen()
        # self.table_label.setText(f'이동중 ...')
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.close)
        self.timer.stop()
        
        self.btnTable.setVisible(True)
        self.lblImage.setVisible(True)        
        self.btnConfirm.setVisible(False)     
        # send_data = String(f'DeliveryTable:{sResult}')
        
        self.btnTable.clicked.connect(self.btnKeypad)
        self.btnConfirm.clicked.connect(self.btn_return)

            
        pub.publish(sResult)
        self.table_label.setText(f"{table_Text}")
        
    def initUi(self):
        self.setupUi(self)

    def btn_return(self):
        
        sResult = 'ID' + chSplit1 + 'BumbleBee1'+ chSplit3 + 'trayA' + chSplit1 + self.spinbox1.text() + chSplit3 + 'trayB' + chSplit1 + self.spinbox2.text() + chSplit3 + 'state' + chSplit1 + 'btn_return' + chSplit3 + 'time' + chSplit1 + '0'
        pub.publish(sResult)

    def timer_second_to_main(self):
        self.close()
        self.__init__()

    def btnKeypad(self):
        global input_Table
        input_Table = ''
        
        self.inputTable = KeypadDialog()
        if self.inputTable.exec_() == QDialog.Accepted: # 두번째 창을 닫을 때 까지 기다림
            input_Table = int(self.inputTable.lineEdit.text())

            
            # setTimestamp()
            send_data = String(f'{sResult}')
            pub.publish(send_data)
            
            self.btnTable.setVisible(False)

            # print(f"입력된 테이블: {input_Table}")
            self.close()
            self.__init__()
 

def setState(state):
    sResult['state'] = state() 
    
def callback(data):
    # parsed_data = data.data.split(chSplit3)
    # key = parsed_data[0]
    # value = parsed_data[1]
    
    CmdProcess(data.data)
    # 수신한 문자열 메시지 출력
    rospy.loginfo("Received message: %s", data.data)
    
def subscribe_to_topic():
    # 구독자(Subscriber) 생성
    rospy.Subscriber('DeliveryTable_topic', String, callback)
    
    # 루프 유지
    rospy.spin()


def CmdProcess(strTmp):
    
    sCmd = GetKey(strTmp, chSplit1)
    sParam = GetVal(strTmp, chSplit1)

    sResult = GetNGMsg(sCmd, sParam)
    
    sKey2 = ""
    sVal2 = ""
    iKey = -1
    iVal = -1
    if sParam.isdigit():
        sKey2 = GetKey(sParam, chSplit2)
        sVal2 = GetVal(sParam, chSplit2)
        if (sKey2.isdigit() and sVal2.isdigit()):
            iKey = int(sKey2)
            iVal = int(sVal2)
    print("Recv : " + strTmp);    
            
def GetNGMsg(cmd, sParam): 
    sResult = "RESULTN:NG,CMDN:" + cmd + ",MSGN:Param Parse error,DATA:" + sParam
    return sResult

if __name__ == '__main__':
    try:
        rospy.init_node('my_node',anonymous=True)
        sub_thread = threading.Thread(target=subscribe_to_topic, daemon=True)
        sub_thread.start()
        app = QApplication(sys.argv)
        mainWindow = WindowClass()
        mainWindow.showFullScreen()
        app.exec_()
        
    except rospy.ROSInterruptException:
        pass