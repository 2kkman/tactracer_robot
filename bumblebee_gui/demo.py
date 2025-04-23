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

import threading

chSplit1 = ':'
chSplit2 = ','
chSplit3 = '`'

def resource_path(relative_path):
    base_path = getattr(sys, "_MEIPASS", os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(base_path, relative_path)

# form = resource_path('ui/mainwindow.ui')
# form_class = uic.loadUiType(form)[0]

# form_second = resource_path('ui/moving.ui')
# form_secondwindow = uic.loadUiType(form_second)[0]

msgfont = QFont('Noto Sans Mono CJK KR', 16)

class WindowClass(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.counter = 0

        self.count = 0
        self.maximumLevel = 3
        self.minimumLevel = -3
        self.floor_level = 0
        
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.resetCount)
        self.timer.setSingleShot(True)
        # self.subscriber = rospy.Subscriber('DeliveryTable_topic', String, self.callback)
        self.pushButton.clicked.connect(self.evt_send_message)
        self.floor_reset.clicked.connect(self.evt_floor_default)
        self.floor_up.clicked.connect(self.evt_floor_up)
        self.floor_down.clicked.connect(self.evt_floor_down)
        
        

        # self.label_4.mousePressEvent = self.mousePressEvent
    def resetCount(self):
        self.count = 0

    def mousePressEvent(self, e):
        self.count += 1
        if self.count >= 10:
            qApp.quit()
        self.timer.start(500) # 500 milliseconds

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
        
    def evt_floor_default(self):
        self.floor_level = 0
        self.floor_label.setText(f"{self.floor_level}")

    def evt_floor_up(self):
        self.floor_level += 1 
        self.floor_level = min(self.floor_level, self.maximumLevel)
        self.floor_label.setText(f"{self.floor_level}")
        
    def evt_floor_down(self):
        self.floor_level -= 1 
        self.floor_level = max(self.floor_level, self.minimumLevel)
        self.floor_label.setText(f"{self.floor_level}")        
               
    def evt_send_message(self):
        global sResult
        global send_data
        global pub

        floor_level_value = str(self.floor_label.text())
        # print(f"floor_level_value : {floor_level_value}")
        
        sResult = 'ID' + chSplit1 + 'BumbleBee1'+ chSplit3 + 'TRAY_A' + chSplit1 + self.spinbox1.text() + chSplit3 + 'TRAY_B' + chSplit1 + self.spinbox2.text() + chSplit3 +'FLOOR_LEVEL' + chSplit1 + str(floor_level_value) + chSplit3 + 'STATE' + chSplit1 + 'move1' + chSplit3 + 'TIME' + chSplit1 + getTimestamp()
        # for key, value in sResult.items():
        #     print(f'{key}층 : {value}번 테이블')
        # print(f'sResult : {sResult}')
        
        msgDeliveryTable = QMessageBox()
        # msgDeliveryTable.setStyleSheet("QLabel{min-width:200 px; font-size: 13px;} QPushButton{ width:25px; font-size: 13px; }");
        # DeliveryTable.setFont(msgfont)
        msgDeliveryTable.setText(f"A :\t{self.spinbox1.text()} \nB :\t{self.spinbox2.text()}")
        msgDeliveryTable.setWindowTitle("테이블 배달")
        # custom_yes_button  = QPushButton("확인")
        # custom_no_button  = QPushButton("취소")
        # msgDeliveryTable.addButton(custom_yes_button , QMessageBox.YesRole)
        # msgDeliveryTable.addButton(custom_no_button , QMessageBox.NoRole)
        msgDeliveryTable.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)

        button_list = msgDeliveryTable.findChildren(QPushButton)
        for button in button_list:
            button.setFixedSize(100, 40)
        msgDeliveryTable.setStyleSheet("QMessageBox{background-color: #FDDE45; border-radius: 5px; font-size: 24px; padding: 10px; text-align: center;} QLabel{min-width: 300px; text-align: center; font-size: 36px;} QPushButton {font-size: 20px;}")
        bOK = msgDeliveryTable.exec_()

        pub = rospy.Publisher('DeliveryTable_topic', String, queue_size=1)

        if bOK == QMessageBox.Ok:
            # getTimestamp_str()
            send_data = String(f'DeliveryTable:{sResult}')
            # print(f"send_data : {send_data}")
            pub.publish(send_data)
            # self.spinbox1.setValue(0)
            # self.spinbox2.setValue(0)
            # self.floor_label.setText(f"{self.floor_default}")
            # print(self.floor_label.text())
            # self.btn_main_to_movewindow()
            # self.spinbox3.setValue(0)
        # else:
        #     send_data = String('no')
        #     print(f"send_data : {send_data}")
            
            
        
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
        self.table_label.setText(f'이동중 ...')
        # self.timer = QTimer(self)
        # self.timer.timeout.connect(self.close)
        # self.timer.stop()
        
        self.btnTable.setVisible(True)
        self.lblImage.setVisible(True)        
        self.btnConfirm.setVisible(False)     
        # send_data = String(f'DeliveryTable:{sResult}')
        
        self.btnTable.clicked.connect(self.btnKeypad)
        self.btnConfirm.clicked.connect(self.btn_return)
        # pub.publish(send_data)
        print(f"sResult: {type(sResult)}")
        # print(f"sResult['TRAY_A']:{sResult['TRAY_A']}")
        if sResult['TRAY_A'] == sResult['TRAY_B'] and sResult['state'] == 'move1':
            # getTimestamp_str()
            setTimestamp()
            sResult['state'] = 'move2'
            # send_data = String(f'DeliveryTable:{sResult}')
            send_data = String(f'{sResult}')
            pub.publish(send_data)

    
        # print(f"state : {sResult['state']}")    
        if sResult['state'] == 'move1':
            table_Text = f"{sResult['TRAY_A']}번 테이블 이동중 ..."
        elif sResult['state'] == 'move2':
            table_Text = f"{sResult['TRAY_B']}번 테이블 이동중 ..."
        elif sResult['state'] == 'move3':
            table_Text = f"복귀중 ..."
            self.btnTable.setVisible(False)
            self.lblImage.setVisible(True)        
            self.btnConfirm.setVisible(True)                
        # print(f"table_Text : {table_Text}")       
        if sResult['state'] == 'trayA':      
            if int(sResult['trayA']) == input_Table:
                self.btnConfirm.setVisible(True)    
                self.btnTable.setVisible(False)
                self.lblImage.setVisible(False)
                # table_Text+=(f"Get to the Table\nThe food you ordered is out\n on the Tray 1")
                table_Text+=("트레이 A\n주문하신 메뉴가 나왔습니다\n메뉴를 확인해주세요")

                # getTimestamp_str()
                sResult['state'] = 'move2'
                # send_data = String(f'DeliveryTable:{sResult}')
            else:
                pass

        elif sResult['state'] == 'trayB':
            
            if int(sResult['trayB']) == input_Table:
                self.btnConfirm.setVisible(True)    
                self.btnTable.setVisible(False)
                self.lblImage.setVisible(False)        
                
                # table_Text+=(f"Get to the Table\nThe foo d you ordered is out\n on the Tray 2")
                table_Text+=("트레이 B\n주문하신 메뉴가 나왔습니다\n메뉴를 확인해주세요")
                # send_data = String(f'DeliveryTable:{sResult}')
            else:
                pass
            
        elif sResult['state'] == 'home':
            table_Text = f"복귀완료"
            self.btnTable.setVisible(False)
            self.lblImage.setVisible(False)        
            self.btnConfirm.setVisible(True)  
            sResult['state'] = 'complete' 
            setTimestamp()
            send_data = String(f'{sResult}')
            
        pub.publish(send_data)
        self.table_label.setText(f"{table_Text}")
        
    def initUi(self):
        self.setupUi(self)

    def btn_return(self):

        if sResult['state'] == 'complete':
            self.close()                    #클릭시 종료됨.
        elif sResult['state'] == 'trayB':
            setTimestamp()
            sResult['state'] = 'move3'
            send_data = String(f'{sResult}')
            pub.publish(send_data)
            self.close()
            self.__init__()
        elif sResult['state'] == 'move3':
            setTimestamp()
            sResult['state'] = 'home'
            send_data = String(f'{sResult}')
            pub.publish(send_data)
            self.close()
            self.__init__()
        else:
            self.close()
            self.__init__()

    def timer_second_to_main(self):
        self.close()
        self.__init__()

    def btnKeypad(self):
        global input_Table
        input_Table = ''
        
        self.inputTable = KeypadDialog()
        if self.inputTable.exec_() == QDialog.Accepted: # 두번째 창을 닫을 때 까지 기다림
            input_Table = int(self.inputTable.lineEdit.text())
            # print(f"btnKeypad sResult['state'] : {sResult['state']}")
            
            if sResult['state'] == 'move1':
                sResult['state'] = 'trayA'
                # getTimestamp_str()
            elif sResult['state'] == 'move2':
                sResult['state'] = 'trayB'
            else:
                return
            
            setTimestamp()
            send_data = String(f'{sResult}')
            pub.publish(send_data)
            
            self.btnTable.setVisible(False)

            # print(f"입력된 테이블: {input_Table}")
            self.close()
            self.__init__()
 
def getTimestamp():
    current_time = rospy.get_rostime()
    
    # 년-월-일 시:분:초 포맷 예시
    # timestamp_str = datetime.fromtimestamp(current_time.to_sec()).strftime('%Y-%m-%d %H:%M:%S')
    
    # ISO 8601 확장형 포맷 예시
    #timestamp_str = datetime.fromtimestamp(current_time.to_sec()).isoformat(timespec='milliseconds')
    
    # 년월일_시분초 포맷 예시
    timestamp_str = datetime.fromtimestamp(current_time.to_sec()).strftime('%Y%m%d_%H%M%S')
    #timestamp_str = datetime.fromtimestamp(current_time.to_sec()).strftime('%Y-%m-%d %H:%M:%S')
    return timestamp_str

def setTimestamp():
    sResult['time'] = getTimestamp() 
    
def setState(state):
    sResult['state'] = state() 
    
def callback(data):
    parsed_data = data.data.split(chSplit1)
    key = parsed_data[0]
    value = parsed_data[1]
    # 수신한 문자열 메시지 출력
    rospy.loginfo("Received message: %s", data.data)
    
def subscribe_to_topic():
    # 구독자(Subscriber) 생성
    rospy.Subscriber('DeliveryTable_topic', String, callback)
    
    # 루프 유지
    rospy.spin()
        
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