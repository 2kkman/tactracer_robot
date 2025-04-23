import os
import sys

from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtGui import *
from PyQt5.QtCore import *

import rospy
from std_msgs.msg import String
from importlib import reload
from ui.KeypadDialog import KeypadDialog

def resource_path(relative_path):
    base_path = getattr(sys, "_MEIPASS", os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(base_path, relative_path)

form = resource_path('mainwindow.ui')
form_class = uic.loadUiType(form)[0]

form_second = resource_path('moving.ui')
form_secondwindow = uic.loadUiType(form_second)[0]

msgfont = QFont('Noto Sans Mono CJK KR', 16)

class WindowClass(QMainWindow, form_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.counter = 0

        self.count = 0
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.resetCount)
        self.timer.setSingleShot(True)
        self.subscriber = rospy.Subscriber('DeliveryTable_topic', String, self.callback)

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

    def evt_send_message(self):
        global table_no
        global send_data
        global pub

        table_no = {'ID':'BumbleBee1','tray1':self.spinbox1.text(),'tray2':self.spinbox2.text(),'state':'move1'}
        # for key, value in table_no.items():
        #     print(f'{key}층 : {value}번 테이블')
        DeliveryTable = QMessageBox()
        DeliveryTable.setFixedSize(500, 500)
        # DeliveryTable.setFont(msgfont)
        DeliveryTable.setText(f"1층 {self.spinbox1.text()}번 Table\n2층 {self.spinbox2.text()}번 Table\n배달 확인")
        DeliveryTable.setWindowTitle("테이블 배달")
        DeliveryTable.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
        button_list = DeliveryTable.findChildren(QPushButton)
        for button in button_list:
            button.setFixedSize(100, 50)
            # button.setFixedWidth(100)
            # button.setFixedHeight(50)
        DeliveryTable.setStyleSheet("QMessageBox{background-color: #FDDE45; font-size: 24px;} QPushButton {font-size: 20px;}")
        bOK = DeliveryTable.exec_()

        pub = rospy.Publisher('DeliveryTable_topic', String, queue_size=30)

        if bOK == QMessageBox.Ok:
            send_data = String(f'DeliveryTable:{table_no}')
            self.spinbox1.setValue(0)
            self.spinbox2.setValue(0)
            pub.publish(send_data)
            self.btn_main_to_movewindow()
            # self.spinbox3.setValue(0)
        else :
            send_data = String('no')
            
    def callback(self, msg):
        global received_message
        received_message = msg.data
        # print(received_message)
        
class secondwindow(QDialog,QWidget,form_secondwindow):
    def __init__(self, parent=None):
        super(secondwindow,self).__init__(parent)
        global send_data
        global table_Text
        global table_no
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
        
        # self.timer.start(5000)
        self.btnTable.clicked.connect(self.btnKeypad)
        self.btnTable.setVisible(True)
        
        if table_no['tray1'] == table_no['tray2'] and table_no['state'] == 'move1':
            table_no['state'] = 'move3'
            send_data = String(f'DeliveryTable:{table_no}')
            print(f'table_no: {table_no}')
            
        # print(f"state : {table_no['state']}")    
        if table_no['state'] == 'move1':
            table_Text = f"테이블{table_no['tray1']} tray1 이동중 ..."
        elif table_no['state'] == 'move2':
            table_Text = f"테이블{table_no['tray2']} tray2 이동중 ..."
        elif table_no['state'] == 'move3':
            table_Text = f"테이블{table_no['tray2']} tray1, 2 이동중 ..."
            
        print(f"table_Text : {table_Text}")       
        
        if table_no['state'] == 'tray1':      
            if int(table_no['tray1']) == input_Table:
                self.btnTable.setVisible(True)
                # table_Text+=(f"Get to the Table\nThe food you ordered is out\n on the Tray 1")
                table_Text+=("트레이 1번에\n주문하신 메뉴가 나왔습니다\n메뉴를 확인해주세요")
                table_no['state'] = 'move2'
                send_data = String(f'DeliveryTable:{table_no}')
                # self.__init__()
            else:
                pass

        elif table_no['state'] == 'tray2':
            print(f"table_no['tray2'] : {table_no['tray2']}")
            self.timer.stop()
            self.timer.timeout.connect(self.close)
            
            if int(table_no['tray2']) == input_Table:
                
                self.btnTable.setVisible(False)
                
                # table_Text+=(f"Get to the Table\nThe foo d you ordered is out\n on the Tray 2")
                table_Text+=("트레이 2번에\n주문하신 메뉴가 나왔습니다\n메뉴를 확인해주세요")
                table_no['state'] = 'complete'
                send_data = String(f'DeliveryTable:{table_no}')
                self.timer.start(5000)
            else:
                pass

        elif table_no['state'] == 'tray3':
            if int(table_no['tray2']) == input_Table:
                self.btnTable.setVisible(False)
                
                # table_Text+=(f"Get to the Table\nThe foo d you ordered is out\n on the Tray 2")
                table_Text+=("트레이 1,2번에\n주문하신 메뉴가 나왔습니다\n메뉴를 확인해주세요")
                table_no['state'] = 'complete'
                send_data = String(f'DeliveryTable:{table_no}')
                self.timer.start(5000)
            else:
                pass        
        pub.publish(send_data)
        self.table_label.setText(f"{table_Text}")
        
    def initUi(self):
        self.setupUi(self)

    def btn_return(self):
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
            print(f"btnKeypad table_no['state'] : {table_no['state']}")
            
            if table_no['state'] == 'move1':
                table_no['state'] = 'tray1'
            elif table_no['state'] == 'move2':
                table_no['state'] = 'tray2'
            elif table_no['state'] == 'move3':
                table_no['state'] = 'tray3'
            else:
                table_no['state'] = 'complete'
                self.close()
                
            # print(table_no['state'])
            
            send_data = String(f'DeliveryTable:{table_no}')
            print(f'send_data: {send_data}')
            self.btnTable.setVisible(False)

            print(f"입력된 테이블: {input_Table}")
            pub.publish(send_data)
            self.close()
            self.__init__()
            

if __name__ == '__main__':
    rospy.init_node('my_node',anonymous=True)
    app = QApplication(sys.argv)
    mainWindow = WindowClass()
    mainWindow.showFullScreen()
    app.exec_()