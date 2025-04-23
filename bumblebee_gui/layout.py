import sys
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QIcon, QPixmap, QFont
from PyQt5.QtCore import QDate, Qt, QTimer
import rospy
from std_msgs.msg import String

class BumblebeeGui(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Bumblebee GUI")
        self.initUI()

    def initUI(self):
        
        spinfont = QFont('Arial', 115)
        btnfont = QFont('Arial', 75)
        
        label = QLabel()
        qPixmapVar = QPixmap()
        qPixmapVar.load('Tray.jpg')
        label.setPixmap(qPixmapVar)
        label.setFixedHeight(qPixmapVar.height())

        sendBtn = QPushButton("GO", self)
        sendBtn.resize(sendBtn.sizeHint())
        sendBtn.clicked.connect(self.evt_send_message)
        # sendBtn.setStyleSheet("QPushButton { width :200px; heirght : 200px; }")
        sendBtn.setStyleSheet("QPushButton {"     
                              "background-color: #FDDE45;" 
                              "border-style: outset; "
                              "border-width: 2px; "
                              "border-radius: 10px;"
                              "border-color: beige;"
                              "min-width: 10em;"
                              "padding: 6px; }")
        
        sendBtn.setFont(btnfont)

        allCheck = QCheckBox('ALL ', self)
        allCheck.move(100, 100)
        allCheck.resize(200, 50)
        allCheck.setStyleSheet("QCheckBox::indicator { width :40px; height : 40px;}")
        allCheck.toggle()
        allCheckFont = allCheck.font()
        allCheckFont.setPointSize(30)
        allCheck.setFont(allCheckFont)
        allCheck.stateChanged.connect(self.evt_changeTitle)
        allCheck.setHidden(True)

        lbl1 = QLabel('1층')
        lbl1.setStyleSheet('background-color: #FDDE45; font-size: 24px; padding: 10px; width: 100px;')
        self.spinbox1 = QSpinBox()
        # 화살표 크기 지정
        self.spinbox1.setStyleSheet("QSpinBox::up-button { width: 100px;} QSpinBox::down-button { width: 100px;}")
        self.spinbox1.setFixedWidth(400)
        self.spinbox1.setRange(0, 10)
        self.spinbox1.textFromValue
        self.spinbox1.setFont(spinfont)
        self.spinbox1.setSingleStep(1)
        self.spinbox1.setWrapping(True)
        self.spinbox1.setAlignment(Qt.AlignCenter)

        lbl2 = QLabel('2층')
        lbl2.setStyleSheet('background-color: #FDDE45; font-size: 24px; padding: 10px; width: 100px;')

        self.spinbox2 = QSpinBox()
        # 화살표 크기 지정
        self.spinbox2.setStyleSheet("QSpinBox::up-button { width: 100px;} QSpinBox::down-button { width: 100px;}")
        self.spinbox2.setFixedWidth(400)
        self.spinbox2.setRange(0, 10)
        self.spinbox2.setFont(spinfont)
        self.spinbox2.setSingleStep(1)
        self.spinbox2.setWrapping(True)
        self.spinbox2.setAlignment(Qt.AlignCenter)

        # lbl3 = QLabel('3층')
        # lbl3.setStyleSheet('background-color: yellow; font-size: 20px; padding: 10px; width: 100px;')

        # self.spinbox3 = QSpinBox()
        # # 화살표 크기 지정
        # self.spinbox3.setStyleSheet("QSpinBox::up-button { width: 100px;} QSpinBox::down-button { width: 100px;}")
        # self.spinbox3.setFixedWidth(300)
        # self.spinbox3.setMinimum(0)
        # self.spinbox3.setMaximum(10)
        # font = self.spinbox3.font()
        # font.setPointSize(75)
        # self.spinbox3.setFont(font)
        # self.spinbox3.setSingleStep(1)
        # self.spinbox3.setWrapping(True)
        # self.spinbox3.setAlignment(Qt.AlignCenter)

        leftInnerLayOut = QHBoxLayout()
        leftInnerLayOut.addStretch()
        leftInnerLayOut.addWidget(lbl1)
        leftInnerLayOut.addWidget(self.spinbox1)
        leftInnerLayOut.addStretch()

        leftInnerLayOut2 = QHBoxLayout()
        leftInnerLayOut2.addStretch()
        leftInnerLayOut2.addWidget(lbl2)
        leftInnerLayOut2.addWidget(self.spinbox2)
        leftInnerLayOut2.addStretch()


        # hbox3 = QHBoxLayout()
        # hbox3.addStretch()
        # hbox3.addWidget(lbl3)
        # hbox3.addWidget(self.spinbox3)
        # hbox3.addWidget(sendBtn)
        # hbox3.addStretch(5)

        leftLayOut = QVBoxLayout()
        leftLayOut.addStretch()
        leftLayOut.addLayout(leftInnerLayOut2)
        leftLayOut.addLayout(leftInnerLayOut)
        leftLayOut.addStretch()

        centerLayOut = QVBoxLayout()
        centerLayOut.addWidget(label)
        label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        label.setAlignment(Qt.AlignCenter)
        
        rightInnerLayOut = QVBoxLayout()
        rightInnerLayOut.addWidget(sendBtn)
        sendBtn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        

        allbox = QHBoxLayout()
        allbox.addLayout(leftLayOut)
        allbox.addLayout(centerLayOut)
        allbox.addLayout(rightInnerLayOut)

        self.setLayout(allbox)

        self.setWindowIcon(QIcon('spidergo.ico'))
        self.setWindowTitle("Layout Example")
        self.setGeometry(50, 50, 400, 300)

    # def evt_changedValue(self):
    #     self.lbl2.setText(str(self.spinbox1.value()))

    def evt_send_message(self):
        # table_no = {1:self.spinbox1.text(),2:self.spinbox2.text(),3:self.spinbox3.text()}
        table_no = {1:self.spinbox1.text(),2:self.spinbox2.text()}
        # for key, value in table_no.items():
        #     print(f'{key}층 : {value}번 테이블')
        msgDeliveryTable = QMessageBox()
        msgDeliveryTable.setText(f"1층 {self.spinbox1.text()}번 테이블\n2층 {self.spinbox2.text()}번 테이블\n배달하시겠습니까?")
        msgDeliveryTable.setWindowTitle("테이블 배달")
        msgDeliveryTable.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)

        bOK = msgDeliveryTable.exec_()
        pub = rospy.Publisher('DeliveryTable_topic', String, queue_size=10)

        if bOK == QMessageBox.Ok:
            send_data = String(f'msgDeliveryTable:{table_no}')
            self.spinbox1.setValue(0)
            self.spinbox2.setValue(0)
            # self.spinbox3.setValue(0)
        else :
            send_data = String('no')

        pub.publish(send_data)

    def evt_changeTitle(self, state):
        if state == Qt.Checked:
            self.setWindowTitle('QCheckBox')
        else:
            self.setWindowTitle(' ')

        
def main():
    rospy.init_node('my_node',anonymous=True)
    app = QApplication(sys.argv)
    widget = BumblebeeGui()
    # widget.raise_()
    widget.showFullScreen()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
