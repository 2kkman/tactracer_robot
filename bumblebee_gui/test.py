import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QLabel, QSpinBox, QVBoxLayout, QHBoxLayout
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import QDate, Qt

import rospy
from std_msgs.msg import String


class MyWidget(QMainWindow):
    def __init__(self):
        super().__init__()
        self.date = QDate.currentDate()
        # self.setWindowFlags(Qt.WindowTitleHint | Qt.WindowCloseButtonHint)
        self.initUI()

    def initUI(self):
        btn = QPushButton("Send", self)
        btn.resize(btn.sizeHint())
        btn.move(300, 50)
        btn.clicked.connect(self.send_message)
        
        self.lbl1 = QLabel('QSpinBox')
        self.spinbox1 = QSpinBox()
        self.spinbox1.setMinimum()
        self.spinbox1.setMaximum(10)
        # self.spinbox.setRange(-10, 30)
        self.spinbox1.setSingleStep(2)
        self.lbl2 = QLabel('0')
        self.spinbox1.valueChanged.connect(self.value_changed)
        
        vbox = QVBoxLayout()
        vbox.addWidget(self.lbl1)
        vbox.addWidget(self.spinbox1)
        vbox.addWidget(self.lbl2)
        vbox.addStretch()

        self.setLayout(vbox)        
        
        btn2 = QPushButton("Send2", self)
        btn2.resize(btn.sizeHint())
        btn2.move(300, 150)
        btn2.clicked.connect(self.send_message2)
        
        btn3 = QPushButton("Send3", self)
        btn3.resize(btn.sizeHint())
        btn3.move(300, 250)
        btn3.clicked.connect(self.send_message3)
        
        self.setWindowIcon(QIcon('spidergo.ico'))
        self.setWindowTitle("Bumblebee ROS PyQt Example") 
        self.setGeometry(400, 400, 400, 150)
        
        stautsdate = QLabel(self.date.toString(Qt.DefaultLocaleLongDate), self)
        self.statusBar().addPermanentWidget(stautsdate)
        
    def value_changed(self):
        self.lbl2.setText(str(self.spinbox1.value()))
        
    def send_message(self):
        pub = rospy.Publisher('my_topic', String, queue_size=10)
        pub.publish('Hello ROS!')
    def send_message2(self):
        pub = rospy.Publisher('my_topic', String, queue_size=10)
        pub.publish('Hello ROS2!')
    def send_message3(self):
        pub = rospy.Publisher('my_topic', String, queue_size=10)
        pub.publish('Hello ROS3!')
        
    
        
if __name__ == '__main__':
    rospy.init_node('my_node')
    app = QApplication(sys.argv)
    widget = MyWidget()
    # widget.showMaximized()
    widget.showFullScreen()
    
    sys.exit(app.exec_())