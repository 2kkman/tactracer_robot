import sys
from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QPushButton, QMainWindow
import rospy
from std_msgs.msg import String  # 토픽 메시지의 데이터 타입에 맞춰 임포트합니다.

class ReadyDialog(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Ready Dialog")
        layout = QVBoxLayout()
        label = QLabel("The robot is ready!")
        layout.addWidget(label)
        self.button_box = QPushButton("Ok")
        self.button_box.clicked.connect(self.accept)
        layout.addWidget(self.button_box)
        self.central_widget = QMainWindow()
        self.central_widget.setLayout(layout)
        self.setCentralWidget(self.central_widget)

class NotReadyDialog(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Not Ready Dialog")
        layout = QVBoxLayout()
        label = QLabel("The robot is not ready!")
        layout.addWidget(label)
        self.button_box = QPushButton("Ok")
        self.button_box.clicked.connect(self.accept)
        layout.addWidget(self.button_box)
        self.central_widget = QMainWindow()
        self.central_widget.setLayout(layout)
        self.setCentralWidget(self.central_widget)

class MyGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS Topic GUI")
        self.showMaximized()  # 윈도우를 최대화합니다.

        layout = QVBoxLayout()

        self.label = QLabel("Waiting for ROS topic data...")
        layout.addWidget(self.label)

        self.ready_button = QPushButton("Show Ready Dialog")
        self.ready_button.clicked.connect(self.show_ready_dialog)
        layout.addWidget(self.ready_button)

        self.not_ready_button = QPushButton("Show Not Ready Dialog")
        self.not_ready_button.clicked.connect(self.show_not_ready_dialog)
        layout.addWidget(self.not_ready_button)

        central_widget = QMainWindow()
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

        # ROS 토픽 구독 설정
        rospy.init_node('ros_topic_subscriber')  # 노드 초기화
        rospy.Subscriber('/your_topic_name', String, self.callback)  # 토픽 이름과 메시지 데이터 타입 설정

    def callback(self, msg):
        # ROS 토픽의 값을 받아와서 라벨 업데이트
        self.label.setText(msg.data)

    def show_ready_dialog(self):
        dialog = ReadyDialog()
        dialog.exec_()

    def show_not_ready_dialog(self):
        dialog = NotReadyDialog()
        dialog.exec_()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = MyGUI()
    gui.show()
    sys.exit(app.exec_())
