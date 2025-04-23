# import sys
# import rospy
# from std_msgs.msg import String
# from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget, QPushButton, QComboBox, QLabel
# from PyQt5.QtCore import QTimer, QThread, pyqtSignal
# import matplotlib.pyplot as plt
# from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

# sDivFieldColon = ":"
# sDivItemComma = ","
# sDivEmart = "`"
# sDivSlash = "/"
# sDivSemiCol = ";"


# class PlotCanvas(FigureCanvas):

#     def __init__(self, parent=None):
#         fig, self.ax = plt.subplots()
#         super().__init__(fig)
#         self.setParent(parent)
#         self.ax.set_title("Real-time Data")
#         self.ax.set_xlabel("Time")
#         self.ax.set_ylabel("Value")
#         self.ax.xaxis.grid(True)
#         self.ax.yaxis.grid(True)

#         self.x_data = []
#         self.y_data = []

#         (self.line,) = self.ax.plot(self.x_data, self.y_data, "b-")
#         self.ax.set_xlim(0, 100)
#         self.ax.set_ylim(0, 300)
#         self.ax.axhline(y=100, color="#FF8724", linestyle="--")
#         self.ax.axhline(y=150, color="#EB003B", linestyle="--")
        
#     def update_plot(self, x, y):
#         self.x_data.append(x)
#         self.y_data.append(y)

#         self.line.set_data(self.x_data, self.y_data)
#         self.ax.set_xlim(max(0, x - 100), x)
#         self.ax.figure.canvas.draw()


# class ROSDataReceiver(QThread):
#     data_received = pyqtSignal(dict)

#     def __init__(self):
#         super().__init__()
#         self.subscriber = None
#         self.rospy_initialized = False

#     def run(self):
#         if not self.rospy_initialized:
#             rospy.init_node("pyqt_subscriber", anonymous=True, disable_signals=True)
#             self.rospy_initialized = True

#     def start_subscription(self, topic):
#         if self.subscriber is not None:
#             self.subscriber.unregister()
#         self.subscriber = rospy.Subscriber(topic, String, self.callback, queue_size=10)

#     def stop_subscription(self):
#         if self.subscriber is not None:
#             self.subscriber.unregister()
#             self.subscriber = None

#     def callback(self, data):
#         try:
#             recvDataMap = self.getDic_strArr(data.data, sDivFieldColon, sDivItemComma)
#             self.data_received.emit(recvDataMap)
#         except AttributeError as e:
#             print(f"AttributeError: {e}")

#     def getDic_strArr(self, strTmp, spliterItemValue, spliterLine):
#         dicReturn = {}
#         file_list = strTmp.split(sep=spliterLine)
#         for i in file_list:
#             iCurrent = i.split(spliterItemValue, 1)
#             if len(iCurrent) > 1:
#                 dicReturn[iCurrent[0]] = iCurrent[1]
#         return dicReturn


# class MyApp(QMainWindow):

#     def __init__(self):
#         super().__init__()
#         self.initUI()
#         self.data_receiver = ROSDataReceiver()
#         self.data_receiver.data_received.connect(self.process_data)
#         self.data_receiver.start()
#         self.pending_data = []
#         self.timer = QTimer()
#         self.timer.timeout.connect(self.update_plot_from_queue)
#         self.timer.start(1)  # 100ms 간격으로 업데이트

#         self.x = 0

#     def initUI(self):
#         central_widget = QWidget(self)
#         self.setCentralWidget(central_widget)

#         layout = QVBoxLayout(central_widget)

#         # 상단에 버튼과 콤보박스를 4:1:1 비율로 배치
#         h_layout = QHBoxLayout()
        
#         self.start_button = QPushButton("Start", self)
#         self.stop_button = QPushButton("Stop", self)
#         self.combo_box = QComboBox(self)
#         self.combo_box.addItems([f"/MB_{i}" for i in range(1, 32)]) 
#         # self.combo_box.addItems(["/MB_10", "/MB_11", "/MB_12", "/MB_13", "/MB_30"])  # 예시 토픽들
#         # self.combo_box_ = QComboBox(self)
#         h_layout.addWidget(self.combo_box, 4)
#         h_layout.addWidget(self.start_button, 1)
#         h_layout.addWidget(self.stop_button, 1)

#         layout.addLayout(h_layout)

#         self.start_button.clicked.connect(self.start_subscription)
#         self.stop_button.clicked.connect(self.stop_subscription)
#         self.stop_button.setEnabled(False)  # 스탑 버튼 비활성화
        
#         self.plot_canvas = PlotCanvas(self)
#         layout.addWidget(self.plot_canvas)

#         self.setWindowTitle("Real-time Data Plotting with ROS")
#         self.setGeometry(300, 300, 800, 600)
#         self.show()


#     def start_subscription(self):
#         selected_topic = self.combo_box.currentText()
#         self.combo_box.setEnabled(False)  # 콤보박스 비활성화
#         self.data_receiver.start_subscription(selected_topic)
#         self.start_button.setEnabled(False)  # 시작 버튼 비활성화
#         self.stop_button.setEnabled(True)  # 스탑 버튼 활성화

#     def stop_subscription(self):
#         self.data_receiver.stop_subscription()
#         self.combo_box.setEnabled(True)  # 콤보박스 다시 활성화
#         self.start_button.setEnabled(True)  # 시작 버튼 다시 활성화
#         self.stop_button.setEnabled(False)  # 스탑 버튼 비활성화

#     def process_data(self, dicData):
#         """수신된 데이터를 큐에 저장"""
#         value = dicData.get("CUR_TORQUE")
#         if value:
#             self.pending_data.append(value)

#     def update_plot_from_queue(self):
#         """큐에 있는 데이터를 일정 간격으로 플롯에 반영"""
#         if self.pending_data:
#             value = self.pending_data.pop(0)
#             self.plot_canvas.update_plot(self.x, abs(int(value)))
#             self.x += 1


# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     ex = MyApp()
#     sys.exit(app.exec_())
import sys
import rospy
from std_msgs.msg import String
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget, QPushButton, QComboBox, QLabel
from PyQt5.QtCore import QTimer, QThread, pyqtSignal
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from collections import deque

sDivFieldColon = ":"
sDivItemComma = ","

class PlotCanvas(FigureCanvas):
    def __init__(self, parent=None):
        fig, self.ax = plt.subplots()
        super().__init__(fig)
        self.setParent(parent)
        self.ax.set_title("Real-time Data")
        self.ax.set_xlabel("Time")
        self.ax.set_ylabel("Value")
        self.ax.grid(True)

        self.x_data = deque(maxlen=1000)  # 오래된 데이터 제거
        self.y_data = deque(maxlen=1000)
        
        (self.line,) = self.ax.plot([], [], "b-")
        self.ax.set_ylim(0, 300)
        self.ax.axhline(y=100, color="#FF8724", linestyle="--")
        self.ax.axhline(y=150, color="#EB003B", linestyle="--")

    def update_plot(self, x, y):
        self.x_data.append(x)
        self.y_data.append(y)

        self.line.set_data(self.x_data, self.y_data)
        self.ax.set_xlim(max(0, x - 100), x)  # 최근 100개만 보이게
        self.draw_idle()  # blit=True 사용 가능
        

class ROSDataReceiver(QThread):
    data_received = pyqtSignal(int)

    def __init__(self):
        super().__init__()
        self.subscriber = None
        self.topic = None
        self.running = True
        self.data_queue = deque(maxlen=50)  # 큐 사용

    def run(self):
        rospy.init_node("pyqt_subscriber", anonymous=True, disable_signals=True)
        rospy.spin()  # 지속적으로 수신

    def start_subscription(self, topic):
        if self.subscriber is not None:
            self.subscriber.unregister()
        self.topic = topic
        self.subscriber = rospy.Subscriber(topic, String, self.callback, queue_size=10)

    def stop_subscription(self):
        if self.subscriber is not None:
            self.subscriber.unregister()
            self.subscriber = None

    def callback(self, data):
        try:
            dicData = self.getDic_strArr(data.data, sDivFieldColon, sDivItemComma)
            value = dicData.get("CUR_TORQUE")
            if value:
                self.data_queue.append(int(value))
                if len(self.data_queue) == 1:  # 큐가 비어있을 때만 신호
                    self.data_received.emit(int(value))
        except (AttributeError, ValueError) as e:
            print(f"Error: {e}")

    def getDic_strArr(self, strTmp, spliterItemValue, spliterLine):
        return {i.split(spliterItemValue, 1)[0]: i.split(spliterItemValue, 1)[1] for i in strTmp.split(spliterLine) if sDivFieldColon in i}


class MyApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.initUI()
        self.data_receiver = ROSDataReceiver()
        self.data_receiver.data_received.connect(self.process_data)
        self.data_receiver.start()
        
        self.x = 0
        self.data_list = []
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot_from_queue)
        self.timer.start(100)  # 50ms 업데이트

    def initUI(self):
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        
        left_layout = QVBoxLayout()
        
        h_layout = QHBoxLayout()
        self.start_button = QPushButton("Start", self)
        self.stop_button = QPushButton("Stop", self)
        self.combo_box = QComboBox(self)
        # self.combo_box.addItems(["/MB_10", "/MB_11", "/MB_12", "/MB_13", "/MB_30"])
        self.combo_box.addItems([f"/MB_{i}" for i in range(1, 32)])
        # self.combo_box_ = QComboBox(self)
        h_layout.addWidget(self.combo_box, 4)
        h_layout.addWidget(self.start_button, 1)
        h_layout.addWidget(self.stop_button, 1)

        layout.addLayout(h_layout)
        self.start_button.clicked.connect(self.start_subscription)
        self.stop_button.clicked.connect(self.stop_subscription)
        self.stop_button.setEnabled(False)

        self.plot_canvas = PlotCanvas(self)
        layout.addWidget(self.plot_canvas)
        
        layout.addLayout(left_layout, 3)
        
                # 우측 UI (토크 정보)
        right_layout = QVBoxLayout()
        self.max_speed_label = QLabel("최고 토크: 0", self)
        self.avg_speed_label = QLabel("평균 토크: 0", self)
        right_layout.addWidget(self.max_speed_label)
        right_layout.addWidget(self.avg_speed_label)
        layout.addLayout(right_layout, 1)  # 우측 UI 비율 1

        self.setWindowTitle("Real-time Data Plotting with ROS")
        self.setGeometry(300, 300, 800, 600)
        self.show()

    def start_subscription(self):
        selected_topic = self.combo_box.currentText()
        self.combo_box.setEnabled(False)
        self.data_receiver.start_subscription(selected_topic)
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)

    def stop_subscription(self):
        self.data_receiver.stop_subscription()
        self.combo_box.setEnabled(True)  # 콤보박스 다시 활성화
        self.start_button.setEnabled(True)  # 시작 버튼 다시 활성화
        self.stop_button.setEnabled(False)  # 스탑 버튼 비활성화
        
        # 그래프 초기화
        self.plot_canvas.x_data.clear()
        self.plot_canvas.y_data.clear()
        self.plot_canvas.line.set_data([], [])
        self.plot_canvas.ax.set_xlim(0, 100)
        self.plot_canvas.ax.figure.canvas.draw()
        
        # 데이터 및 토크 초기화
        self.x = 0
        self.data_receiver.data_queue.clear()
        self.data_list.clear()
        self.max_speed_label.setText("최고 토크: 0")
        self.avg_speed_label.setText("평균 토크: 0")

    def process_data(self, value):
        self.data_list.append(value)
        self.plot_canvas.update_plot(self.x, abs(value))
        self.x += 1
        
        # 최고 토크 및 평균 토크 계산 후 표시
        max_speed = max(self.data_list, default=0)
        avg_speed = sum(self.data_list) // len(self.data_list) if self.data_list else 0
        self.max_speed_label.setText(f"최고 토크: {max_speed}")
        self.avg_speed_label.setText(f"평균 토크: {avg_speed}")

    def update_plot_from_queue(self):
        if self.data_receiver.data_queue:
            value = self.data_receiver.data_queue.popleft()
            self.process_data(value)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    ex = MyApp()
    sys.exit(app.exec_())
