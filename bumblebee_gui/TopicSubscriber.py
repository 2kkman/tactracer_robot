import sys
import rospy
from std_msgs.msg import String
from PyQt5.QtWidgets import QApplication, QDialog, QLabel, QVBoxLayout

class TopicSubscriber(QDialog):
    def __init__(self, topic_name):
        super().__init__()
        self.setWindowTitle("ROS Topic Subscriber")
        self.label = QLabel()
        self.label.setText("Waiting for messages...")
        self.layout = QVBoxLayout()
        
        self.layout.addWidget(self.label)
        self.setLayout(self.layout)

        self.topic_name = topic_name
        rospy.Subscriber(self.topic_name, String, self.messageCallback)

    def messageCallback(self, msg):
        received_message = msg.data
        self.label.setText(received_message)

def main():
    rospy.init_node('topic_subscriber')
    app = QApplication(sys.argv)
    topic_name = "BLB_STATUS"  # 변경 가능한 토픽 이름
    subscriber = TopicSubscriber(topic_name)
    subscriber.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()