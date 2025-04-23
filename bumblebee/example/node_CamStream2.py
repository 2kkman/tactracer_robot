#!/usr/bin/env python3
from sensor_msgs.msg import CompressedImage
from flask import Flask, Response, render_template_string
import cv2
import logging
from OpenSSL import SSL
from cheroot.wsgi import Server as WSGIServer
from cheroot.ssl.builtin import BuiltinSSLAdapter
import threading
import time
import socket
import datetime
import rospy
import re
from cv_bridge import CvBridge, CvBridgeError
from UtilBLB import *

logging.basicConfig(level=logging.DEBUG)

# 전역 설정 변수
FPS = 1  # 프레임 레이트 설정
SHOW_TIMESTAMP = 2  # 타임스탬프 표시 위치: 0 = 사용 안 함, 1 = 좌상단, 2 = 우상단, 3 = 좌하단, 4 = 우하단
BASE_PORT = 5000  # 기본 포트 번호

class CameraStream:
    def __init__(self, fps, topic):
        self.lock = threading.Lock()
        self.bridge = CvBridge()
        self.frame = None
        self.is_running = False
        self.fps = fps
        self.frame_interval = 1.0 / fps
        self.topic = topic
        self.last_cam_time = getDateTime()

        # ROS Subscriber 초기화
        self.image_sub = rospy.Subscriber(topic, CompressedImage, self.image_callback)

    def image_callback(self, msg):
        if not isTimeExceeded(self.last_cam_time, self.fps):
            return

        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)            
        except CvBridgeError as e:
            rospy.logerr(f"imdecode Error for {self.topic}: {e}")
            return

        self.last_cam_time = getDateTime()

        if SHOW_TIMESTAMP != 0:
            timestamp = getDateTime().strftime("%Y-%m-%d %H:%M:%S")
            position = self.get_timestamp_position(frame)
            cv2.putText(frame, timestamp, position, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        with self.lock:
            self.frame = frame

    def get_timestamp_position(self, frame):
        height, width, _ = frame.shape
        padding = 10
        positions = {
            1: (padding, padding + 30),
            2: (width - 360 - padding, padding + 30),
            3: (padding, height - padding),
            4: (width - 360 - padding, height - padding)
        }
        return positions.get(SHOW_TIMESTAMP, (padding, padding + 30))

    def get_frame(self):
        with self.lock:
            if self.frame is not None:
                return self.frame
            return None

    def start(self):
        self.is_running = True

    def stop(self):
        self.is_running = False

def create_app(camera_stream):
    app = Flask(__name__)

    @app.route('/video_feed')
    def video_feed():
        def generate_frames():
            while True:
                time.sleep(1.0 / FPS)
                frame = camera_stream.get_frame()
                if frame is not None:
                    ret, buffer = cv2.imencode('.jpg', frame, None)
                    frame = buffer.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

    @app.route('/')
    def index():
        html_template = '''
            <html>
            <img src="/video_feed" height="100%" width="100%">
            </html>
        '''
        return render_template_string(html_template)

    return app

def get_publish_topics():
    topics = rospy.get_published_topics()
    return [topic for topic, type in topics if topic.startswith('/QBI/IMG_PUBLISH') and type == 'sensor_msgs/CompressedImage']

def get_ip_address():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        ip_address = s.getsockname()[0]
    except Exception:
        ip_address = "127.0.0.1"
    finally:
        s.close()
    return ip_address

def run_server(app, port):
    server = WSGIServer((ip_address, port), app)
    server.ssl_adapter = BuiltinSSLAdapter(pathCert, pathKey)
    try:
        server.start()
    except KeyboardInterrupt:
        server.stop()

def get_topic_number(topic):
    match = re.search(r'/QBI/IMG_PUBLISH(\d+)', topic)
    if match:
        return int(match.group(1))
    return None

if __name__ == '__main__':
    # ROS 노드 초기화
    rospy.init_node('image_subscriber', anonymous=True)

    # SSL 인증서 경로 설정
    pathKey = f'{getConfigPath(UbuntuEnv.ITX.name)}/key.pem'
    pathCert = f'{getConfigPath(UbuntuEnv.ITX.name)}/cert.pem'
    print(f"SSL Key: {pathKey}")
    print(f"SSL Cert: {pathCert}")

    # IP 주소 가져오기
    ip_address = get_ip_address()

    time.sleep(20)
    # 토픽 찾기
    topics = get_publish_topics()

    if not topics:
        print("No /QBI/IMG_PUBLISH topics found. Exiting.")
        exit(1)

    print(f"Found {len(topics)} camera topics:")
    for topic in topics:
        print(f"  - {topic}")

    # CameraStream 인스턴스와 Flask 앱 생성
    camera_streams = []
    apps = []
    threads = []

    for topic in topics:
        camera_stream = CameraStream(FPS, topic)
        camera_stream.start()
        camera_streams.append(camera_stream)

        app = create_app(camera_stream)
        apps.append(app)

        topic_number = get_topic_number(topic)
        if topic_number is not None:
            port = BASE_PORT + topic_number
            print(f"Stream for {topic}: https://{ip_address}:{port}")

            thread = threading.Thread(target=run_server, args=(app, port))
            threads.append(thread)
            thread.start()
        else:
            print(f"Warning: Could not determine port number for topic {topic}")

    print(f"현재 FPS: {FPS}")
    print(f"타임스탬프 표시 위치: {SHOW_TIMESTAMP}")

    # 메인 스레드에서 KeyboardInterrupt를 처리
    try:
        for thread in threads:
            thread.join()
    except KeyboardInterrupt:
        print("서버를 종료합니다.")
        for camera_stream in camera_streams:
            camera_stream.stop()