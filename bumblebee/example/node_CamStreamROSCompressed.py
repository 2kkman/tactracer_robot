#!/usr/bin/env python3
from sensor_msgs.msg import CompressedImage
from flask import Flask, Response, render_template_string, request
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
#from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from UtilBLB import *

logging.basicConfig(level=logging.DEBUG)
app = Flask(__name__)
lastCamTime = getDateTime()

# 전역 설정 변수
FPS = 10  # 프레임 레이트 설정
SHOW_TIMESTAMP = 2  # 타임스탬프 표시 위치: 0 = 사용 안 함, 1 = 좌상단, 2 = 우상단, 3 = 좌하단, 4 = 우하단

class CameraStream:
    def __init__(self, fps):
        self.lock = threading.Lock()
        self.bridge = CvBridge()
        self.frame = None
        self.is_running = False
        self.thread = None
        self.fps = fps
        self.frame_interval = 1.0 / fps

        # ROS 노드 초기화
        rospy.init_node('image_subscriber2', anonymous=True)
        self.image_sub = rospy.Subscriber("/QBI/IMG_PUBLISH0", CompressedImage, self.image_callback)

    def image_callback(self, msg):
        # global lastCamTime
        # if not isTimeExceeded(lastCamTime, FPS):
        #     return

        try:
            #frame = self.bridge.imgmsg_to_cv2(data, "rgb8")
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)            
        except CvBridgeError as e:
            rospy.logerr("imdecode Error: {0}".format(e))
            return

        lastCamTime = getDateTime()

        #frame = cv2.rotate(frame, cv2.ROTATE_180)
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
            1: (padding, padding + 30),                 # 좌상단
            2: (width - 360 - padding, padding + 30),   # 우상단
            3: (padding, height - padding),             # 좌하단
            4: (width - 360 - padding, height - padding) # 우하단
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

camera_stream = CameraStream(FPS)
camera_stream.start()

@app.route('/video_feed')
def video_feed():
    def generate_frames():
        while True:
            time.sleep(1)  # 0.5초마다 프레임을 전송하여 초당 2프레임으로 제한
            frame = camera_stream.get_frame()
            if frame is not None:
                #ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
                ret, buffer = cv2.imencode('.jpg', frame, None)
                frame = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    client_ip = request.remote_addr
    html_template = '''
        <html>
        <img src="/video_feed" height="100%" width="100%">
        </html>
    '''
    return render_template_string(html_template)

pathKey = '/root/key.pem'
pathCert = '/root/cert.pem'
pathKey = f'{getConfigPath(UbuntuEnv.ITX.name)}/key.pem'
pathCert = f'{getConfigPath(UbuntuEnv.ITX.name)}/cert.pem'
print(pathKey,pathCert)
context = SSL.Context(SSL.TLSv1_2_METHOD)
context.use_privatekey_file(pathKey)
context.use_certificate_file(pathCert)

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

if __name__ == '__main__':
    ip_address = get_ip_address()
    port = 5000
    print(f"웹 브라우저에서 다음 주소로 접속하세요: https://{ip_address}:{port}")
    print(f"현재 FPS: {FPS}")
    print(f"타임스탬프 표시 위치: {SHOW_TIMESTAMP}")
    
    server = WSGIServer((ip_address, port), app)
    server.ssl_adapter = BuiltinSSLAdapter(pathCert, pathKey)
    try:
        server.start()
    except KeyboardInterrupt:
        camera_stream.stop()
        server.stop()
