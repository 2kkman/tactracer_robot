from flask import Flask, Response, render_template_string, request
import cv2
import logging
from OpenSSL import SSL
from cheroot.wsgi import Server as WSGIServer
from cheroot.ssl.builtin import BuiltinSSLAdapter
import threading
import time
import socket
from UtilBLB import *

logging.basicConfig(level=logging.DEBUG)
app = Flask(__name__)
lastCamTime = getDateTime()

# 전역 설정 변수
FPS = 1000  # 프레임 레이트 설정
SHOW_TIMESTAMP = 2  # 타임스탬프 표시 위치: 0 = 사용 안 함, 1 = 좌상단, 2 = 우상단, 3 = 좌하단, 4 = 우하단

class CameraStream:
    def __init__(self, fps):
        self.lock = threading.Lock()
        self.available_resolutions = self.get_available_resolutions()
        self.best_resolution = self.get_best_resolution()
        self.camera = cv2.VideoCapture(0)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.best_resolution[0])
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.best_resolution[1])
        self.camera.set(cv2.CAP_PROP_FPS, fps)  # 카메라 FPS 설정
        self.frame = None
        self.is_running = False
        self.thread = None
        self.fps = fps
        self.frame_interval = 1.0 / fps

    def get_available_resolutions(self):
        camera = cv2.VideoCapture(0)
        resolutions = [
            (160, 120), (320, 240), (640, 480), (800, 600),
            (1024, 768), (1280, 720), (1280, 1024), (1920, 1080),
            (2560, 1440), (3840, 2160), (4000, 3000), (4096, 2160)
        ]
        available_resolutions = []
        for width, height in resolutions:
            camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            actual_width = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
            if (actual_width, actual_height) not in available_resolutions:
                available_resolutions.append((actual_width, actual_height))

        camera.set(cv2.CAP_PROP_FRAME_WIDTH, 10000)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 10000)
        max_width = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH))
        max_height = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if (max_width, max_height) not in available_resolutions:
            available_resolutions.append((max_width, max_height))

        camera.release()
        return sorted(available_resolutions, key=lambda r: r[0] * r[1], reverse=True)

    def get_best_resolution(self):
        return self.available_resolutions[0]

    def start(self):
        if not self.is_running:
            self.is_running = True
            self.thread = threading.Thread(target=self.update, args=())
            self.thread.start()

    def update(self):
        global lastCamTime
        while self.is_running:
            if not isTimeExceeded(lastCamTime, FPS):
              continue
            
            success, frame = self.camera.read()
            lastCamTime = getDateTime()
            if success:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
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

    def stop(self):
        self.is_running = False
        if self.thread:
            self.thread.join()
        self.camera.release()

    def get_current_resolution(self):
        width = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
        return (width, height)

camera_stream = CameraStream(FPS)
camera_stream.start()

@app.route('/video_feed')
def video_feed():
    def generate_frames():
        while True:
            frame = camera_stream.get_frame()
            if frame is not None:
                ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
                frame = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    best_width, best_height = camera_stream.best_resolution
    client_ip = request.remote_addr
    html_template = '''
        <html>
        <img src="/video_feed" height="100%" width="100%">
        </html>
    '''
    return render_template_string(html_template, width=best_width, height=best_height)

pathKey = '/root/key.pem'
pathCert = '/root/cert.pem'
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
    current_resolution = camera_stream.get_current_resolution()
    print(f"웹 브라우저에서 다음 주소로 접속하세요: https://{ip_address}:{port}")
    print(f"현재 FPS: {FPS}")
    print(f"타임스탬프 표시 위치: {SHOW_TIMESTAMP}")
    print(f"사용 가능한 카메라 해상도:")
    for res in camera_stream.available_resolutions:
        print(f"  {res[0]}x{res[1]}")
    print(f"선택된 최고 해상도: {camera_stream.best_resolution[0]}x{camera_stream.best_resolution[1]}")
    print(f"현재 카메라 해상도: {current_resolution[0]}x{current_resolution[1]}")
    
    server = WSGIServer((ip_address, port), app)
    server.ssl_adapter = BuiltinSSLAdapter(pathCert, pathKey)
    try:
        server.start()
    except KeyboardInterrupt:
        camera_stream.stop()
        server.stop()
