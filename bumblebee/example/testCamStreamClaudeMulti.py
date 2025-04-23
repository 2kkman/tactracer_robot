from flask import Flask, Response, render_template_string, request
import cv2
import logging
from OpenSSL import SSL
from cheroot.wsgi import Server as WSGIServer
from cheroot.ssl.builtin import BuiltinSSLAdapter
import threading

logging.basicConfig(level=logging.DEBUG)
app = Flask(__name__)

class CameraStream:
    def __init__(self):
        self.lock = threading.Lock()
        self.best_resolution = self.get_best_resolution()
        self.camera = cv2.VideoCapture(0)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.best_resolution[0])
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.best_resolution[1])
        self.frame = None
        self.is_running = False
        self.thread = None

    def get_best_resolution(self):
        camera = cv2.VideoCapture(0)
        resolutions = [
            #(1920, 1080), (1280, 720), (640, 480), (320, 240), (160, 120)
            (1280, 720), (640, 480), (320, 240), (160, 120)
        ]
        best_resolution = (640, 480)  # 기본 해상도
        for width, height in resolutions:
            camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            if (camera.get(cv2.CAP_PROP_FRAME_WIDTH) == width and
                    camera.get(cv2.CAP_PROP_FRAME_HEIGHT) == height):
                best_resolution = (width, height)
                break
        camera.release()
        return best_resolution

    def start(self):
        if not self.is_running:
            self.is_running = True
            self.thread = threading.Thread(target=self.update, args=())
            self.thread.start()

    def update(self):
        while self.is_running:
            success, frame = self.camera.read()
            if success:
                with self.lock:
                    self.frame = cv2.rotate(frame, cv2.ROTATE_180)

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

camera_stream = CameraStream()
camera_stream.start()

@app.route('/video_feed')
def video_feed():
    def generate_frames():
        while True:
            frame = camera_stream.get_frame()
            if frame is not None:
                ret, buffer = cv2.imencode('.jpg', frame)
                frame = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame',
                    headers={'Cache-Control': 'no-cache, no-store, must-revalidate'})

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

if __name__ == '__main__':
    server = WSGIServer(('0.0.0.0', 5000), app)
    server.ssl_adapter = BuiltinSSLAdapter(pathCert, pathKey)
    try:
        server.start()
    except KeyboardInterrupt:
        camera_stream.stop()
        server.stop()