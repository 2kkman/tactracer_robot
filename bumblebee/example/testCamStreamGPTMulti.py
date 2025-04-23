from flask import Flask, render_template_string, request
from flask_socketio import SocketIO, emit
import cv2
import logging
import eventlet
import eventlet.wsgi
from OpenSSL import SSL
import base64

logging.basicConfig(level=logging.DEBUG)
app = Flask(__name__)
socketio = SocketIO(app)

def get_best_resolution():
    camera = cv2.VideoCapture(0)
    resolutions = [
        (1920, 1080), (1280, 720), (640, 480), (320, 240), (160, 120)
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

@socketio.on('connect')
def handle_connect():
    logging.info(f'Client {request.sid} connected')

@socketio.on('disconnect')
def handle_disconnect():
    logging.info(f'Client {request.sid} disconnected')

def generate_frames():
    best_width, best_height = get_best_resolution()
    camera = cv2.VideoCapture(0)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, best_width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, best_height)
    if not camera.isOpened():
        msglog = 'Failed to open CAM'
        logging.error(msglog)
        return
    
    while True:
        success, frame = camera.read()
        if not success:
            break
        else:
            # 프레임을 180도 회전
            frame = cv2.rotate(frame, cv2.ROTATE_180)
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = base64.b64encode(buffer).decode('utf-8')
            socketio.emit('video_frame', {'frame': frame})
        eventlet.sleep(0)

@app.route('/')
def index():
    html_template = '''
    <html>
        <head>
            <title>Webcam Video Stream</title>
        </head>
        <body>
            <h1>Webcam Video Stream</h1>
            <img id="video" src="">
            <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.0/socket.io.js"></script>
            <script type="text/javascript" charset="utf-8">
                var socket = io();
                socket.on('video_frame', function(data) {
                    var img = document.getElementById('video');
                    img.src = 'data:image/jpeg;base64,' + data.frame;
                });
            </script>
        </body>
    </html>
    '''
    return render_template_string(html_template)

pathKey = '/root/key.pem'
pathCert = '/root/cert.pem'

if __name__ == '__main__':
    eventlet.monkey_patch()
    eventlet.spawn(generate_frames)
    socketio.run(app, host='0.0.0.0', port=5000, keyfile=pathKey, certfile=pathCert)
