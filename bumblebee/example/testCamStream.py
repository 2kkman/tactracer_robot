from flask import Flask, request, jsonify, Response, render_template_string
import cv2
from flask_cors import CORS
import ssl

app = Flask(__name__)

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

@app.route('/video_feed')
def video_feed():
    def generate_frames():
        best_width, best_height = get_best_resolution()
        camera = cv2.VideoCapture(0)
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, best_width)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, best_height)
        
        while True:
            success, frame = camera.read()
            if not success:
                break
            else:
                # 프레임을 180도 회전
                frame = cv2.rotate(frame, cv2.ROTATE_180)
                ret, buffer = cv2.imencode('.jpg', frame)
                frame = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    best_width, best_height = get_best_resolution()
    html_template = '''
        <html>
            <head>
                <title>Webcam Video Stream</title>
            </head>
            <body>
                <h1>Webcam Video Stream</h1>
                <p>Resolution: {{ width }}x{{ height }}</p>
                <img src="/video_feed" width="640" height="480">
            </body>
        </html>
    '''
    return render_template_string(html_template, width=best_width, height=best_height)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
