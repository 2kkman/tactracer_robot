from flask import Flask, Response
import time

app = Flask(__name__)

def generate_messages():
    """Generator to simulate real-time messages."""
    while True:
        # 메시지 생성
        message = f"Message at {time.strftime('%Y-%m-%d %H:%M:%S')}"
        yield f"data: {message}\n\n"
        time.sleep(1)  # 1초마다 새로운 메시지 전송

@app.route('/stream')
def stream():
    """Route to stream messages."""
    return Response(generate_messages(), content_type='text/event-stream')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True, threaded=True)

