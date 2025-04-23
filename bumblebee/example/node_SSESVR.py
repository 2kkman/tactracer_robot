from flask import Flask, request, render_template_string
from flask_socketio import SocketIO
import rospy
import threading
import time
from std_msgs.msg import String
from UtilBLB import *

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')
shared_data = {}

# ROS 노드 초기화
rospy.init_node('imu_data_receiver', anonymous=True)
tag_publisher = rospy.Publisher(TopicName.ARUCO_RESULT.name, String, queue_size=10)
and_publisher = rospy.Publisher(TopicName.ANDROID.name, String, queue_size=10)
alarm_publisher = rospy.Publisher(TopicName.HISTORY_ALARM.name, String, queue_size=10)

@app.route('/favicon.ico')
def favicon():
    return '', 204

@app.route('/log', methods=['GET'])
def index():
    return render_template_string("""
    <!DOCTYPE html>
    <html lang="ko">
    <head>
        <meta charset="UTF-8">
        <title>실시간 데이터</title>
        <link rel="icon" href="data:,">
        <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.1/socket.io.js"></script>
        <style>
            .data-item {
                margin: 5px 0;
                padding: 5px;
                border-bottom: 1px solid #eee;
            }
            .timestamp {
                color: #666;
                font-size: 0.8em;
            }
        </style>
    </head>
    <body>
        <h1>실시간 데이터 업데이트</h1>
        <div id="connection-status">연결 상태: 대기중...</div>
        <div id="data"></div>
        <script>
            let socket;
            
            function connectWebSocket() {
                socket = io("ws://" + location.hostname + ":6002", {
                    transports: ["websocket"],
                    reconnection: true,
                    reconnectionDelay: 1000,
                    reconnectionDelayMax: 5000,
                    reconnectionAttempts: Infinity
                });

                socket.on("connect", () => {
                    console.log("WebSocket 연결됨");
                    document.getElementById("connection-status").innerHTML = 
                        '<span style="color: green;">연결 상태: 연결됨</span>';
                });

                socket.on("disconnect", () => {
                    console.log("WebSocket 연결 끊김");
                    document.getElementById("connection-status").innerHTML = 
                        '<span style="color: red;">연결 상태: 연결 끊김</span>';
                });

                socket.on("update", (data) => {
                    console.log("데이터 수신:", data);
                    updateDisplay(data);
                });

                socket.on("connect_error", (error) => {
                    console.error("연결 오류:", error);
                    document.getElementById("connection-status").innerHTML = 
                        '<span style="color: red;">연결 상태: 오류 발생</span>';
                });
            }

            function updateDisplay(data) {
                let displayText = "<div class='data-container'>";
                for (let key in data) {
                    displayText += `
                        <div class='data-item'>
                            <strong>${key}:</strong> ${data[key]}
                            <div class='timestamp'>${new Date().toLocaleTimeString()}</div>
                        </div>`;
                }
                displayText += "</div>";
                document.getElementById("data").innerHTML = displayText;
            }

            // 페이지 로드 시 WebSocket 연결
            document.addEventListener('DOMContentLoaded', connectWebSocket);

            // 연결이 끊어졌을 때 재연결 시도
            window.addEventListener('online', connectWebSocket);
        </script>
    </body>
    </html>
    """)

def update_data():
    """주기적으로 데이터를 업데이트하고 클라이언트에 전송"""
    while True:
        try:
            if shared_data:  # 데이터가 있을 때만 전송
                #print("Emitting update:", shared_data)
                socketio.emit("update", shared_data, namespace='/')
        except Exception as e:
            print(f"Error in update_data: {e}")
        time.sleep(1)  # 더 빠른 업데이트를 위해 간격 축소

@socketio.on("connect")
def handle_connect():
    print("✅ 클라이언트가 WebSocket에 연결됨")
    if shared_data:  # 연결 즉시 현재 데이터 전송
        socketio.emit("update", shared_data)

@app.route('/<path:path>', methods=['POST'])
def receive_data(path):
    try:
        global shared_data
        data = request.json
        if not data:
            return {"error": "No data received"}, 400
            
        if path == "ARUCO_RESULT":
            tag_publisher.publish(str(data))
        else:
            shared_data.update(data)
            and_publisher.publish(str(data))
            # 데이터 수신 즉시 모든 클라이언트에 업데이트 전송
            socketio.emit("update", shared_data)
        return {"status": "data received and published to ROS"}, 200
    except Exception as e:
        rospy.logerr(f"❌ 데이터 처리 오류: {e}")
        return {"error": str(e)}, 500

if __name__ == "__main__":
    try:
        threading.Thread(target=update_data, daemon=True).start()
        socketio.run(app, host="0.0.0.0", port=6002, debug=False, use_reloader=False, log_output=True)
    except rospy.ROSInterruptException:
        pass