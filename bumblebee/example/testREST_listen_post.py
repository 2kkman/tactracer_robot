from flask import Flask, request, render_template_string
from flask_socketio import SocketIO
import rospy
import threading
import time
from std_msgs.msg import String
from enum import Enum, auto

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

class TopicName(Enum):
    CMD_DEVICE = auto()
    KEEPALIVE = auto()
    MB_ = auto()
    ACK = auto()
    ARD_CARRIER = auto()  # 새로운 토픽 추가

shared_data = {topic.name: {} for topic in TopicName}

rospy.init_node('imu_data_receiver', anonymous=True)

def callback_factory(topic_name):
    def callback(msg):
        global shared_data
        shared_data[topic_name] = eval(msg.data)
        print(f"📥 {topic_name} 데이터 수신: {shared_data[topic_name]}")
        socketio.emit(f"update_{topic_name}", shared_data[topic_name])
    return callback

subscribers = {}
for topic in TopicName:
    topic_name = topic.name
    subscribers[topic_name] = rospy.Subscriber(f"/{topic_name}", String, callback_factory(topic_name))

@app.route("/<topic_name>")
def topic_page(topic_name):
    if topic_name not in shared_data:
        result = ','.join(subscribers.keys())
        return f"<h1>토픽 {topic_name}은 존재하지 않습니다.{result}</h1>", 404

    line_limit = request.args.get("line", default=30, type=int)  # line 파라미터 받기

    return render_template_string(f"""
    <!DOCTYPE html>
    <html lang="ko">
    <head>
        <meta charset="UTF-8">
        <title>{topic_name} 실시간 데이터</title>
        <link rel="icon" href="data:,">
        <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.1/socket.io.js"></script>
        <style>
            body {{
                font-family: Arial, sans-serif;
                margin: 20px;
            }}
            h1 {{ text-align: center; }}
            #connection-status {{
                text-align: center;
                margin-bottom: 20px;
                font-size: 1.2em;
            }}
            .data-container {{
                display: flex;
                flex-wrap: wrap;
                gap: 10px;
            }}
            .column {{
                flex: 1;
                min-width: 250px;
                border: 1px solid #ddd;
                padding: 10px;
                background: #f9f9f9;
                border-radius: 5px;
            }}
            .data-item {{
                margin: 5px 0;
                padding: 5px;
                border-bottom: 1px solid #eee;
            }}
            .timestamp {{
                color: #666;
                font-size: 0.8em;
            }}
        </style>
    </head>
    <body>
        <h1>{topic_name} 실시간 데이터</h1>
        <div id="connection-status">연결 상태: 대기중...</div>
        <div class="data-container" id="data"></div>

        <script>
            let socket;
            const lineLimit = {{ line_limit }};  // Python에서 받은 line 값을 JS에서 사용
            
            function connectWebSocket() {{
                socket = io("ws://" + location.hostname + ":6001", {{
                    transports: ["websocket"],
                    reconnection: true,
                    reconnectionDelay: 1000,
                    reconnectionDelayMax: 5000,
                    reconnectionAttempts: Infinity
                }});

                socket.on("connect", () => {{
                    console.log("WebSocket 연결됨");
                    document.getElementById("connection-status").innerHTML = 
                        '<span style="color: green;">연결 상태: 연결됨</span>';
                }});

                socket.on("disconnect", () => {{
                    console.log("WebSocket 연결 끊김");
                    document.getElementById("connection-status").innerHTML = 
                        '<span style="color: red;">연결 상태: 연결 끊김</span>';
                }});

                socket.on("update_{topic_name}", (data) => {{
                    console.log("데이터 수신:", data);
                    updateDisplay(data);
                }});

                socket.on("connect_error", (error) => {{
                    console.error("연결 오류:", error);
                    document.getElementById("connection-status").innerHTML = 
                        '<span style="color: red;">연결 상태: 오류 발생</span>';
                }});
            }}

            function updateDisplay(data) {{
                let keys = Object.keys(data);
                let totalLines = keys.length;
                let numColumns = Math.ceil(totalLines / lineLimit); // 컬럼 개수 계산
                let columnData = Array.from({{length: numColumns}}, () => []);

                keys.forEach((key, index) => {{
                    let columnIndex = Math.floor(index / lineLimit);
                    columnData[columnIndex].push(`<div class='data-item'>
                        <strong>${{key}}:</strong> ${{data[key]}}
                        <div class='timestamp'>${{new Date().toLocaleTimeString()}}</div>
                    </div>`);
                }});

                let displayText = "<div class='data-container'>";
                columnData.forEach(column => {{
                    displayText += `<div class='column'>${{column.join("")}}</div>`;
                }});
                displayText += "</div>";

                document.getElementById("data").innerHTML = displayText;
            }}

            document.addEventListener('DOMContentLoaded', connectWebSocket);
            window.addEventListener('online', connectWebSocket);
        </script>
    </body>
    </html>
    """)

def update_data():
    while True:
        for topic_name, data in shared_data.items():
            if data:
                socketio.emit(f"update_{topic_name}", data)
        time.sleep(1)

@socketio.on("connect")
def handle_connect():
    print("✅ 클라이언트가 WebSocket에 연결됨")
    for topic_name, data in shared_data.items():
        if data:
            socketio.emit(f"update_{topic_name}", data)

if __name__ == "__main__":
    try:
        threading.Thread(target=update_data, daemon=True).start()
        socketio.run(app, host="0.0.0.0", port=6001, debug=False, use_reloader=False, log_output=True)
    except rospy.ROSInterruptException:
        pass
