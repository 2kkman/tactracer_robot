from flask import Flask, request, jsonify, Response, stream_with_context
import threading
import time

app = Flask(__name__)

# 서버 상태 및 데이터 저장
current_hz = 0  # 전송 주기 (0이면 전송 중지)
connected_clients = []  # SSE 클라이언트
received_data = []  # 수신된 데이터 저장


# Control 엔드포인트 (주기 설정, 종료/재시작, 상태 반환)
@app.route('/control', methods=['GET'])
def control():
    global current_hz

    # 전송 주기 설정
    hz = request.args.get('hz', type=int)
    if hz is not None:
        current_hz = hz
        return jsonify({"status": "success", "message": f"Transmission rate set to {hz} Hz"})

    # 프로그램 종료/재시작
    terminate = request.args.get('terminate', type=int)
    if terminate == 1:
        print("Terminating the server...")
        shutdown_server()
        return jsonify({"status": "success", "message": "Server terminated"})
    elif terminate is not None:
        print("Restarting the server...")
        # Restart logic could go here
        return jsonify({"status": "success", "message": "Server restarted"})

    # 현재 상태 반환
    return jsonify({
        "current_hz": current_hz,
        "connected_clients": len(connected_clients),
        "received_data": received_data[-5:],  # 최근 5개의 데이터
    })


# Android 폰으로부터 데이터 수신
@app.route('/data', methods=['POST'])
def receive_data():
    data = request.get_json()
    if not data:
        return jsonify({"status": "error", "message": "Invalid data"}), 400

    print(f"Received data: {data}")  # 데이터 출력
    received_data.append(data)
    return jsonify({"status": "success", "message": "Data received"})


# SSE 엔드포인트
@app.route('/events', methods=['GET'])
def events():
    def event_stream():
        while True:
            for data in received_data[-5:]:  # 최근 5개의 데이터 스트리밍
                yield f"data: {data}\n\n"
            time.sleep(1 / max(current_hz, 1))  # 전송 주기 (Hz)

    return Response(stream_with_context(event_stream()), content_type='text/event-stream')


# 서버 종료 함수
def shutdown_server():
    func = request.environ.get('werkzeug.server.shutdown')
    if func is None:
        raise RuntimeError('Not running with the Werkzeug Server')
    func()


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=7000, debug=False)
