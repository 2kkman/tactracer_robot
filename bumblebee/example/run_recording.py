import subprocess
import os

SCRIPT_PATH = "./record_mjpeg.sh"  # 스크립트 경로

def start_recording(stream_url: str, save_dir: str = "."):
    try:
        # 비동기 실행
        process = subprocess.Popen(
            [SCRIPT_PATH, "start", stream_url, save_dir],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        print(f"녹화 시작: PID={process.pid}")
        return process.pid
    except Exception as e:
        print(f"[오류] 녹화 시작 실패: {e}")

def stop_recording():
    try:
        # stop은 파라미터 없이 실행
        process = subprocess.Popen(
            [SCRIPT_PATH, "stop"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        stdout, stderr = process.communicate()
        print(stdout.decode())
        if stderr:
            print(f"[stderr]: {stderr.decode()}")
    except Exception as e:
        print(f"[오류] 녹화 중지 실패: {e}")

# 예제 실행
if __name__ == "__main__":
    # 실행 예시 (실제 URL로 교체하세요)
    test_url = "https://example.com:8080/stream"
    save_path = "/home/user/Videos"

    # 녹화 시작
    start_recording(test_url, save_path)

    # ... 이후 원하는 시점에서 중지하려면 stop_recording() 호출
    # stop_recording()
