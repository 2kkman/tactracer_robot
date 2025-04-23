import requests

def sse_client(url):
    """
    Connect to an SSE server and print messages as they are received.
    :param url: The URL of the SSE server.
    """
    try:
        # SSE 서버에 연결
        with requests.get(url, stream=True) as response:
            if response.status_code != 200:
                print(f"Failed to connect: {response.status_code}")
                return

            print("Connected to SSE server. Receiving RFID data...")
            
            buffer = ""  # 버퍼를 초기화

            # 스트림 데이터 실시간 처리
            for chunk in response.iter_content(chunk_size=1, decode_unicode=True):
                if chunk:
                    buffer += chunk  # 데이터를 버퍼에 추가
                    if chunk == ":":  # ':' 문자를 기준으로 메시지 구분
                        dataTmp = buffer.strip()
                        if len(dataTmp) > 10:
                            print(f"RFID Data: {dataTmp}")
                        else:
                            print(buffer)
                        buffer = ""  # 버퍼 초기화
    except requests.exceptions.RequestException as e:
        print(f"Error connecting to SSE server: {e}")
    except KeyboardInterrupt:
        print("\nDisconnected from SSE server.")

if __name__ == "__main__":
    # ESP32-S3 SSE 서버 주소
    sse_url = "http://172.30.1.8:9001/EPC"

    while True:
        print("Attempting to connect to the SSE server...")
        sse_client(sse_url)
        print("Reconnecting in 5 seconds...")
