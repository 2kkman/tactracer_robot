import requests
from requests.exceptions import RequestException

def send_agv_pause_resume(ipaddress: str, pause: int, resume: int, suspended: int):
    api_url_address = f"https://{ipaddress}:9000/navinfo"
    
    # URL 파라미터로 데이터를 전송
    params = {
        "pause": pause,
        "resume": resume,
        "suspended": suspended
    }
    # params = {
    #     {"tag_serialno":"T12342340sdfr0002","aruco_id": "A1"},
    #     {"tag_serialno":"T12342340sdfr0002","aruco_id": "A2"},
    #     {"tag_serialno":"T12342340sdfr0002","aruco_id": "A3"},
    #     {"tag_serialno":"T12342340sdfr0001","aruco_id": "A4"},
    # }

    try:
        response = requests.get(api_url_address, params=params, verify=False)
        response.raise_for_status()  # Raises a HTTPError if the status is 4xx, 5xx
        return response.json()
    except RequestException as e:
        print(f"An error occurred: {e}")
        return None

# 테스트
if __name__ == "__main__":
    ip_address = "172.30.1.29"  # 또는 실제 서버 IP
    pause = 1
    resume = 0
    suspended = 500000
    
    result = send_agv_pause_resume(ip_address, pause, resume, suspended)
    if result:
        print("Response:", result)
    else:
        print("Failed to get a response")