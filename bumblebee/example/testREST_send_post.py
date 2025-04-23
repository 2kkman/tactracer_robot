import requests
import json
from requests.exceptions import RequestException

def send_agv_pause_resume(ipaddress: str, pause: int, resume: int, suspended: int):
    api_url_address = f"https://{ipaddress}:9000/nav_state"
    url = api_url_address
    #url = api_url_address + 'nav_state'
    
    body = {
        "pause": pause,
        "resume": resume,
        "suspended": suspended
    }
    
    headers = {
        'Content-Type': 'application/json'
    }
    
    try:
        response = requests.post(url, data=json.dumps(body), headers=headers, verify=False)
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