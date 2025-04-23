import bluetooth
import struct

# CRC 계산 함수 (Modbus RTU에서 사용)
def calc_crc(data):
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for _ in range(8):
            if (crc & 0x0001) != 0:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc

# Modbus 요청 패킷 생성 (슬레이브 주소, 기능 코드, 시작 주소, 레지스터 수)
def create_modbus_request(slave_addr, function_code, start_addr, register_count):
    request = struct.pack('>B', slave_addr) + struct.pack('>B', function_code) + struct.pack('>H', start_addr) + struct.pack('>H', register_count)
    crc = calc_crc(request)
    crc_bytes = struct.pack('<H', crc)  # CRC는 리틀 엔디안으로 패킹
    return request + crc_bytes
  
# BMS 장치의 블루투스 주소
bms_address = "20:20:06:01:01:A1"  # BMS 장치의 MAC 주소로 변경

# 블루투스 소켓 생성
sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)

# BMS에 연결 (포트는 일반적으로 1번을 사용, 필요에 따라 변경)
port = 1
sock.connect((bms_address, port))

print("BMS에 연결되었습니다.")

# BMS로부터 데이터를 읽는 함수
def read_bms_data(sock):
    try:
        while True:
            # 데이터를 수신 (1024바이트를 읽음, 필요시 수정)
            data = sock.recv(1024)
            if data:
                print(f"BMS 데이터 수신: {data}")
            else:
                break
    except bluetooth.btcommon.BluetoothError as e:
        print(f"블루투스 오류 발생: {e}")
    finally:
        # 소켓 닫기
        sock.close()

# 데이터 읽기 시작
#read_bms_data(sock)
def parse_modbus_response(response):
    if len(response) < 5:
        raise ValueError("응답 길이가 너무 짧습니다.")
    
    slave_addr, function_code, byte_count = struct.unpack('>BBB', response[:3])
    
    # 레지스터 값은 2바이트씩 나누어져 있음
    register_values = []
    for i in range(byte_count // 2):
        register_value = struct.unpack('>H', response[3 + i*2 : 5 + i*2])[0]
        register_values.append(register_value)
    
    # CRC는 마지막 2바이트
    crc_received = struct.unpack('<H', response[-2:])[0]
    
    # CRC 계산해서 비교
    if calc_crc(response[:-2]) != crc_received:
        raise ValueError("CRC 불일치")
    
    return slave_addr, function_code, register_values
# # 논블로킹 모드 설정
# sock.setblocking(False)

# try:
#     response = sock.recv(10240)
#     if response:
#         print(f"응답: {response}")
#     else:
#         print("데이터가 아직 도착하지 않았습니다.")
# except bluetooth.BluetoothError as e:
#     print(f"블루투스 오류 발생: {e}")
# except BlockingIOError:
#     print("현재 수신할 데이터가 없습니다.")

# 요청 생성 (슬레이브 주소 1, 기능 코드 0x03, 주소 0x0000, 레지스터 2개)
modbus_request = create_modbus_request(1, 3, 0x1000, 16)
# BMS에 Modbus 요청 전송
sock.send(modbus_request)
# 응답 읽기 (1024 바이트까지 읽음, 필요에 따라 변경)
response = sock.recv(10240)
# Modbus 응답 해석
slave_addr, function_code, registers = parse_modbus_response(response)
print(f"슬레이브 주소: {slave_addr}, 기능 코드: {function_code}, 레지스터 값: {registers}")

