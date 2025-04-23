import serial
import struct
import time

# RS485 설정
SERIAL_PORT = '/dev/ttyUSB1'  # 사용하는 Serial 포트를 지정하세요
BAUD_RATE = 38400
BMS_ID = 0x01  # 기본 ID

# 배터리 상태와 플래그에 대한 설명
BATTERY_STATUS = {
    0: "Idle (Ready)",
    1: "Discharge",
    2: "Charge"
}

PROTECTION_FLAG = {
    0: "Cell over-voltage",
    1: "Cell under-voltage",
    2: "Discharge over-current",
    3: "Charge over-current",
    4: "Short circuit",
    5: "Charge over-temperature",
    6: "Charge under-temperature",
    7: "Discharge over-temperature",
    8: "Discharge under-temperature",
    9: "Pack over-voltage",
    10: "Pack under-voltage",
    11: "BMS IC Error",
    12: "Cell imbalance",
    13: "MOSFET Failure",
    14: "Reserved",
    15: "Reserved"
}

WARNING_FLAG = {
    0: "Pack over-voltage",
    1: "Pack under-voltage",
    2: "Cell over-voltage",
    3: "Cell under-voltage",
    4: "Discharge over-current",
    5: "Charge over-current",
    6: "Over-temperature",
    7: "Under-temperature",
    8: "Reserved",
    9: "Reserved",
    10: "Reserved",
    11: "Reserved",
    12: "Reserved",
    13: "Reserved",
    14: "Reserved",
    15: "Reserved"
}

# 배터리 정보 요청 명령 생성
def create_request_command(bms_id):
    header = 0xFA
    command = [0x42, 0x41, 0x54]  # 'B', 'A', 'T'
    checksum = (header + sum(command) + bms_id) & 0xFF
    return bytearray([header] + command + [bms_id, checksum])

# 데이터 파싱
def parse_response(data):
    if len(data) < 60 or data[0] != 0xAF:
        raise ValueError("Invalid data received")
    
    response = {
        "Battery ID": data[4],
        "Hardware Version": data[5],
        "Firmware Version": data[6],
        "Total Capacity (Ah)": struct.unpack('>H', data[7:9])[0] / 10.0,
        "Remaining Capacity (Ah)": struct.unpack('>H', data[9:11])[0] / 10.0,
        "Cycle Count": struct.unpack('>H', data[11:13])[0],
        "Pack Voltage (V)": struct.unpack('>H', data[13:15])[0] / 10.0,
        "Pack Current (A)": struct.unpack('>h', data[15:17])[0] / 10.0,
        "SOC (%)": struct.unpack('>H', data[17:19])[0] / 10.0,
        "SOH (%)": data[19],
        "Serial Cell Count": data[20],
        "Battery Status": BATTERY_STATUS.get(data[21], "Unknown"),
        "Protection Flags": decode_flags(struct.unpack('>H', data[22:24])[0], PROTECTION_FLAG),
        "Warning Flags": decode_flags(struct.unpack('>H', data[24:26])[0], WARNING_FLAG),
        "Cell Temperatures (°C)": [
            struct.unpack('>h', data[28 + i * 2:30 + i * 2])[0] / 10.0 for i in range(2)
        ],
        "Cell Voltages (mV)": [
            struct.unpack('>H', data[34 + i * 2:36 + i * 2])[0] for i in range(13)
        ]
    }
    return response

# 플래그 해석
def decode_flags(flags, description_map):
    return [desc for bit, desc in description_map.items() if flags & (1 << bit)]

# Serial 통신 초기화
def init_serial():
    return serial.Serial(
        port=SERIAL_PORT,
        baudrate=BAUD_RATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=1
    )

def main():
    try:
        ser = init_serial()
        request_command = create_request_command(BMS_ID)
        
        while True:
            ser.write(request_command)
            response = ser.read(100)
            try:
                data = parse_response(response)
                print(len(response))
                print("Battery Data:")
                for key, value in data.items():
                    print(f"{key}: {value}")
                print("-" * 50)
            except ValueError as e:
                print("Error parsing response:", e)
            time.sleep(1)
    except serial.SerialException as e:
        print("Serial communication error:", e)
    finally:
        if ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()
