#!/usr/bin/env python3
from UtilBLB import *
import serial
# import struct
# import time
# import rospy
# from std_msgs.msg import String
# import json

# RS485 설정
SERIAL_PORT = '/dev/ttyUSB1'  # 사용하는 Serial 포트를 지정하세요
SERIAL_PORT = '/dev/ttC485'  # 사용하는 Serial 포트를 지정하세요
BAUD_RATE = 38400 
BMS_ID = 0x01  # 기본 ID
CHARGER_IP=BLB_CHARGERPLUG_IP_DEFAULT
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

# 플래그 해석
def decode_flags(flags, description_map):
    return [desc for bit, desc in description_map.items() if flags & (1 << bit)]

# 배터리 정보 요청 명령 생성
def create_request_command(bms_id):
    header = 0xFA
    command = [0x42, 0x41, 0x54]  # 'B', 'A', 'T'
    checksum = (header + sum(command) + bms_id) & 0xFF
    return bytearray([header] + command + [bms_id, checksum])

# 데이터 파싱
def parse_response(data):
    if len(data) < 60 or data[0] != 0xAF:
        raise ValueError(f"Invalid data received : {data}")
    cell_temperatures_c = [
            struct.unpack('>h', data[28 + i * 2:30 + i * 2])[0] / 10.0 for i in range(2)
        ]
    cell_voltages_mv = [
            struct.unpack('>H', data[34 + i * 2:36 + i * 2])[0] for i in range(13)
        ]
    CurCadc = (struct.unpack('>h', data[15:17])[0] / 10.0)*-1
    Voltage = struct.unpack('>H', data[13:15])[0] / 10.0
    # Chager = get_tasmota_state(CHARGER_IP)
    # if Chager == "ON":
    #   if CurCadc > -1.5 and CurCadc <=0:
    #     set_tasmota_state(CHARGER_IP, "off")
    # elif Chager == "OFF":
    #   if Voltage <=52.5:
    #     set_tasmota_state(CHARGER_IP, "on")
    
    Watt = round(Voltage*CurCadc)
    response = {
        "battery_id": data[4],
        "hardware_version": data[5],
        "firmware_version": data[6],
        "total_capacity_ah": struct.unpack('>H', data[7:9])[0] / 10.0,
        "remaining_capacity_ah": struct.unpack('>H', data[9:11])[0] / 10.0,
        "cycle_count": struct.unpack('>H', data[11:13])[0],
        #MonitoringField_BMS.Charger.name : Chager,
        MonitoringField_BMS.Vmax.name: max(cell_voltages_mv)/1000,
        MonitoringField_BMS.Vmin.name: min(cell_voltages_mv)/1000,
        MonitoringField_BMS.Tmax.name: max(cell_temperatures_c),
        MonitoringField_BMS.Tmin.name: min(cell_temperatures_c),
        MonitoringField_BMS.WATT.name: Watt,
        MonitoringField_BMS.Voltage.name: Voltage,
        MonitoringField_BMS.CurCadc.name: CurCadc,
        MonitoringField_BMS.RSOC.name: struct.unpack('>H', data[17:19])[0] / 10.0,
        "soh_percent": data[19],
        "serial_cell_count": data[20],
        "battery_status": BATTERY_STATUS.get(data[21], "Unknown"),
        "protection_flags": sDivEmart.join(decode_flags(struct.unpack('>H', data[22:24])[0], PROTECTION_FLAG)),
        "warning_flags": sDivEmart.join(decode_flags(struct.unpack('>H', data[24:26])[0], WARNING_FLAG)),
        #"protection_flags_little": sDivEmart.join(decode_flags(struct.unpack('<H', data[22:24])[0], PROTECTION_FLAG)),
        #"warning_flags_little": sDivEmart.join(decode_flags(struct.unpack('<H', data[24:26])[0], WARNING_FLAG)),
        "cell_temperatures_c": sDivEmart.join(map(str, cell_temperatures_c)),
        "cell_voltages_mv": sDivEmart.join(map(str,cell_voltages_mv))
    }
    #print(" ".join(f"{b:02X}" for b in data))
    width = 10
    for i, b in enumerate(data):
        if i % width == 0 and i != 0:
            print()  # 개행
        print(f"{b:02X} ", end="")
    print()  # 마지막 개행
    return response

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
    rospy.init_node('bms_publisher', anonymous=True)
    pub = rospy.Publisher(TopicName.BMS.name, String, queue_size=10)

    try:
        ser = init_serial()
        request_command = create_request_command(BMS_ID)
        
        rate = rospy.Rate(0.5)  # 0.5Hz
        while not rospy.is_shutdown():
            ser.write(request_command)
            response = ser.read(63)
            try:
                data = parse_response(response)
                json_data = json.dumps(data)
                pub.publish(json_data)
                rospy.loginfo(f"Published: {json_data}")
            except ValueError as e:
                rospy.logerr(f"Error parsing response: {e}")
            rate.sleep()
    except serial.SerialException as e:
        rospy.logerr(f"Serial communication error: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()
