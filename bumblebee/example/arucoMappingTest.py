import math
from UtilBLB import *

#print(mapRange(370, 0,760,0,1080))


def calculate_position_and_movement(pixel_x, pixel_y):
    # 픽셀 당 실제 거리 계산 (mm/pixel)
    mm_per_pixel_x = 1410 / 930
    #mm_per_pixel_y = 760 / 895
    mm_per_pixel_y = 760 / 895

    # 실제 x, y 거리 계산 (mm)
  
    real_x = pixel_x * mm_per_pixel_x
    real_y = pixel_y * mm_per_pixel_y
    print(f'1픽셀x 길이:{mm_per_pixel_x}mm,1픽셀y 길이:{mm_per_pixel_y}mm')
    print(f'POS_X:{real_x}mm,POS_Y:{real_y}mm')
    #카메라 오프셋. X축 70mm, Y축 -250mm
    real_y -= 70
    real_x += 260
    print(f'최종 POS_X:{real_x}mm,최종 POS_Y:{real_y}mm')
    
    # 대상까지의 직선 거리 계산
    distance = math.sqrt(real_x**2 + real_y**2)

    # 회전 각도 계산 (라디안)
    angle_rad = math.atan2(real_y, real_x)
    
    # 라디안을 도로 변환
    angle_deg = math.degrees(angle_rad)

    return distance, angle_deg

def calculate_position_and_movementXY(real_x, real_y):
    print(f'POS_X:{real_x}mm,POS_Y:{real_y}mm')
    #카메라 오프셋. X축 70mm, Y축 -250mm
    real_y += 70
    real_x += 240
    print(f'최종 POS_X:{real_x}mm,최종 POS_Y:{real_y}mm')
    
    # 대상까지의 직선 거리 계산
    distance = math.sqrt(real_x**2 + real_y**2)

    # 회전 각도 계산 (라디안)
    angle_rad = math.atan2(real_y, real_x)
    
    # 라디안을 도로 변환
    angle_deg = math.degrees(angle_rad)

    return distance, angle_deg

dicTmp2 = {"DIFF_X": -28.93, "DIFF_Y": 27.31, "ANGLE": -178, "CAM_ID": 0, "MARKER_VALUE"
  : 2, "X": -438, "Y": 203, "Z": 2503}

dicTmp7 = {"DIFF_X": -45.53, "DIFF_Y": 44.4, "ANGLE": -163, "CAM_ID": 0, "MARKER_VALUE"
  : 7, "X": -189, "Y": 161, "Z": 3627}

dicTmp6 = {"DIFF_X": -45.53, "DIFF_Y": 44.42, "ANGLE": -163, "CAM_ID": 0, "MARKER_VALUE"
  : 6, "X": -2, "Y": 5, "Z": 3627}

dicTmp9 = {"DIFF_X": 38.1, "DIFF_Y": -16.09, "ANGLE": 97, "CAM_ID": 0, "MARKER_VALUE"
  : 8, "X": 592, "Y": -178, "Z": 2617}

dicTmp8 = {"DIFF_X": 25.29, "DIFF_Y": -25.46, "ANGLE": -163, "CAM_ID": 2, "MARKER_VALUE"
  : 9, "X": 58+554, "Y": -51-86, "Z": 3627}
lsTmp = [dicTmp7,dicTmp8,dicTmp6,dicTmp2,dicTmp9]
# dicTmp = {"DIFF_X": 25.29, "DIFF_Y": -25.46, "ANGLE": -163, "CAM_ID": 0, "MARKER_VALUE"
#   : 3, "X": 537, "Y": 209, "Z": 3627}

#P1 - 범블비 원점 (회전중심)
#P2 - 현재 카메라의 센터 좌표.
#P3 - 아르코마커의 좌표

P2X = 93 #현재 카메라의 센터 (이 상태에서 트레이 내려가면 딱 맞음)
P2Y = -160

P1X = P2X
P1Y = P2Y+660 #660 은 현재 암 뻗은 길이

P3X = 225-P2X
P3Y = 58-P2Y

print(f'P1:{P1X},{P1Y},P1:{P2X},{P2Y},P3:{P3X},{P3Y}')
print(calculate_triangle(P1X,P1Y,P2X,P2Y,P3X,P3Y))

#print(calculate_length_and_angle(426, -(58-(-554)), -(-51-(86))))

for dicTmp in lsTmp:
  
  #x,y = CalculatePosXYFromAruco(dicTmp, 0)
  X,Y = CalculatePosXYFromAruco(dicTmp,270,-2,5)
  #X,Y = CalculatePosXYFromAruco(dicTmp,270)
  dist, angle = calculate_distance_and_angle(X, Y)    
  #dist,angle = CalculateDistanceAngleFromArucoTop(dicTmp)
  #print(dicTmp,calculate_distance_and_angle(x,y))
  print(dicTmp,dist,angle)

# pixel_x = -473
# pixel_y = 169

# pixel_x = -604
# pixel_y = 5



# distance, angle = calculate_position_and_movementXY(pixel_x, pixel_y)
# print(f"팔을 뻗어야 할 거리: {distance:.2f} mm")
# print(f"회전해야 할 각도: {angle:.2f} 도")