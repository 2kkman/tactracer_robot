import numpy as np
import pcl
import os
dirPath = os.path.dirname(__file__)
filePath_modbusconfig = f'{dirPath}/cyg_3d_full.pcd'
# LiDAR 데이터 로드 (예시: 'lidar_data.pcd')
cloud = pcl.load(filePath_modbusconfig)

# 포인트 클라우드를 numpy 배열로 변환
cloud_np = cloud.to_array()

# 각 포인트의 z (높이) 값을 추출
z_values = cloud_np[:, 0]
print(z_values)

# 임계값을 설정 (예시: 1m)
threshold = 1.0

# 임계값 이상의 z 값을 갖는 포인트를 장애물로 간주
obstacles = cloud_np[z_values > threshold]

# 결과 출력
print(obstacles)
