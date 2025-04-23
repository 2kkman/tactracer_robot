import numpy as np
from sklearn.cluster import DBSCAN
import pcl
import os
dirPath = os.path.dirname(__file__)
filePath_modbusconfig = f'{dirPath}/submap_0.pcd'
# LiDAR 데이터 로드 (예시: 'lidar_data.pcd')
cloud = pcl.load(filePath_modbusconfig)

# 포인트 클라우드를 numpy 배열로 변환
cloud_np = cloud.to_array()

# DBSCAN 클러스터링
dbscan = DBSCAN(eps=0.3, min_samples=10)
clusters = dbscan.fit_predict(cloud_np)

# 클러스터마다 순회
for cluster_id in np.unique(clusters):
    if cluster_id == -1:
        # 잡음 포인트는 무시
        continue

    # 이 클러스터에 속하는 포인트들
    points_in_cluster = cloud_np[clusters == cluster_id]

    # 이 클러스터의 포인트 수가 일정 수준 이상이면 장애물로 간주
    if len(points_in_cluster) > 10:
        print(f"Obstacle detected, cluster id: {cluster_id}, points: {len(points_in_cluster)}")
