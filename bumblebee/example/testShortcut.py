import random

# 그리드 크기
width = 5
height = 5

# 빈 그리드 생성
grid = [[0 for _ in range(width)] for _ in range(height)]

# 장애물 생성
for _ in range(int(width * height * 0.3)):  # 약 30%가 장애물
    x, y = random.randint(0, width - 1), random.randint(0, height - 1)
    grid[y][x] = 1

# 시작점과 목표점 설정
start = (0, 0)  # 시작점 (0, 0)
goal = (width - 1, height - 1)  # 목표점 (오른쪽 하단)
grid[0][0] = "S"  # 시작 지점을 S로 표시
grid[height - 1][width - 1] = "G"  # 목표 지점을 G로 표시

# 그리드 출력
for row in grid:
    print(" ".join(str(cell) for cell in row))
