import cv2
import numpy as np
import apriltag
from reportlab.lib.pagesizes import A4
from reportlab.pdfgen import canvas
import os

# PDF 설정
pdf_filename = "/root/Downloads/apriltag_36h11.pdf"
c = canvas.Canvas(pdf_filename, pagesize=A4)
page_width, page_height = A4

# 태그 배치 설정
cols, rows = 5, 2  # 5 x 2 배열
spacing_x, spacing_y = 120, 120
start_x, start_y = 50, page_height - 100

# AprilTag 생성기 설정
detector = apriltag.Detector(apriltag.DetectorOptions(families="tag36h11"))

# ID 1~10까지 태그 생성 및 PDF 삽입
for idx, tag_id in enumerate(range(1, 11)):
    # AprilTag 생성
    encoder = apriltag.Encoder(apriltag.EncoderOptions(families="tag36h11"))
    tag_image = encoder.encode(tag_id, 6)  # ID와 해상도 지정

    # 이미지 변환
    tag_image = np.array(tag_image, dtype=np.uint8) * 255

    # 테두리 추가
    total_size = 75
    tag_size = 60
    border = (total_size - tag_size) // 2
    bordered_image = np.ones((total_size, total_size), dtype=np.uint8) * 255
    bordered_image[border:border+tag_size, border:border+tag_size] = tag_image

    # 이미지 파일 저장
    img_filename = f"/root/Downloads/apriltag_36h11_{tag_id}.png"
    cv2.imwrite(img_filename, bordered_image)

    # PDF에 이미지 삽입
    x = start_x + (idx % cols) * spacing_x
    y = start_y - (idx // cols) * spacing_y
    c.drawImage(img_filename, x, y, width=75, height=75)
    c.drawString(x, y - 10, f"ID: {tag_id}")

# PDF 저장
c.save()
print(f"PDF 저장 완료: {pdf_filename}")
