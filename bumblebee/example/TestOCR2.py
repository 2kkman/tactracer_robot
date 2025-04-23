import csv
import cv2
from pytesseract import pytesseract as pt
from PIL import Image
fp = '/root/SpiderGo/test_1675221806_07.png'
image = Image.open(fp)
ret = pt.image_to_boxes(image)

print(ret)

# To read the coordinates
boxes = []
with open('output.box', 'rb') as f:
    reader = csv.reader(f, delimiter = ' ')
    for row in reader:
        if(len(row)==6):
            boxes.append(row)

# Draw the bounding box
img = cv2.imread(fp)
h, w, _ = img.shape
for b in boxes:
    img = cv2.rectangle(img,(int(b[1]),h-int(b[2])),(int(b[3]),h-int(b[4])),(255,0,0),2)

cv2.imshow('output',img)