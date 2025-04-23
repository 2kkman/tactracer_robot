import cv2
import time
import os
import datetime
 

capture = cv2.VideoCapture(1)       #노트북 내장 웹캠은 0, USB웹캠은 2로 하면 연결되었음.

capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)

capture.set(cv2.CAP_PROP_FRAME_HEIGHT,1080)

 

while True:

    ret, frame= capture.read()
    cv2.imwrite(f'/root/Downloads/{(int)(getDateTime().timestamp())}.jpg',frame, params=[cv2.IMWRITE_JPEG_QUALITY,100])
    cv2.imshow("original", frame)

    if cv2.waitKey(1) == ord('q'):          #웹캠 프로그램을 종료할때는 키보드 q를 누름.
        print(ret)
        #cv2.imwrite(f'~/SpiderGo/{(int)(getDateTime().timestamp())}.png',frame)
        cv2.imwrite(f'/root/Downloads/{(int)(getDateTime().timestamp())}.jpg',frame, params=[cv2.IMWRITE_JPEG_QUALITY,100])
        break

 

capture.release()

cv2.destroyAllWindows()

 