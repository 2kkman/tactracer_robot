import cv2
import time
import os
import datetime

capture = cv2.VideoCapture(0)       #노트북 내장 웹캠은 0, USB웹캠은 2로 하면 연결되었음.
capture.set(cv2.CAP_PROP_FRAME_WIDTH, 3264)
capture.set(cv2.CAP_PROP_FRAME_HEIGHT,2448)
capture.set(cv2.CAP_PROP_AUTOFOCUS,0)
print(capture.get(cv2.CAP_PROP_AUTOFOCUS))
print(capture.get(cv2.CAP_PROP_FOCUS))
print(capture.get(cv2.CAP_PROP_AUTO_WB))
print(capture.get(cv2.CAP_PROP_FORMAT))
print(capture.get(cv2.CAP_PROP_ZOOM))


while True:
    ret, frame= capture.read()
    cv2.imshow("original", frame)

    if cv2.waitKey(1) == ord('q'):          #웹캠 프로그램을 종료할때는 키보드 q를 누름.
        print(ret)
        #cv2.imwrite(f'~/SpiderGo/{(int)(datetime.datetime.now().timestamp())}.png',frame)
        cv2.imwrite(f'/root/SpiderGo/{(int)(datetime.datetime.now().timestamp())}.jpg',frame, params=[cv2.IMWRITE_JPEG_QUALITY,100])
        break

capture.release()

cv2.destroyAllWindows()

 