import cv2

# 카메라 캡처 객체 생성
cap = cv2.VideoCapture(0)  # 0번 카메라를 사용 (만약 여러 카메라가 연결되어 있다면 다른 인덱스를 사용할 수 있음)

# 움직임 감지를 위한 배경 차분 객체 생성
fgbg = cv2.createBackgroundSubtractorMOG2()

while True:
    # 프레임 읽기
    ret, frame = cap.read()
    if not ret:
        break

    # 배경 차분 수행
    fgmask = fgbg.apply(frame)

    # 노이즈 제거를 위한 모폴로지 연산 수행
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_OPEN, kernel)

    # 이진화 처리
    _, thresh = cv2.threshold(fgmask, 128, 255, cv2.THRESH_BINARY)

    # 감지된 물체의 외곽선 검출
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 외곽선 그리기
    for contour in contours:
        # 외곽선을 감싸는 사각형 그리기
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # 카메라로부터 캡처된 이미지를 화면에 표시
    cv2.imshow('Camera', frame)

    # 'q'를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 자원 해제
cap.release()
cv2.destroyAllWindows()
