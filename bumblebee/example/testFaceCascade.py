import cv2
import numpy as np

def decode_video():
    global video_width
    global video_height
    
    # a. Decode video from camera
    #vc0 = cv2.VideoCapture(0)
    vc0 = cv2.VideoCapture(0,cv2.CAP_V4L)
    object_detection = cv2.createBackgroundSubtractorMOG2()
    # # b. Decode video file
    # video_file = "Put your video file path here."
    # vc = cv2.VideoCapture(video_file)
    
    video_width  = 4000
    video_height = 3000
    #video_width  = int(vc.get(cv2.CAP_PROP_FRAME_WIDTH))
    #video_height = int(vc.get(cv2.CAP_PROP_FRAME_HEIGHT))
    #vc.set(cv2.CAP_PROP_FRAME_WIDTH, 3840) #set width
    #vc.set(cv2.CAP_PROP_FRAME_HEIGHT, 2880) #set height
    vc0.set(cv2.CAP_PROP_AUTOFOCUS,1)
    #vc.set(cv2.CAP_PROP_EXPOSURE,156)
    vc0.set(3, video_width) #set width
    vc0.set(4, video_height) #set height
    vc0.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))    
    print("CV_CAP_PROP_FRAME_WIDTH: '{}'".format(vc0.get(cv2.CAP_PROP_FRAME_WIDTH)))
    print("CV_CAP_PROP_FRAME_HEIGHT : '{}'".format(vc0.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    print("CAP_PROP_FPS : '{}'".format(vc0.get(cv2.CAP_PROP_FPS)))
    print("CAP_PROP_POS_MSEC : '{}'".format(vc0.get(cv2.CAP_PROP_POS_MSEC)))
    print("CAP_PROP_FRAME_COUNT : '{}'".format(vc0.get(cv2.CAP_PROP_FRAME_COUNT)))
    print("CAP_PROP_BRIGHTNESS : '{}'".format(vc0.get(cv2.CAP_PROP_BRIGHTNESS)))
    print("CAP_PROP_CONTRAST : '{}'".format(vc0.get(cv2.CAP_PROP_CONTRAST)))
    print("CAP_PROP_SATURATION : '{}'".format(vc0.get(cv2.CAP_PROP_SATURATION)))
    print("CAP_PROP_HUE : '{}'".format(vc0.get(cv2.CAP_PROP_HUE)))
    print("CAP_PROP_GAIN : '{}'".format(vc0.get(cv2.CAP_PROP_GAIN)))
    print("CAP_PROP_CONVERT_RGB : '{}'".format(vc0.get(cv2.CAP_PROP_CONVERT_RGB)))
    print("CAP_PROP_FOCUS : '{}'".format(vc0.get(cv2.CAP_PROP_FOCUS)))
    print("CAP_PROP_AUTOFOCUS : '{}'".format(vc0.get(cv2.CAP_PROP_AUTOFOCUS)))
    print("CAP_PROP_EXPOSURE : '{}'".format(vc0.get(cv2.CAP_PROP_EXPOSURE)))
    print("CAP_PROP_FOURCC : '{}'".format(vc0.get(cv2.CAP_PROP_FOURCC)))

    stride = 0
    if vc0.isOpened():  
        rval, frame = vc0.read()
        stride = frame.strides[0]
        
    else:
        return

    windowName = "Video"
    windowName1 = "Video THRESH_OTSU"
    windowName2 = "Video ADAPTIVE_THRESH_MEAN_C"
    windowName3 = "Video ADAPTIVE_THRESH_GAUSSIAN_C"
    
    def onMouse(x):
        pass
    
    # # start video decoding. Pass three callbacks at the same time.
    # reader.start_video_mode(parameters, text_results_callback_func, "", intermediate_results_callback_func, error_callback_func, reader)
    cv2.namedWindow(windowName2, cv2.WINDOW_NORMAL)
    cv2.createTrackbar("Canny Min", windowName2, 0, 360, onMouse)
    cv2.createTrackbar("Canny Max", windowName2, 0, 360, onMouse)
    cv2.createTrackbar("Threshold", windowName2, 0, 200, onMouse)
    
    faceCascade = cv2.CascadeClassifier('/root/catkin_ws/src/tta_blb/example/haarcascade_frontalface_default.xml')
    eye_cascade = cv2.CascadeClassifier('/root/catkin_ws/src/tta_blb/example/haarcascade_eye_tree_eyeglasses.xml')
    
    while True:
        
        image = cv2.resize(frame, (640, 480))
        image_roi = image.copy()
        # ret,threshold1 = cv2.threshold(image,127,255,cv2.THRESH_TRUNC)
        block_size = 15
        C = 15
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # blur = cv2.medianBlur(gray, 7)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        bilateral = cv2.bilateralFilter(gray, 9, 75, 75)
        
        canny_min = cv2.getTrackbarPos("Canny Min", windowName2) 
        canny_max = cv2.getTrackbarPos("Canny Max", windowName2) 
        threshold_val = cv2.getTrackbarPos("Threshold", windowName2) 
        
        faces = faceCascade.detectMultiScale( #이미지에서 얼굴을 검출
            gray, #grayscale로 이미지 변환한 원본.
            scaleFactor=1.2, #이미지 피라미드에 사용하는 scalefactor
            #scale 안에 들어가는 이미지의 크기가 1.2씩 증가 즉 scale size는 그대로
            # 이므로 이미지가 1/1.2 씩 줄어서 scale에 맞춰지는 것이다.
            minNeighbors=3, #최소 가질 수 있는 이웃으로 3~6사이의 값을 넣어야 detect가 더 잘된다고 한다.
            #Neighbor이 너무 크면 알맞게 detect한 rectangular도 지워버릴 수 있으며,
            #너무 작으면 얼굴이 아닌 여러개의 rectangular가 생길 수 있다.
            #만약 이 값이 0이면, scale이 움직일 때마다 얼굴을 검출해 내는 rectangular가 한 얼굴에
            #중복적으로 발생할 수 있게 된다.
            minSize=(20, 20) #검출하려는 이미지의 최소 사이즈로 이 크기보다 작은 object는 무시
            #maxSize도 당연히 있음.
        )
        for (x,y,w,h) in faces: #좌표 값과 rectangular의 width height를 받게 된다.
            #x,y값은 rectangular가 시작하는 지점의 좌표
            #원본 이미지에 얼굴의 위치를 표시하는 작업을 함.
            #for문을 돌리는 이유는 여러 개가 검출 될 수 있기 때문.
            cv2.rectangle(image,(x,y),(x+w,y+h),(255,0,0),1)
            #다른 부분, 얼굴 안에 들어있는 눈과 입 등을 검출할 때 얼굴 안엣 검출하라는 의미로 이용되는 것
            roi_gray = gray[y:y+h, x:x+w] #눈,입을 검출할 때 이용
            roi_color = image[y:y+h, x:x+w] #눈,입등을 표시할 때 이용
            eyes = eye_cascade.detectMultiScale(roi_gray)
            for (ex, ey, ew, eh) in eyes:
                cv2.rectangle(roi_color, (ex, ey), (ex+ew, ey+eh), (0, 255, 0), 2)
        # edge = cv2.Canny(blur, 250, 340)
        edge = cv2.Canny(bilateral, canny_min, canny_max)
        # ret, th1 = cv2.threshold(image, 0, 255, cv2.THRESH_OTSU)
        th2 = cv2.adaptiveThreshold(blur,255,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY,block_size,C)
        # th3 = cv2.adaptiveThreshold(blur,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,block_size,C)
        # image_mask = cv2.resize(mask, (640, 480))

        
        # Defined region of interest
        rows, cols = image.shape[:2]
        bottom_left = [cols * 0.1, rows]
        top_left = [cols * 0.2, rows*0.1]
        bottom_right = [cols * 0.9, rows]
        top_right = [cols * 0.8, rows*0.1]
        vertices = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)
        
        # Masked image
        region_of_interest_image = region_of_interest(edge, vertices)
        
        # Hough transform
        lines = cv2.HoughLinesP(region_of_interest_image, 1, np.pi/180, threshold=threshold_val, maxLineGap=30)
        
        # lines = cv2.HoughLinesP(edge, 1, np.pi/180, threshold=120, minLineLength=30, maxLineGap=50)
        # cv2.imshow(windowName2, image_mask)
        if lines is not None:
            frame_with_lines = draw_lines(image, lines)
        else:
            frame_with_lines = image
        numpy_horizontal_cat = cv2.hconcat((image, frame_with_lines))

        # 이미지를 LAB 색 공간으로 변환 (L: 밝기, A: 초록-빨강, B: 파랑-노랑)
        lab = cv2.cvtColor(image, cv2.COLOR_BGR2Lab)
        # LAB 색 공간의 각 채널 분리
        l, a, b = cv2.split(lab)

        # L 채널에 CLAHE 적용
        # cl = clahe.apply(l)
        cl = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8)).apply(l)
        cg = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8)).apply(gray)
        
        hsv = cv2.cvtColor(cg, cv2.COLOR_GRAY2BGR)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        
        hsv_roi = cv2.cvtColor(image_roi, cv2.COLOR_BGR2HSV)
        
        # roi_hist = cv2.calcHist([hsv_roi], [0, 1], None, [180, 256], [0, 180, 0, 256])
        roi_hist = cv2.calcHist([hsv_roi], [0], None, [180], [0, 180])
        cv2.normalize(roi_hist, roi_hist, 0, 255, cv2.NORM_MINMAX)
        
        
        # dst = cv2.calcBackProject([hsv], [0, 1], ros_hist, [0, 180, 0, 256], 1)
        backproj  = cv2.calcBackProject([hsv], [0], roi_hist,  [0, 180], 1)
        merged = cv2.merge([cl, a, b])
        final = cv2.cvtColor(merged, cv2.COLOR_Lab2BGR)
        dst = cv2.copyTo(final, backproj)
        # 적용된 L 채널과 원래의 A, B 채널을 병합

        # LAB 색 공간에서 BGR 색 공간으로 변환
        
        numpy_horizontal_cat2 = cv2.hconcat((image, final))
        cv2.imshow(windowName2, numpy_horizontal_cat)
        cv2.imshow(windowName3, numpy_horizontal_cat2)
        # image = cv2.resize(frame, (1600, 1200))
        cv2.imshow('dst', dst)        
        # cv2.imshow(windowName2, region_of_interest_image)
        
        # cv2.imshow(windowName3, th3)
        # cv2.imshow("edge", edge)
        # frame_with_lines = cv2.resize(frame_with_lines, (640, 480))
        # cv2.imshow("lines", frame_with_lines)
        
        #pub.publish(imgMsg)
        #cv2.imshow(windowName, frame,interpolation='nearest', aspect='auto')
        #cv2.imshow(windowName, frame)
        rval, frame = vc0.read()
        if rval == False:
            break
                
        # 'ESC' for quit
        key = cv2.waitKey(1)
        if key == 27:
            break
        
    vc0.release()
    cv2.destroyAllWindows()
    
    
def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    match_mask_color = 255
    cv2.fillPoly(mask, vertices, match_mask_color)
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image

def draw_lines(img, lines):
    img = np.copy(img)
    line_image = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
    
    for line in lines:
        for x1, y1, x2, y2 in line:
            cv2.line(line_image, (x1, y1), (x2, y2), (0, 0, 255), 3)
    
    img = weighted_img(line_image, img, 0.8, 1, 0.0)
    return img

def weighted_img(img, initial_img, α=0.8, β=1., λ=0.):
    return cv2.addWeighted(initial_img, α, img, β, λ)
    
if __name__ == "__main__":
    print("-------------------start------------------------")
        # Decode video from file or camera
    decode_video()

    print("-------------------over------------------------")