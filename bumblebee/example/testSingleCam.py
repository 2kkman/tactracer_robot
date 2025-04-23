import cv2
import numpy as np

def decode_video():
    global video_width
    global video_height
    
    # a. Decode video from camera
    #vc0 = cv2.VideoCapture(0)
    vc0 = cv2.VideoCapture(2,cv2.CAP_V4L)
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
    
    

    # # start video decoding. Pass three callbacks at the same time.
    # reader.start_video_mode(parameters, text_results_callback_func, "", intermediate_results_callback_func, error_callback_func, reader)
    while True:
        
        image = cv2.resize(frame, (640, 480))
        # ret,threshold1 = cv2.threshold(image,127,255,cv2.THRESH_TRUNC)
        block_size = 15
        C = 15
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blur = cv2.medianBlur(gray, 3)
        edge = cv2.Canny(blur, 200, 340)
        # ret, th1 = cv2.threshold(image, 0, 255, cv2.THRESH_OTSU)
        # th2 = cv2.adaptiveThreshold(blur,255,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY,block_size,C)
        # th3 = cv2.adaptiveThreshold(blur,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,block_size,C)
        # image_mask = cv2.resize(mask, (640, 480))

        
        # Defined region of interest
        rows, cols = image.shape[:2]
        bottom_left = [cols * 0.1, rows]
        top_left = [cols * 0.4, rows * 0.6]
        bottom_right = [cols * 0.9, rows]
        top_right = [cols * 0.6, rows * 0.6]
        vertices = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)
        
        # Masked image
        # region_of_interest_image = region_of_interest(th2, vertices)
        
        # Hough transform
        lines = cv2.HoughLinesP(edge, 1, np.pi/180, threshold=110, maxLineGap=30)
        
        # lines = cv2.HoughLinesP(edge, 1, np.pi/180, threshold=120, minLineLength=30, maxLineGap=50)
        # cv2.imshow(windowName2, image_mask)
        if lines is not None:
            frame_with_lines = draw_lines(image, lines)
        else:
            frame_with_lines = image
            
        # image = cv2.resize(frame, (1600, 1200))
        cv2.imshow(windowName, image)
        # cv2.imshow(windowName1, th1)        
        # cv2.imshow(windowName2, th2)
        # cv2.imshow(windowName3, th3)
        cv2.imshow("edge", edge)
        # frame_with_lines = cv2.resize(frame_with_lines, (640, 480))
        cv2.imshow("lines", frame_with_lines)
        
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