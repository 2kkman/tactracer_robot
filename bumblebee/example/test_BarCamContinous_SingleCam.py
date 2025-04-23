#! /usr/bin/python3
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rospy
from dbr import *
import cv2
video_width = 0
video_height = 0
bridge = CvBridge()
pub = rospy.Publisher('image', Image, queue_size=10)

def config_accuracy_first(dbr:BarcodeReader):

    # Obtain current runtime settings of instance.
    sts = dbr.get_runtime_settings()
    
    # 1. Set expected barcode format
    # The more precise the barcode format is set, the higher the accuracy.
    # Mostly, misreading only occurs on reading oneD barcode. So here we use OneD barcode format to demonstrate.
    sts.barcode_format_ids = EnumBarcodeFormat.BF_NULL | EnumBarcodeFormat.BF_QR_CODE
    sts.barcode_format_ids_2 = EnumBarcodeFormat.BF_NULL
    
    # 2. Set the minimal result confidence.
    # The greater the confidence, the higher the accuracy.
    # Filter by minimal confidence of the decoded barcode. We recommend using 30 as the default minimal confidence value
    sts.minResultConfidence = 50
    
    # 3. Sets the minimum length of barcode text for filtering.
    # The more precise the barcode text length is set, the higher the accuracy.
    sts.minBarcodeTextLength = 2
    
    # Apply the new settings to the instance
    dbr.update_runtime_settings(sts)

def GetCenterPT():
    global video_width
    global video_height
    return video_width/2 , video_height/2
    
    
def text_results_callback_func(frame_id, t_results, user_data):
        print(frame_id)
        for result in t_results:
            #if text_result.barcode_text.find('03') > 0:
            if True:    
                text_result = TextResult(result)
                print("Barcode Format : ")
                print(text_result.barcode_format_string)
                print("Barcode Text : ")
                print(text_result.barcode_text)
                print("Localization Points : ")
                print(text_result.localization_result.localization_points)
                x = 0
                y= 0
                for resultTP in text_result.localization_result.localization_points:
                    x += resultTP[0]
                    y += resultTP[1]
                avX = x/4
                avY = y/4
                xc,yc = GetCenterPT()
                print(f"위치오차 : {avX-xc},{avY-yc} , 현재위치 : {avX},{avY}")                
                    
                print("Angle : ")
                print(text_result.localization_result.angle)
                print("Exception : ")
                print(text_result.exception)
            else:
                print(f"Invalid Barcd : {text_result.barcode_text}")
            print("-------------")

def intermediate_results_callback_func(frame_id, i_results, user_data):
        print(frame_id)
        for result in i_results:
            intermediate_result = IntermediateResult(result)
            print('Intermediate Result data type : {0}'.format(intermediate_result.result_type))
            print('Intermediate Result data : {0}'.format(intermediate_result.results))
            print("-------------")

def error_callback_func(frame_id, error_code, user_data):
        print(frame_id)
        error_msg = user_data.get_error_string(error_code)
        print('Error : {0} ; {1}'.format(error_code, error_msg))

def decode_video():
    global video_width
    global video_height
    
    # a. Decode video from camera
    #vc0 = cv2.VideoCapture(0)
    vc0 = cv2.VideoCapture(4,cv2.CAP_V4L)

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

    windowName = "Video Barcode Reader"
    reader = BarcodeReader()
    config_accuracy_first(reader)

    parameters = reader.init_frame_decoding_parameters()
    parameters.max_queue_length = 30
    parameters.max_result_queue_length = 30
    parameters.width = video_width
    parameters.height = video_height
    parameters.stride = stride
    parameters.image_pixel_format = EnumImagePixelFormat.IPF_RGB_888
    parameters.region_top = 0
    parameters.region_bottom = 100
    parameters.region_left = 0
    parameters.region_right = 100
    parameters.region_measured_by_percentage = 1
    parameters.threshold = 0.01
    parameters.fps = 0
    parameters.auto_filter = 1

    # start video decoding. The callback function will receive the recognized barcodes.
    reader.start_video_mode(parameters, text_results_callback_func)

    # # start video decoding. Pass three callbacks at the same time.
    # reader.start_video_mode(parameters, text_results_callback_func, "", intermediate_results_callback_func, error_callback_func, reader)
    while True:
        #image = cv2.resize(frame, (640, 480))
        #imgMsg = bridge.cv2_to_imgmsg(frame, "bgr8")
        #cv2.imshow(windowName, image)
        cv2.imshow(windowName, frame)
        
        #pub.publish(imgMsg)
        #cv2.imshow(windowName, frame,interpolation='nearest', aspect='auto')
        rval, frame = vc0.read()
        if rval == False:
            break
        
        try:
            # append frame to video buffer cyclically
            ret = reader.append_video_frame(frame)
            print(ret)
        except:
            pass
        
        # 'ESC' for quit
        key = cv2.waitKey(1)
        if key == 27:
            break

    reader.stop_video_mode()
    cv2.destroyWindow(windowName)

if __name__ == "__main__":
    print("-------------------start------------------------")

    try:
        # Initialize license.
        # The string "DLS2eyJvcmdhbml6YXRpb25JRCI6IjIwMDAwMSJ9" here is a free public trial license. Note that network connection is required for this license to work.
        # You can also request a 30-day trial license in the customer portal: https://www.dynamsoft.com/customer/license/trialLicense?product=dbr&utm_source=samples&package=python
        error = BarcodeReader.init_license("DLS2eyJoYW5kc2hha2VDb2RlIjoiMTAwNzA2NDgxLTEwMTczNzQwOCIsIm1haW5TZXJ2ZXJVUkwiOiJodHRwczovL21kbHMuZHluYW1zb2Z0b25saW5lLmNvbS8iLCJvcmdhbml6YXRpb25JRCI6IjEwMDcwNjQ4MSIsInN0YW5kYnlTZXJ2ZXJVUkwiOiJodHRwczovL3NkbHMuZHluYW1zb2Z0b25saW5lLmNvbS8iLCJjaGVja0NvZGUiOi0xNDQ2MDIyMjc5fQ==")
        if error[0] != EnumErrorCode.DBR_OK:
            print("License error: "+ error[1])

        
        # Decode video from file or camera
        decode_video()

    except BarcodeReaderError as bre:
        print(bre)

    print("-------------------over------------------------")