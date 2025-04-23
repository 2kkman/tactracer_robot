#!/usr/bin/env python3
import rospy
import rosnode
from std_srvs.srv import *
from rospy_tutorials.srv import *
from sensor_msgs.msg import Image
import os
import subprocess
from varname import *
from dbr import *
import cv2
import time
from Util import *
from UtilGPIO import *
from SPG_Keys import *
#from ServoNano import *
from varname import *
from cv_bridge import CvBridge
from std_msgs.msg import *
from UtilBLB import *

runFromLaunch = rospy.get_param("~", default=False)
bScanBarcd = False
bCamPublish = False
bCamSaveImg = False
MaskingCam = 7
camBin = bin(MaskingCam)[2:]
cam_count = 3
dicQRResult = {}
publish_topic_name = TopicName.ARUCO_RESULT.name
pub_ka = rospy.Publisher(publish_topic_name, String, queue_size=1)

def prtMsg(sendbuf):
    prtbuf = f'{sys._getframe(1).f_code.co_name}:{sendbuf}'
    if runFromLaunch:
        rospy.loginfo(prtbuf)
    else:
        print(prtbuf)

def SendKeepAlive(sendbuf):
    global pub_ka
    if pub_ka is not None:
        pub_ka.publish(sendbuf)
        prtMsg(sendbuf)



class CamService():
    def __init__(self):
        self.BAR_SET_service = rospy.Service(ServiceBLB.MarkerScan.value, AddTwoInts, self.QR_Enable)
        self.IMG_SET_service = rospy.Service(ServiceBLB.IMG_PUBLISH.value, SetBool, self.CAM_ENABLE)
        prtMsg(f'{CamService.__name__} Service is Started')
        
    def CAM_ENABLE(self, req):
        """카메라 On/Off

        Args:
            req (_type_): True or False

        Returns:
            _type_: 정상 True, 예외발생시 False
        """
        global bCamPublish
        bReturn = True
        try:
            bCamPublish = req.data
            prtMsg({sys._getframe(0).f_code.co_name})
        except Exception as e:
            bPass = False
            prtMsg(traceback.format_exc())
        returnMsg = f'{sys._getframe(0).f_code.co_name}:{bCamPublish}'
        return SetBoolResponse(bReturn, returnMsg)

    def QR_Enable(self, req):
        """카메라가 On 되어있을때만 호출.
        QR코드 파싱할 카메라를 지정, On/Off

        Args:
            req (_type_): req.a 는 제어할 카메라 넘버의 이진수 합. req.b 는 1일 경우 On, 0이면 Off
            

        Returns:
            _type_: _description_
        """
        global bScanBarcd
        global camBin
        reqNumber = req.a
        bScanBarcd = isTrue(req.b)
        camBin = bin(reqNumber)[2:].zfill(3)[::-1]
        resp = AddTwoIntsResponse()
        resp.sum = 1
        return resp

video_width = 0
video_height = 0
lsResolution = []
lsImagePublishers = []

video_width = 0
video_height = 0
lsResolution = []
def count_cameras():
    global lsResolution
    lsAlive = []
    max_tested = 100
    camFinalIdx = -1
    for i in range(max_tested):
        temp_camera = cv2.VideoCapture(i,cv2.CAP_V4L)
        if temp_camera.isOpened():
            temp_camera.set(cv2.CAP_PROP_FRAME_WIDTH, 10000) #set width
            temp_camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 10000) #set height
            temp_camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))    
            rval, frame = temp_camera.read()
            stride = frame.strides[0]
            curHeight = temp_camera.get(cv2.CAP_PROP_FRAME_HEIGHT)
            curWidth = temp_camera.get(cv2.CAP_PROP_FRAME_WIDTH)            
            print(f'Camera {i} is {rval}, W:{curWidth},H:{curHeight},Stride:{stride}')
            curWH = [curWidth,curHeight]
            lsResolution.append(curWH)
            camFinalIdx = i
            temp_camera.release()
            lsAlive.append(i)
            #continue
    return lsAlive

lsAlives =count_cameras()
print(lsAlives)
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
    
    
# def text_results_callback_func(frame_id, t_results, user_data):
#         #print(frame_id,user_data)
#         print(frame_id,user_data)
#         for result in t_results:
#             #if text_result.barcode_text.find('03') > 0:
#             dicTmp = {}
#             if True:    
                
#                 text_result = TextResult(result)
#                 print("Barcode Format : ")
#                 print(text_result.barcode_format_string)
#                 print("Barcode Text : ")
#                 print(text_result.barcode_text)
#                 print("Localization Points : ")
#                 print(text_result.localization_result.localization_points)
#                 x = 0
#                 y= 0
#                 for resultTP in text_result.localization_result.localization_points:
#                     x += resultTP[0]
#                     y += resultTP[1]
#                 avX = x/4
#                 avY = y/4
#                 xc,yc = GetCenterPT()
#                 print(f"위치오차 : {avX-xc},{avY-yc} , 현재위치 : {avX},{avY}")                
                    
#                 print("Angle : ")
#                 print(text_result.localization_result.angle)
#                 print("Exception : ")
#                 print(text_result.exception)
#                 print(f'Data : {text_result}')
#             else:
#                 print(f"Invalid Barcd : {text_result.barcode_text}")
#             print("-------------")

def intermediate_results_callback_func(frame_id, i_results, user_data):
        #print(frame_id)
        for result in i_results:
            intermediate_result = IntermediateResult(result)
            print('Intermediate Result frame and data type : {0}-{1}'.format(frame_id,intermediate_result.result_type))
            print('Intermediate Result data : {0}'.format(intermediate_result.results))
            print("-------------")

def error_callback_func(frame_id, error_code, user_data):
        #print(frame_id)
        error_msg = user_data.get_error_string(error_code)
        print('Error Frame : {0} ; {1}-{2}'.format(frame_id,error_code, error_msg))

def print_cv_info(vc0):
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
    
InstanceBarcodeReader = [None,None,None]
InstanceVC = [None,None,None]

def decode_video():
    global video_width
    global video_height
    global cam_count
    #camList = [0,2,4]
    camList = lsAlives
    cam_count = len(lsAlives)    
    camListResolution = lsResolution
    #camListResolution = [(4656,3496),(3264,2448),(3840,2880)]

    for camIdx in range(cam_count):
        # a. Decode video from camera
        #vc0 = cv2.VideoCapture(0)
        InstanceVC[camIdx] = cv2.VideoCapture(camList[camIdx],cv2.CAP_V4L)
        
        # # b. Decode video file
        # video_file = "Put your video file path here."
        # vc = cv2.VideoCapture(video_file)
        video_width  = camListResolution[camIdx][0]
        video_height = camListResolution[camIdx][1]
        #video_width  = int(vc.get(cv2.CAP_PROP_FRAME_WIDTH))
        #video_height = int(vc.get(cv2.CAP_PROP_FRAME_HEIGHT))
        #vc.set(cv2.CAP_PROP_FRAME_WIDTH, 3840) #set width
        #vc.set(cv2.CAP_PROP_FRAME_HEIGHT, 2880) #set height
        InstanceVC[camIdx].set(cv2.CAP_PROP_AUTOFOCUS,1)
        #vc.set(cv2.CAP_PROP_EXPOSURE,156)
        InstanceVC[camIdx].set(cv2.CAP_PROP_FRAME_WIDTH, video_width) #set width
        InstanceVC[camIdx].set(cv2.CAP_PROP_FRAME_HEIGHT, video_height) #set height
        InstanceVC[camIdx].set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))    

        stride = 0
        if InstanceVC[camIdx].isOpened():  
            rval, frame = InstanceVC[camIdx].read()
            stride = frame.strides[0]
        else:
            return

        windowName = str(camList[camIdx])
        #reader0 = BarcodeReader()
        InstanceBarcodeReader[camIdx] = BarcodeReader()
        config_accuracy_first(InstanceBarcodeReader[camIdx])

        parameters = InstanceBarcodeReader[camIdx].init_frame_decoding_parameters()
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
        InstanceBarcodeReader[camIdx].start_video_mode(parameters, text_results_callback_func, intermediate_result_callback_func=intermediate_results_callback_func, error_callback_func=error_callback_func, unique_barcode_callback_func=None, user_data=camList[camIdx])
        # # start video decoding. Pass three callbacks at the same time.
        # reader.start_video_mode(parameters, text_results_callback_func, "", intermediate_results_callback_func, error_callback_func, reader)

    # while True:
    #     #cv2.imshow(windowName, frame,interpolation='nearest', aspect='auto')        
    #     for camIdx in range(cam_count):
    #         windowName = str(camList[camIdx])
    #         rval, frame = InstanceVC[camIdx].read()
    #         if rval == False:
    #             break
    #         image = cv2.resize(frame, (640, 480))
    #         cv2.imshow(windowName, image)            
    #         try:
    #             # append frame to video buffer cyclically
    #             ret = InstanceBarcodeReader[camIdx].append_video_frame(frame)
    #         except:
    #             pass
        
    #     # 'ESC' for quit
    #     key = cv2.waitKey(1)
    #     if key == 27:
    #         break

    # for camIdx in range(cam_count):
    #     InstanceBarcodeReader[camIdx].stop_video_mode()
    #     cv2.destroyWindow(str(camList[camIdx]))


# if __name__ == "__main__":

#     print("-------------------start------------------------")

#     try:
#         # Initialize license.
#         # The string "DLS2eyJvcmdhbml6YXRpb25JRCI6IjIwMDAwMSJ9" here is a free public trial license. Note that network connection is required for this license to work.
#         # You can also request a 30-day trial license in the customer portal: https://www.dynamsoft.com/customer/license/trialLicense?product=dbr&utm_source=samples&package=python
#         error = BarcodeReader.init_license("DLS2eyJoYW5kc2hha2VDb2RlIjoiMTAwNzA2NDgxLTEwMTczNzQwOCIsIm1haW5TZXJ2ZXJVUkwiOiJodHRwczovL21kbHMuZHluYW1zb2Z0b25saW5lLmNvbS8iLCJvcmdhbml6YXRpb25JRCI6IjEwMDcwNjQ4MSIsInN0YW5kYnlTZXJ2ZXJVUkwiOiJodHRwczovL3NkbHMuZHluYW1zb2Z0b25saW5lLmNvbS8iLCJjaGVja0NvZGUiOi0xNDQ2MDIyMjc5fQ==")
#         if error[0] != EnumErrorCode.DBR_OK:
#             print("License error: "+ error[1])

        
#         # Decode video from file or camera
#         decode_video()

#     except BarcodeReaderError as bre:
#         print(bre)

#     print("-------------------over------------------------")
    
# def count_cameras():
#     global lsResolution
#     lsAlive = []
#     max_tested = 100
#     camFinalIdx = -1
#     for i in range(max_tested):
#         temp_camera = cv2.VideoCapture(i,cv2.CAP_V4L)
#         if temp_camera.isOpened():
#             temp_camera.set(cv2.CAP_PROP_FRAME_WIDTH, 10000) #set width
#             temp_camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 10000) #set height
#             temp_camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))    
#             rval, frame = temp_camera.read()
#             stride = frame.strides[0]
#             curHeight = temp_camera.get(cv2.CAP_PROP_FRAME_HEIGHT)
#             curWidth = temp_camera.get(cv2.CAP_PROP_FRAME_WIDTH)            
#             prtMsg(f'Camera {i} is {rval}, W:{curWidth},H:{curHeight},Stride:{stride}')
#             curWH = [curWidth,curHeight]
#             lsResolution.append(curWH)
#             pub_img = rospy.Publisher(f'CAM_{i}', Image, queue_size=10)
#             lsImagePublishers.append(pub_img)
#             camFinalIdx = i
#             temp_camera.release()
#             lsAlive.append(i)
#     return lsAlive

# lsAlives =count_cameras()
# prtMsg(lsAlives)
# def config_accuracy_first(dbr:BarcodeReader):
#     sts = dbr.get_runtime_settings()
#     sts.barcode_format_ids = EnumBarcodeFormat.BF_NULL | EnumBarcodeFormat.BF_QR_CODE
#     sts.barcode_format_ids_2 = EnumBarcodeFormat.BF_NULL
#     sts.minResultConfidence = 50
#     sts.minBarcodeTextLength = 2
#     dbr.update_runtime_settings(sts)

def GetCenterPT(frame_id):
    global lsResolution
    try:
        video_width_cur = lsResolution[frame_id][0]
        video_height_cur = lsResolution[frame_id][1]
        return video_width_cur/2 , video_height_cur/2
    except Exception as e:
        prtMsg(traceback.format_exc())
        return -1,-1
    
def text_results_callback_func(frame_id, t_results, user_data):
        #print(frame_id,user_data)
        prtMsg(frame_id,user_data)
        for result in t_results:
            dicTmp = {}
            #if text_result.barcode_text.find('03') > 0:
            if True:    
                text_result = TextResult(result)
                prtMsg("Barcode Format : ")
                dicTmp["FORMAT"] = text_result.barcode_format_string
                prtMsg(text_result.barcode_format_string)
                prtMsg("Barcode Text : ")
                dicTmp["TEXT"]=text_result.barcode_text
                prtMsg(text_result.barcode_text)
                prtMsg("Localization Points : ")
                prtMsg(text_result.localization_result.localization_points)
                x = 0
                y= 0
                for resultTP in text_result.localization_result.localization_points:
                    x += resultTP[0]
                    y += resultTP[1]
                avX = x/4
                avY = y/4
                xc,yc = GetCenterPT(user_data)
                dicTmp['LOC_X']=xc
                dicTmp['LOC_Y']=yc
                
                prtMsg(f"위치오차 : {avX-xc},{avY-yc} , 현재위치 : {avX},{avY}")                
                    
                prtMsg("Angle : ")
                dicTmp['ANGLE']=text_result.localization_result.angle
                prtMsg(text_result.localization_result.angle)
                prtMsg("Exception : ")
                dicTmp['EXCEPTION']=text_result.exception
                prtMsg(text_result.exception)
                dicTmp['DATA']=text_result
                prtMsg(f'Data : {text_result}')
                #strTmp = getStr_fromDicPrivate(dicTmp,sDivFieldColon,sDivEmart)
                strTmp = json.dumps(dicTmp)
                SendKeepAlive(strTmp)
            else:
                prtMsg(f"Invalid Barcd : {text_result.barcode_text}")
            prtMsg("-------------")

# def intermediate_results_callback_func(frame_id, i_results, user_data):
#         #print(frame_id)
#         for result in i_results:
#             intermediate_result = IntermediateResult(result)
#             prtMsg('Intermediate Result frame and data type : {0}-{1}'.format(frame_id,intermediate_result.result_type))
#             prtMsg('Intermediate Result data : {0}'.format(intermediate_result.results))
#             prtMsg("-------------")

# def error_callback_func(frame_id, error_code, user_data):
#         #prtMsg(frame_id)
#         error_msg = user_data.get_error_string(error_code)
#         prtMsg('Error Frame : {0} ; {1}-{2}'.format(frame_id,error_code, error_msg))

# def print_cv_info(vc0):
#     prtMsg("CV_CAP_PROP_FRAME_WIDTH: '{}'".format(vc0.get(cv2.CAP_PROP_FRAME_WIDTH)))
#     prtMsg("CV_CAP_PROP_FRAME_HEIGHT : '{}'".format(vc0.get(cv2.CAP_PROP_FRAME_HEIGHT)))
#     prtMsg("CAP_PROP_FPS : '{}'".format(vc0.get(cv2.CAP_PROP_FPS)))
#     prtMsg("CAP_PROP_POS_MSEC : '{}'".format(vc0.get(cv2.CAP_PROP_POS_MSEC)))
#     prtMsg("CAP_PROP_FRAME_COUNT : '{}'".format(vc0.get(cv2.CAP_PROP_FRAME_COUNT)))
#     prtMsg("CAP_PROP_BRIGHTNESS : '{}'".format(vc0.get(cv2.CAP_PROP_BRIGHTNESS)))
#     prtMsg("CAP_PROP_CONTRAST : '{}'".format(vc0.get(cv2.CAP_PROP_CONTRAST)))
#     prtMsg("CAP_PROP_SATURATION : '{}'".format(vc0.get(cv2.CAP_PROP_SATURATION)))
#     prtMsg("CAP_PROP_HUE : '{}'".format(vc0.get(cv2.CAP_PROP_HUE)))
#     prtMsg("CAP_PROP_GAIN : '{}'".format(vc0.get(cv2.CAP_PROP_GAIN)))
#     prtMsg("CAP_PROP_CONVERT_RGB : '{}'".format(vc0.get(cv2.CAP_PROP_CONVERT_RGB)))
#     prtMsg("CAP_PROP_FOCUS : '{}'".format(vc0.get(cv2.CAP_PROP_FOCUS)))
#     prtMsg("CAP_PROP_AUTOFOCUS : '{}'".format(vc0.get(cv2.CAP_PROP_AUTOFOCUS)))
#     prtMsg("CAP_PROP_EXPOSURE : '{}'".format(vc0.get(cv2.CAP_PROP_EXPOSURE)))
#     prtMsg("CAP_PROP_FOURCC : '{}'".format(vc0.get(cv2.CAP_PROP_FOURCC)))
    
# InstanceBarcodeReader = [None,None,None]
# InstanceVC = [None,None,None]
# cam_count = 3
# def decode_video():
#     global video_width
#     global video_height
#     #global InstanceBarcodeReader
#     #global InstanceVC
    
#     #camList = [0,2,4]
#     camList = lsAlives
#     camListResolution = lsResolution
#     #camListResolution = [(4656,3496),(3264,2448),(3840,2880)]

#     for camIdx in range(cam_count):
#         InstanceVC[camIdx] = cv2.VideoCapture(camList[camIdx],cv2.CAP_V4L)
#         video_width  = camListResolution[camIdx][0]
#         video_height = camListResolution[camIdx][1]
#         InstanceVC[camIdx].set(cv2.CAP_PROP_AUTOFOCUS,1)
#         InstanceVC[camIdx].set(cv2.CAP_PROP_FRAME_WIDTH, video_width) #set width
#         InstanceVC[camIdx].set(cv2.CAP_PROP_FRAME_HEIGHT, video_height) #set height
#         InstanceVC[camIdx].set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))    

#         stride = 0
#         if InstanceVC[camIdx].isOpened():  
#             rval, frame = InstanceVC[camIdx].read()
#             stride = frame.strides[0]
#         else:
#             return

#         #windowName = str(camList[camIdx])
#         #reader0 = BarcodeReader()
#         InstanceBarcodeReader[camIdx] = BarcodeReader()
#         config_accuracy_first(InstanceBarcodeReader[camIdx])

#         parameters = InstanceBarcodeReader[camIdx].init_frame_decoding_parameters()
#         parameters.max_queue_length = 30
#         parameters.max_result_queue_length = 30
#         parameters.width = video_width
#         parameters.height = video_height
#         parameters.stride = stride
#         parameters.image_pixel_format = EnumImagePixelFormat.IPF_RGB_888
#         parameters.region_top = 0
#         parameters.region_bottom = 100
#         parameters.region_left = 0
#         parameters.region_right = 100
#         parameters.region_measured_by_percentage = 1
#         parameters.threshold = 0.01
#         parameters.fps = 0
#         parameters.auto_filter = 1
        
#         # start video decoding. The callback function will receive the recognized barcodes.
#         InstanceBarcodeReader[camIdx].start_video_mode(parameters, text_results_callback_func, intermediate_result_callback_func=intermediate_results_callback_func, error_callback_func=error_callback_func, unique_barcode_callback_func=None, user_data=camIdx)
#         # # start video decoding. Pass three callbacks at the same time.
#         # reader.start_video_mode(parameters, text_results_callback_func, "", intermediate_results_callback_func, error_callback_func, reader)

#     # while True:
#     #     #cv2.imshow(windowName, frame,interpolation='nearest', aspect='auto')        
#     #     for camIdx in range(cam_count):
#     #         windowName = str(camList[camIdx])
#     #         rval, frame = InstanceVC[camIdx].read()
#     #         if rval == False:
#     #             break
#     #         image = cv2.resize(frame, (640, 480))
#     #         cv2.imshow(windowName, image)            
#     #         try:
#     #             # append frame to video buffer cyclically
#     #             ret = InstanceBarcodeReader[camIdx].append_video_frame(frame)
#     #             prtMsg(ret)
#     #         except:
#     #             pass
        
#     #     # 'ESC' for quit
#     #     key = cv2.waitKey(1)
#     #     if key == 27:
#     #         break

#     # for camIdx in range(cam_count):
#     #     InstanceBarcodeReader[camIdx].stop_video_mode()
#     #     cv2.destroyWindow(str(camList[camIdx]))


#if __name__ == "__main__":
rospy.init_node(f'node_{publish_topic_name}',anonymous=False) #노드 생성
rate = rospy.Rate(10)
CamService()
imgQuality = 80
bBreak = False
sFolderName = f'/root/BB/'
os.makedirs(sFolderName, exist_ok=True)
bridge = CvBridge()
lastLogTime = getDateTime()

#rospy.Subscriber(source_GPIO_topic_name, String, callbackGPIOTopic, queue_size=1) # LaserScan 토픽이 오면 콜백 함수가 호출되도록 세팅
prtMsg("-------------------start------------------------")

# Initialize license.
# The string "DLS2eyJvcmdhbml6YXRpb25JRCI6IjIwMDAwMSJ9" here is a free public trial license. Note that network connection is required for this license to work.
# You can also request a 30-day trial license in the customer portal: https://www.dynamsoft.com/customer/license/trialLicense?product=dbr&utm_source=samples&package=python
error = BarcodeReader.init_license("DLS2eyJoYW5kc2hha2VDb2RlIjoiMTAwNzA2NDgxLTEwMTczNzQwOCIsIm1haW5TZXJ2ZXJVUkwiOiJodHRwczovL21kbHMuZHluYW1zb2Z0b25saW5lLmNvbS8iLCJvcmdhbml6YXRpb25JRCI6IjEwMDcwNjQ4MSIsInN0YW5kYnlTZXJ2ZXJVUkwiOiJodHRwczovL3NkbHMuZHluYW1zb2Z0b25saW5lLmNvbS8iLCJjaGVja0NvZGUiOi0xNDQ2MDIyMjc5fQ==")
if error[0] != EnumErrorCode.DBR_OK:
    prtMsg("License error: "+ error[1])
else:
    prtMsg(f'License OK: {error}')

decode_video()
while not rospy.is_shutdown() and not bBreak:
    bReturn = True
    if bScanBarcd or bCamPublish or bCamSaveImg:
        try:
            for camIdx in range(cam_count):
                if isTrue(camBin[camIdx]):
                    windowName = str(lsAlives[camIdx])
                    rval, frame = InstanceVC[camIdx].read()
                    if rval == False:
                        break
                    if bCamPublish:
                        #fra = bridge.cv2_to_imgmsg(frame,encoding="passthrough")
                        fra = bridge.cv2_to_imgmsg(frame,encoding='rgb8')
                        lsImagePublishers[camIdx].publish(fra)
                    if bCamSaveImg:
                        sFileName = f'{sFolderName}/{publish_topic_name}_{(int)(getDateTime().timestamp())}_{InstanceVC[camIdx].get(cv2.CAP_PROP_FOCUS)}.jpg'
                        prtMsg(f'Write to : {sFileName}')
                        cv2.imwrite(sFileName,frame, params=[cv2.IMWRITE_JPEG_QUALITY,imgQuality])
                        prtMsg(f'saveImageTest Result : {rval}, Focus : {InstanceVC[camIdx].get(cv2.CAP_PROP_FOCUS)}, Distance : {InstanceVC[camIdx].get(cv2.CAP_PROP_XI_LENS_FOCUS_DISTANCE)}')

                    if not runFromLaunch:
                        image = cv2.resize(frame, (640, 480))
                        cv2.imshow(windowName, image)
                    try:
                        # append frame to video buffer cyclically
                        if bScanBarcd:
                            ret = InstanceBarcodeReader[camIdx].append_video_frame(frame)
                            if isTimeExceeded(lastLogTime, 1000):
                                prtMsg(ret)
                                lastLogTime = getDateTime()
                    except:
                        pass
            # 'ESC' for quit
            key = cv2.waitKey(1)
            if key == 27:
                bBreak = True
                break

        except Exception as e:
            bReturn = False
            prtMsg(traceback.format_exc())
            rospy.signal_shutdown(e)
            #sCmd = '/root/.rrStart -&'
            #os.system(sCmd)
    rate.sleep()

        
for camIdx in range(cam_count):
    strCam = str(lsAlives[camIdx])
    try:
        prtMsg(f'Trying to stop_video_mode : {strCam}')
        InstanceBarcodeReader[camIdx].stop_video_mode()
        cv2.destroyWindow(strCam)
    except Exception as e:
        prtMsg(traceback.format_exc())