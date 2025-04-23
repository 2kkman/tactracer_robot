#! /usr/bin/python3
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import message_filters
from tta_blb.srv import *
import cv2
import time
from datetime import datetime
from Util import *


emptyStr = ''
ts = None
count = 0
imgQuality = 100
global reqData
global reqValue
CAM_WIDTH = 3264
CAM_HEIGHT = 2448
#CAM_WIDTH = 10000
#CAM_HEIGHT = 10000
#CAM_WIDTH = 2592
#CAM_HEIGHT = 1944
#CAM_WIDTH = 1920
#CAM_HEIGHT = 1080
capture = None
isRotate = False
# CAM_FOCUS = None
# CAM_AUTO_FOCUS = None

def PrintPropertis():
    rospy.loginfo("CV_CAP_PROP_FRAME_WIDTH: '{}'".format(capture.get(cv2.CAP_PROP_FRAME_WIDTH)))
    rospy.loginfo("CV_CAP_PROP_FRAME_HEIGHT : '{}'".format(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    rospy.loginfo("CAP_PROP_FPS : '{}'".format(capture.get(cv2.CAP_PROP_FPS)))
    rospy.loginfo("CAP_PROP_POS_MSEC : '{}'".format(capture.get(cv2.CAP_PROP_POS_MSEC)))
    rospy.loginfo("CAP_PROP_FRAME_COUNT : '{}'".format(capture.get(cv2.CAP_PROP_FRAME_COUNT)))
    rospy.loginfo("CAP_PROP_BRIGHTNESS : '{}'".format(capture.get(cv2.CAP_PROP_BRIGHTNESS)))
    rospy.loginfo("CAP_PROP_CONTRAST : '{}'".format(capture.get(cv2.CAP_PROP_CONTRAST)))
    rospy.loginfo("CAP_PROP_SATURATION : '{}'".format(capture.get(cv2.CAP_PROP_SATURATION)))
    rospy.loginfo("CAP_PROP_HUE : '{}'".format(capture.get(cv2.CAP_PROP_HUE)))
    rospy.loginfo("CAP_PROP_GAIN : '{}'".format(capture.get(cv2.CAP_PROP_GAIN)))
    rospy.loginfo("CAP_PROP_CONVERT_RGB : '{}'".format(capture.get(cv2.CAP_PROP_CONVERT_RGB)))
    rospy.loginfo("CAP_PROP_FOCUS : '{}'".format(capture.get(cv2.CAP_PROP_FOCUS)))
    rospy.loginfo("CAP_PROP_AUTOFOCUS : '{}'".format(capture.get(cv2.CAP_PROP_AUTOFOCUS)))
    rospy.loginfo("CAP_PROP_EXPOSURE : '{}'".format(capture.get(cv2.CAP_PROP_EXPOSURE)))

class SaveImage():
    def __init__(self):
        global capture
        cv2.destroyAllWindows()
        while True:
            #capture = cv2.VideoCapture(0,cv2.CAP_DSHOW)
            capture = cv2.VideoCapture(0)
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if capture.isOpened():
                # frame_no = capture.grab()
                # frame_no = capture.grab()
                # frame_no = capture.grab()
                break
            else:
                if capture != None:
                    capture.release()
                time.sleep(1)

        self.set_saveImage_service = rospy.Service('/utilbox/saveImage', utilboxCam, self.saveImage)
        self.set_saveImage_service2 = rospy.Service('/utilbox/saveImageFocus', utilboxCam, self.saveImageManualFocus)
        self.set_saveImage_service3 = rospy.Service('/utilbox/saveImageTest', utilboxCam, self.saveImageTest)
        self.set_saveImage_service4 = rospy.Service('/utilbox/CAM_RESET', Kill, self.CamRelease)
        
        self.topics = rospy.get_param('/tta_blb/utilbox/saveImageTopics', ['/usb_cam_R/image_raw', '/usb_cam_L/image_raw'])
        self.save_Directory = rospy.set_param('/tta_blb/utilbox/saveImageDirectory', '/root/SpiderGo/')
        self.save_Prefix = rospy.set_param('/tta_blb/utilbox/saveFilenamePrefix', '0000')
        self.save_Suffix = rospy.set_param('/tta_blb/utilbox/saveFilenameSuffix', ['_R.jpg','_L.jpg'])
        self.save_Value = rospy.set_param('/tta_blb/utilbox/reqValue', 0)
        self.save_flag = rospy.set_param('/tta_blb/utilbox/saveFlag', True)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT,CAM_HEIGHT)
        capture.set(cv2.CAP_PROP_AUTOFOCUS,1)
        capture.set(cv2.CAP_PROP_EXPOSURE,156)
        
        # CAM_FOCUS = capture.get(cv2.CAP_PROP_FOCUS)
        # CAM_AUTO_FOCUS = capture.get(cv2.CAP_PROP_AUTOFOCUS)
        # rospy.loginfo(f'CAP_PROP_FOCUS : {CAM_FOCUS}')
        # rospy.loginfo(f'CAM_AUTO_FOCUS : {CAM_AUTO_FOCUS}')
        PrintPropertis()
        # rospy.loginfo(f'CAP_PROP_FOCUS : {capture.get(cv2.focu)}')
        # rospy.loginfo(f'CAP_PROP_FOCUS : {capture.get(cv2.CAP_PROP_FOCUS)}')
        # rospy.loginfo(f'CAP_PROP_FOCUS : {capture.get(cv2.CAP_PROP_FOCUS)}')
        # rospy.loginfo(f'CAP_PROP_FOCUS : {capture.get(cv2.CAP_PROP_FOCUS)}')
        
        
        # self.loop_rate = rospy.Rate(1)

        if not self.topics:
            rospy.logerr('No Topics Specified.')
        frame= capture.read()
        time.sleep(1)
        frame= capture.read()
        time.sleep(1)
        frame= capture.read()
        rospy.loginfo('SPG Image Sever Started')

    def CamRelease(self, req):
        # release window
        capture.release()
        cv2.destroyAllWindows()
        #print(10 / 0)

    def RemoveDummy():
        # release window
        capture.release()
        cv2.destroyAllWindows()

    def saveImageTest(self, req):
        reqData = req.message
        reqValue = req.value
        capture.set(cv2.CAP_PROP_AUTOFOCUS,1.0)
        #capture.set(cv2.CAP_PROP_FOCUS,0.0)        

        # rospy.loginfo("reqData: {}, reqValue: {}".format(reqData, reqValue))
        rospy.set_param('/tta_blb/utilbox/saveFilenamePrefix', reqData)
        rospy.set_param('/tta_blb/utilbox/reqValue', reqValue)
        rospy.set_param('/tta_blb/utilbox/saveFlag', True)
        result = False
        try:
            capCnt = 0
            result, frame= capture.read()
            while True:
                result, frame= capture.read()
                if result:
                    cntPad = (str)(capCnt+1).rjust(2,'0')
                    if isRotate:
                        img180 = cv2.rotate(frame, cv2.ROTATE_180)
                    else:
                        img180 = frame
                    emptyStr = ''
                    #reqDataList = str(reqData).split(sep='_')
                    #sFolderSector = f'{reqDataList[0]}_{reqDataList[1]}'
                    sFolderName = f'/root/SpiderGo/'
                    os.makedirs(sFolderName, exist_ok=True)
                    #추후 IMU값에 따라 자동뒤집는 옵션을 구현한다.
                    #img180 = cv2.rotate(frame, cv2.ROTATE_180)
                    #sFileName = f'/root/SpiderGo/{reqData}_{capture.get(cv2.CAP_PROP_FOCUS)}_{(int)(getDateTime().timestamp())}_{cntPad}.jpg'
                    sFileName = f'{sFolderName}/{reqData}_{(int)(getDateTime().timestamp())}_{capture.get(cv2.CAP_PROP_FOCUS)}_{cntPad}.jpg'
                    #sFileName = f'{sFolderName}/{reqData}_{cntPad}.jpg'
                    rospy.loginfo(f'Write to : {sFileName}')
                    cv2.imwrite(sFileName,img180, params=[cv2.IMWRITE_JPEG_QUALITY,imgQuality])
                    #cv2.imwrite(f'/root/SpiderGo/{reqData}_{(int)(datetime.now().timestamp())}_{cntPad}.png',img180)
                    rospy.loginfo(f'saveImageTest Result : {result}, Cnt : {capCnt}, Focus : {capture.get(cv2.CAP_PROP_FOCUS)}, Distance : {capture.get(cv2.CAP_PROP_XI_LENS_FOCUS_DISTANCE)}')
                    capCnt += 1
                
                PrintPropertis()
                
                if capCnt == reqValue:                    
                    break
            
            # imageL = message_filters.Subscriber("/usb_cam_L/image_raw", Image)
            # imageR = message_filters.Subscriber("/usb_cam_R/image_raw", Image)

        except CvBridgeError as e:
            print(e)
            
        # Synchronize images
        #ts = message_filters.ApproximateTimeSynchronizer([imageL, imageR], queue_size=10, slop=0.5)
        resp = utilboxCamResponse(result, 'Stoped saveImageTest')       
        # if ts.registerCallback(self.imageCallback) is not None:
        #     resp = utilboxCamResponse(True, 'Started saveImage ')
        # else:
        #     resp = utilboxCamResponse(False, 'Stoped saveImage')
                    
        return resp
    
    def saveImage(self, req):
        reqData = req.message
        reqValue = req.value
        capture.set(cv2.CAP_PROP_AUTOFOCUS,1.0)
        #capture.set(cv2.CAP_PROP_FOCUS,0.0)        

        # rospy.loginfo("reqData: {}, reqValue: {}".format(reqData, reqValue))
        # rospy.set_param('/tta_blb/utilbox/saveFilenamePrefix', reqData)
        # rospy.set_param('/tta_blb/utilbox/reqValue', reqValue)
        # rospy.set_param('/tta_blb/utilbox/saveFlag', True)
        result = False
        try:
            capCnt = 0
            # while True:
            #     result, frame= capture.read()
            #     if result:
            #         break
            #frame_no = capture.grab()
            #frame_no = capture.grab()
            
            frame= capture.read()
            while True:
                result, frame= capture.read()
                if result:
                    cntPad = (str)(capCnt+1).rjust(2,'0')
                    if isRotate:
                        img180 = cv2.rotate(frame, cv2.ROTATE_180)
                    else:
                        img180 = frame
                    reqDataList = str(reqData).split(sep='_')
                    sFolderSector = f'{reqDataList[0]}_{reqDataList[1]}'
                    sFolderName = f'/root/SpiderGo/{getCurrentDate(emptyStr)}/{sFolderSector}'
                    os.makedirs(sFolderName, exist_ok=True)
                    #추후 IMU값에 따라 자동뒤집는 옵션을 구현한다.
                    #img180 = cv2.rotate(frame, cv2.ROTATE_180)
                    #sFileName = f'/root/SpiderGo/{reqData}_{capture.get(cv2.CAP_PROP_FOCUS)}_{(int)(getDateTime().timestamp())}_{cntPad}.jpg'
                    #sFileName = f'{sFolderName}/{reqData}_{capture.get(cv2.CAP_PROP_FOCUS)}_{(int)(datetime.now().timestamp())}_{cntPad}.jpg'
                    sFileName = f'{sFolderName}/{reqData}_{cntPad}.jpg'
                    rospy.loginfo(f'Write to : {sFileName}')
                    cv2.imwrite(sFileName,img180, params=[cv2.IMWRITE_JPEG_QUALITY,imgQuality])
                    #cv2.imwrite(f'/root/SpiderGo/{reqData}_{(int)(datetime.now().timestamp())}_{cntPad}.png',img180)
                    rospy.loginfo(f'Cam Result : {result}, Cnt : {capCnt}, Focus : {capture.get(cv2.CAP_PROP_FOCUS)}, Distance : {capture.get(cv2.CAP_PROP_XI_LENS_FOCUS_DISTANCE)}')
                    capCnt += 1
                
                PrintPropertis()
                
                if capCnt == reqValue:                    
                    break
            
            # imageL = message_filters.Subscriber("/usb_cam_L/image_raw", Image)
            # imageR = message_filters.Subscriber("/usb_cam_R/image_raw", Image)

        except CvBridgeError as e:
            print(e)
            
        # Synchronize images
        #ts = message_filters.ApproximateTimeSynchronizer([imageL, imageR], queue_size=10, slop=0.5)
        resp = utilboxCamResponse(result, 'Stoped saveImage')       
        # if ts.registerCallback(self.imageCallback) is not None:
        #     resp = utilboxCamResponse(True, 'Started saveImage ')
        # else:
        #     resp = utilboxCamResponse(False, 'Stoped saveImage')
                    
        return resp

    def saveImageManualFocus(self, req):
        reqData = req.message
        reqValue = req.value
        capture.set(cv2.CAP_PROP_AUTOFOCUS,0)
        capture.set(cv2.CAP_PROP_FOCUS, reqValue)        
        # rospy.loginfo("reqData: {}, reqValue: {}".format(reqData, reqValue))
        #rospy.set_param('/tta_blb/utilbox/saveFilenamePrefix', reqData)
        #rospy.set_param('/tta_blb/utilbox/reqValue', reqValue)
        #rospy.set_param('/tta_blb/utilbox/saveFlag', True)
        result = False
        try:
            capCnt = 0
            # while True:
            #     result, frame= capture.read()
            #     if result:
            #         break
            #frame_no = capture.grab()
            #frame_no = capture.grab()
            frame= capture.read()
            #time.sleep(1)
            while True:
                result, frame= capture.read()
                if result:
                    cntPad = (str)(capCnt+1).rjust(2,'0')
                    if isRotate:
                        img180 = cv2.rotate(frame, cv2.ROTATE_180)
                    else:
                        img180 = frame
                    
                    reqDataList = str(reqData).split(sep='_')
                    sFolderSector = f'{reqDataList[0]}_{reqDataList[1]}'
                    sFolderName = f'/root/SpiderGo/{getCurrentDate(emptyStr)}/{sFolderSector}'
                    os.makedirs(sFolderName, exist_ok=True)                        
                    #추후 IMU값에 따라 자동뒤집는 옵션을 구현한다.
                    #img180 = cv2.rotate(frame, cv2.ROTATE_180)
                    sFileName = f'{sFolderName}/{reqData}_{cntPad}.jpg'
                    rospy.loginfo(f'Write to : {sFileName}')
                    cv2.imwrite(sFileName,img180, params=[cv2.IMWRITE_JPEG_QUALITY,imgQuality])
                    rospy.loginfo(f'Cam Result : {result}, Cnt : {capCnt}, Focus : {capture.get(cv2.CAP_PROP_FOCUS)}, Distance : {capture.get(cv2.CAP_PROP_XI_LENS_FOCUS_DISTANCE)}')
                    #cv2.imwrite(f'/root/SpiderGo/{reqData}_{capture.get(cv2.CAP_PROP_FOCUS)}_{(int)(getDateTime().timestamp())}_{cntPad}.jpg',img180, params=[cv2.IMWRITE_JPEG_QUALITY,100])
                    
                    capCnt += 1
                rospy.loginfo(f'saveImageManualFocus Result : {result}, Cnt : {capCnt}, Focus : {capture.get(cv2.CAP_PROP_FOCUS)}, Distance : {capture.get(cv2.CAP_PROP_XI_LENS_FOCUS_DISTANCE)}')
                PrintPropertis()
                if capCnt == 1:                    
                    break
            
            # imageL = message_filters.Subscriber("/usb_cam_L/image_raw", Image)
            # imageR = message_filters.Subscriber("/usb_cam_R/image_raw", Image)

        except CvBridgeError as e:
            print(e)
            
        # Synchronize images
        #ts = message_filters.ApproximateTimeSynchronizer([imageL, imageR], queue_size=10, slop=0.5)
        resp = utilboxCamResponse(result, 'Stoped saveImageManualFocus')
        # if ts.registerCallback(self.imageCallback) is not None:
        #     resp = utilboxCamResponse(True, 'Started saveImage ')
        # else:
        #     resp = utilboxCamResponse(False, 'Stoped saveImage')
                    
        return resp

    def imageCallback(self, imageL, imageR):

        saveDirectory = str(rospy.get_param('/tta_blb/utilbox/saveImageDirectory'))
        savePrefix = str(rospy.get_param('/tta_blb/utilbox/saveFilenamePrefix'))
        saveFlag = rospy.get_param('/tta_blb/utilbox/saveFlag')
        saveValue = rospy.get_param('/tta_blb/utilbox/reqValue')

        try:
            br = CvBridge()
            imageLeft = br.imgmsg_to_cv2(imageL)
            imageRight = br.imgmsg_to_cv2(imageR)
        except CvBridgeError as e:
            print(e)
        if (saveFlag):
            timestamp = datetime.now().strftime('%Y-%m-%d_%H%M%S.%f')[:-3]
            saveImagePath = saveDirectory + savePrefix + '_' + timestamp

            if saveValue == 0:
                # rospy.loginfo("end saving image")
                rospy.set_param('/tta_blb/utilbox/saveFlag', False)
                return

            # rospy.loginfo("receiving frame")
            imageLeft = cv2.cvtColor(imageLeft, cv2.COLOR_BGR2RGB)
            imageRight = cv2.cvtColor(imageRight, cv2.COLOR_BGR2RGB)
            # rospy.loginfo("saving frame")
            isWriteL = cv2.imwrite(saveImagePath + '_L.jpg', imageLeft)
            isWriteR = cv2.imwrite(saveImagePath + '_R.jpg', imageRight)
            
            count = saveValue - 1
            rospy.set_param('/tta_blb/utilbox/reqValue', count)

            if isWriteL and isWriteR:
                pass
                rospy.loginfo("saved image {} {}".format(saveImagePath + '_L.jpg', saveImagePath + '_R.jpg'))

        
if __name__ == '__main__':
    rospy.init_node('spg_ImageSaver', anonymous=True, log_level=rospy.INFO)
    try:
        SaveImage()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass