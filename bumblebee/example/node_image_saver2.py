#! /usr/bin/python3
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import message_filters
from tta_blb.srv import *
import cv2
import time
from datetime import datetime


ts = None
count = 0
global reqData
global reqValue


class SaveImage():
    def __init__(self):

        self.set_saveImage_service = rospy.Service('/utilbox/saveImage', utilboxCam, self.saveImage)
        self.topics = rospy.get_param('/tta_blb/utilbox/saveImageTopics', ['/spg_cam/image_raw'])
        # self.topics = rospy.get_param('/tta_blb/utilbox/saveImageTopics', ['/usb_cam_R/image_raw', '/usb_cam_L/image_raw'])
        self.save_Directory = rospy.set_param('/tta_blb/utilbox/saveImageDirectory', '/root/SpiderGo/')
        self.save_Prefix = rospy.set_param('/tta_blb/utilbox/saveFilenamePrefix', '0000')
        self.save_Suffix = rospy.set_param('/tta_blb/utilbox/saveFilenameSuffix', ['_R.jpg','_L.jpg'])
        self.save_Value = rospy.set_param('/tta_blb/utilbox/reqValue', 0)
        self.save_flag = rospy.set_param('/tta_blb/utilbox/saveFlag', True)
        
        # self.loop_rate = rospy.Rate(1)

        if not self.topics:
            rospy.logerr('No Topics Specified.')

        rospy.loginfo('SPG Image Sever Started')

    def saveImage(self, req):

        reqData = req.message
        reqValue = req.value

        # rospy.loginfo("reqData: {}, reqValue: {}".format(reqData, reqValue))
        rospy.set_param('/tta_blb/utilbox/saveFilenamePrefix', reqData)
        rospy.set_param('/tta_blb/utilbox/reqValue', reqValue)
        rospy.set_param('/tta_blb/utilbox/saveFlag', True)

        try:
            image = rospy.Subscriber("/spg_cam/image_raw", Image, self.imageCallback)
            # imageL = message_filters.Subscriber("/usb_cam_L/image_raw", Image)
            #imageR = message_filters.Subscriber("/usb_cam_R/image_raw", Image)

        except CvBridgeError as e:
            print(e)
            
        # Synchronize images
        #ts = message_filters.ApproximateTimeSynchronizer([imageL, imageR], queue_size=10, slop=0.5)
        # ts = message_filters.ApproximateTimeSynchronizer([image], queue_size=10, slop=0.5)
               
        if (self.imageCallback) is not None:
            resp = utilboxCamResponse(True, 'Started saveImage ')
        else:
            resp = utilboxCamResponse(False, 'Stoped saveImage')
                    
        return resp

    def imageCallback(self, image):

        saveDirectory = str(rospy.get_param('/tta_blb/utilbox/saveImageDirectory'))
        savePrefix = str(rospy.get_param('/tta_blb/utilbox/saveFilenamePrefix'))
        saveFlag = rospy.get_param('/tta_blb/utilbox/saveFlag')
        saveValue = rospy.get_param('/tta_blb/utilbox/reqValue')

        try:
            br = CvBridge()
            image = br.imgmsg_to_cv2(image)
            
            # imageLeft = br.imgmsg_to_cv2(imageL)
            #imageRight = br.imgmsg_to_cv2(imageR)
        except CvBridgeError as e:
            print(e)
        if (saveFlag):
            # timestamp = datetime.now().strftime('%Y-%m-%d_%H%M%S.%f')[:-3]
            timestamp = datetime.now().strftime('%Y-%m-%d_%H%M%S.%f')[:-3]
            
            saveImagePath = saveDirectory + savePrefix + '_' + timestamp
            # rospy.loginfo(saveImagePath)

            if saveValue == 0:
                rospy.set_param('/tta_blb/utilbox/saveFlag', False)
                return

            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            isWrite = cv2.imwrite(saveImagePath + '.jpg', image, params=[cv2.IMWRITE_JPEG_QUALITY,100])
            
            count = saveValue - 1
            rospy.set_param('/tta_blb/utilbox/reqValue', count)

            if isWrite:
                pass
                rospy.loginfo("saved image[{}] {}".format(saveValue, saveImagePath + '.jpg'))

        
if __name__ == '__main__':
    rospy.init_node('spg_ImageSaver', anonymous=True, log_level=rospy.INFO)
    try:
        SaveImage()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass