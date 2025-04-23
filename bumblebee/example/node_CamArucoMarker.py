#!/usr/bin/env python3
import rospy
import rosnode
from std_srvs.srv import *
from rospy_tutorials.srv import *
from sensor_msgs.msg import Image
import os
import subprocess
from varname import *
from UtilAruco import *
import cv2
import time
from Util import *
from UtilBLB import *
from varname import *
from cv_bridge import CvBridge
from std_msgs.msg import *
from sensor_msgs.msg import CompressedImage

def resize_maintain_aspect(frame, target_width=800):
    height, width = frame.shape[:2]
    aspect_ratio = width / height
    new_height = int(target_width / aspect_ratio)
    resized = cv2.resize(frame, (target_width, new_height))
    return resized
  
runFromLaunch = rospy.get_param("~startReal", default=False)
bScanBarcd = False
bCamPublish = True
bCamSaveImg = False
MaskingCam = 7
camBin = bin(MaskingCam)[2:]
cam_count = 1
dicQRResult = {}
publish_topic_name = TopicName.ARUCO_RESULT.name
pub_ka = rospy.Publisher(publish_topic_name, String, queue_size=1)
dirPath = os.path.dirname(__file__)
arucoSizeMeter = 0.063
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
arucoParams = cv2.aruco.DetectorParameters()
machineName = GetMachineStr()
dirPath2 = getConfigPath(machineName)
k={}
d={}
idx = 0
lastUpdateTimeStamp = getDateTime()
frameDelay = 1000
img_resize_width = 1024
meter_to_milimeter = 1000
"""
v4l2-ctl -d /dev/video2 --list-formats-ext
ioctl: VIDIOC_ENUM_FMT
        Type: Video Capture

        [0]: 'MJPG' (Motion-JPEG, compressed)
                Size: Discrete 3840x2160
                        Interval: Discrete 0.033s (30.000 fps)
                Size: Discrete 1920x1080
                        Interval: Discrete 0.033s (30.000 fps)
                Size: Discrete 1280x720
                        Interval: Discrete 0.033s (30.000 fps)
                Size: Discrete 4000x3000
                        Interval: Discrete 0.067s (15.000 fps)
                Size: Discrete 8000x6000
                        Interval: Discrete 0.200s (5.000 fps)
"""


def prtMsg(sendbuf):
    prtbuf = f"{sys._getframe(1).f_code.co_name}:{sendbuf}"
    if runFromLaunch:
        rospy.loginfo(prtbuf)
    else:
        print(prtbuf)


def SendKeepAlive(sendbuf):
    global pub_ka
    if pub_ka is not None:
        pub_ka.publish(sendbuf)
        prtMsg(sendbuf)


class CamService:
    def __init__(self):
        self.BAR_SET_service = rospy.Service(
            ServiceBLB.MarkerScan.value, SetBool, self.Marker_Enable
        )
        self.IMG_SET_service = rospy.Service(
            ServiceBLB.IMG_PUBLISH.value, SetBool, self.CAM_ENABLE
        )
        prtMsg(f"{CamService.__name__} Service is Started")

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
        returnMsg = f"{sys._getframe(0).f_code.co_name}:{bCamPublish}"
        return SetBoolResponse(bReturn, returnMsg)

    def Marker_Enable(self, req):
        """카메라가 On 되어있을때만 호출.
        마커코드 파싱할 카메라를 지정, On/Off

        Args:
            req (_type_): req.a 는 제어할 카메라 넘버의 이진수 합. req.b 는 1일 경우 On, 0이면 Off


        Returns:
            _type_: _description_
        """
        global bScanBarcd
        global camBin
        reqNumber = 3
        bScanBarcd = req.data
        bReturn = True
        camBin = bin(reqNumber)[2:].zfill(3)[::-1]
        returnMsg = f"{sys._getframe(0).f_code.co_name}:{bScanBarcd}"
        SendKeepAliveOnly(1 if bScanBarcd else 0)
        prtMsg(returnMsg)
        return SetBoolResponse(bReturn, returnMsg)

    def Marker_Enable_Old(self, req):
        """카메라가 On 되어있을때만 호출.
        마커코드 파싱할 카메라를 지정, On/Off

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


video_width_preset = 1920
video_height_preset = 1080
lsImagePublishers = []
video_width = 0
video_height = 0
lsResolution = []


def count_cameras():
    global lsResolution
    lsAlive = []
    max_tested = 1
    camFinalIdx = -1
    for i in range(max_tested):
        temp_camera = cv2.VideoCapture(i, cv2.CAP_V4L)
        if temp_camera.isOpened():
            temp_camera.set(cv2.CAP_PROP_FRAME_WIDTH, video_width_preset)  # set width
            temp_camera.set(
                cv2.CAP_PROP_FRAME_HEIGHT, video_height_preset
            )  # set height
            temp_camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            rval, frame = temp_camera.read()
            stride = frame.strides[0]
            curHeight = temp_camera.get(cv2.CAP_PROP_FRAME_HEIGHT)
            curWidth = temp_camera.get(cv2.CAP_PROP_FRAME_WIDTH)
            print(f"Camera {i} is {rval}, W:{curWidth},H:{curHeight},Stride:{stride}")
            curWH = [curWidth, curHeight]
            lsResolution.append(curWH)
            camFinalIdx = i
            temp_camera.release()
            lsAlive.append(i)
            # continue
    return lsAlive


lsAlives = count_cameras()
print(lsAlives)


def GetCenterPT():
    global video_width
    global video_height
    return video_width / 2, video_height / 2


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


InstanceVC = [None, None, None]


def decode_video():
    global video_width
    global video_height
    global cam_count
    global d,k
    # camList = [0,2,4]
    camList = lsAlives
    cam_count = len(lsAlives)
    camListResolution = lsResolution
    
    # camListResolution = [(4656,3496),(3264,2448),(3840,2880)]

    for camIdx in range(cam_count):
        InstanceVC[camIdx] = cv2.VideoCapture(camList[camIdx], cv2.CAP_V4L)
        video_width = camListResolution[camIdx][0]
        video_height = camListResolution[camIdx][1]
        InstanceVC[camIdx].set(cv2.CAP_PROP_FPS, 15)
        InstanceVC[camIdx].set(cv2.CAP_PROP_AUTOFOCUS, 1)
        InstanceVC[camIdx].set(cv2.CAP_PROP_FRAME_WIDTH, video_width)  # set width
        InstanceVC[camIdx].set(cv2.CAP_PROP_FRAME_HEIGHT, video_height)  # set height
        InstanceVC[camIdx].set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        img_publish_name = f"{ServiceBLB.IMG_PUBLISH.value}{camList[camIdx]}"
        lsImagePublishers.append(
            rospy.Publisher(img_publish_name, CompressedImage, queue_size=1)
        )
        camID = lsAlives[camIdx]
        calibration = f"calibration_matrix_{camID}.npy"
        distortion = f"distortion_coefficients_{camID}.npy"
        calibration_matrix_path = f"{dirPath2}/{calibration}"
        distortion_coefficients_path = f"{dirPath2}/{distortion}"
        print(calibration_matrix_path)
        k[camID] = np.load(calibration_matrix_path)
        d[camID] = np.load(distortion_coefficients_path)


        # stride = 0
        # if InstanceVC[camIdx].isOpened():
        #     rval, frame = InstanceVC[camIdx].read()
        #     if frame is not None:
        #       stride = frame.strides[0]
        # else:
        #     continue


def GetCenterPT(frame_id):
    global lsResolution
    try:
        video_width_cur = lsResolution[frame_id][0]
        video_height_cur = lsResolution[frame_id][1]
        return video_width_cur / 2, video_height_cur / 2
    except Exception as e:
        prtMsg(traceback.format_exc())
        return -1, -1


currentMachine = GetMachineStr()
rospy.init_node(f"{currentMachine}_{publish_topic_name}", anonymous=False)  # 노드 생성
rate = rospy.Rate(10)
instanceCam = CamService()
imgQuality = 80
bBreak = False
sFolderName = f"/root/BB/"
os.makedirs(sFolderName, exist_ok=True)
bridge = CvBridge()
lastLogTime = getDateTime()

# rospy.Subscriber(source_GPIO_topic_name, String, callbackGPIOTopic, queue_size=1) # LaserScan 토픽이 오면 콜백 함수가 호출되도록 세팅
prtMsg("-------------------start with compressed img ------------------------")

decode_video()


def SendKeepAliveOnly(windowName):
    dicKeepAlive = {}
    dicKeepAlive[ARUCO_RESULT_FIELD.CAM_ID.name] = int(windowName)
    dicKeepAlive[ARUCO_RESULT_FIELD.IS_MARKERSCAN.name] = bScanBarcd
    dicKeepAlive[ARUCO_RESULT_FIELD.IS_IMGPUBLISH.name] = bCamPublish
    sendbuf = json.dumps(dicKeepAlive)
    SendKeepAlive(sendbuf)
    return sendbuf

# SetBoolRequest 객체 생성
request = SetBoolRequest(data=True)
instanceCam.Marker_Enable(request)
instanceCam.CAM_ENABLE(request)

while not rospy.is_shutdown() and not bBreak:
    # print(arucoParams.adaptiveThreshConstant)
    bReturn = True
    td = getDateTime() - lastUpdateTimeStamp
    isKeepAlive = td.total_seconds() >= 2
    
    if bScanBarcd or bCamPublish or bCamSaveImg or isKeepAlive:
        try:
            for camIdx in range(cam_count):
                if isTrue(camBin[camIdx]):
                    windowNameInt = lsAlives[camIdx]
                    windowName = str(windowNameInt)
                    rval, frameRaw = InstanceVC[camIdx].read()
                    frame = cv2.rotate(frameRaw, cv2.ROTATE_180)
                    # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    idx += 1
                    if rval == False:
                        break
                    if bCamPublish and isTimeExceeded(lastUpdateTimeStamp,frameDelay):# and camIdx == 0:
                        # fra = bridge.cv2_to_imgmsg(frame,encoding="passthrough")
                        fra2 = resize_maintain_aspect(frame, img_resize_width)
                        #fra = bridge.cv2_to_imgmsg(image, encoding="rgb8")
                        #fra = bridge.cv2_to_imgmsg(fra2, encoding="rgb8")
                        msg = CompressedImage()
                        msg.header.stamp = rospy.Time.now()
                        msg.format = "jpeg"
                        #msg.data = np.array(cv2.imencode('.jpg', fra2)[1]).tobytes()
                        msg.data = np.array(cv2.imencode('.jpg', fra2, [int(cv2.IMWRITE_JPEG_QUALITY), 50])[1]).tobytes()

                        lsImagePublishers[camIdx].publish(msg)
                        lastUpdateTimeStamp = getDateTime()
                    try:
                        # append frame to video buffer cyclically
                        if bScanBarcd:
                            corners, ids, rejected = cv2.aruco.detectMarkers(
                                frame, arucoDict, parameters=arucoParams
                            )
                            if ids is not None:
                                # Estimate pose of each marker
                                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                                    corners, arucoSizeMeter, k[windowNameInt], d[windowNameInt]
                                )
                            else:
                                continue
                            # print(ids)
                            # Iterate over detected markers
                            for i in range(len(ids)):
                                # Draw axis for each marker
                                # cv2.aruco.drawAxis(frame, k, d, rvecs[i], tvecs[i], 0.03)
                                frame = cv2.drawFrameAxes(
                                    frame, k[windowNameInt], d[windowNameInt], rvecs[i], tvecs[i], arucoSizeMeter
                                )
                                degr = Vector2Degree(
                                    rvecs[i]
                                )  # Z값이 마커 Rotate 각도가 됨
                                # CAMID,X,Y,Z,ANGLE,MARKER_VALUE 를 topic 으로 발행하여 전송.
                                diff_x, diff_y = calculate_offset(
                                    video_width_preset,
                                    video_height_preset,
                                    corners[i][0],
                                )
                                dicResult = {}
                                np_numbers = np.array(tvecs)

                                x_distance = round(
                                    np_numbers[i][0][0] * meter_to_milimeter
                                )
                                y_distance = round(
                                    np_numbers[i][0][1] * meter_to_milimeter*-1
                                )
                                z_distance = round(
                                    np_numbers[i][0][2] * meter_to_milimeter
                                )
                                rounded_numbers = np.round(np_numbers, 4)

                                # if z_distance > 10:
                                #     continue

                                dicResult[ARUCO_RESULT_FIELD.DIFF_X.name] = np.round(
                                    diff_x, 2
                                )
                                dicResult[ARUCO_RESULT_FIELD.DIFF_Y.name] = -np.round(
                                    diff_y, 2
                                )
                                dicResult[ARUCO_RESULT_FIELD.ANGLE.name] = round(
                                    # convert_angle(degr[2])
                                    degr[2]
                                )
                                dicResult[ARUCO_RESULT_FIELD.CAM_ID.name] = int(
                                    windowName
                                )
                                dicResult[ARUCO_RESULT_FIELD.MARKER_VALUE.name] = int(
                                    ids[i][0]
                                )
                                # if dicResult[ARUCO_RESULT_FIELD.MARKER_VALUE.name] > 9:
                                #     cv2.imwrite(
                                #         f"{sFolderName}{dicResult[ARUCO_RESULT_FIELD.MARKER_VALUE.name]}-{windowName}-{getCurrentTime()}",
                                #         frame,
                                #     )

                                dicResult[ARUCO_RESULT_FIELD.X.name] = x_distance
                                dicResult[ARUCO_RESULT_FIELD.Y.name] = y_distance
                                #2번카메라 위치보정
                                if windowName == '2':
                                  dicResult[ARUCO_RESULT_FIELD.X.name] = x_distance +35
                                  dicResult[ARUCO_RESULT_FIELD.Y.name] = y_distance -30
                                  #y_distance -= 30                                                                  
                                
                                dicResult[ARUCO_RESULT_FIELD.Z.name] = z_distance
                                # distanceServingTeleTotal, angle_degrees = calculate_distance_and_angle(x_distance, y_distance)    
                                # dicResult[ARUCO_RESULT_FIELD.DISTANCE.name] = distanceServingTeleTotal
                                # dicResult[ARUCO_RESULT_FIELD.DISTANCE_ANGLE.name] = angle_degrees
                                
                                # print(dicResult)
                                # dic = {}
                                # # NumPy int32를 Python 기본 int로 변환
                                # for key, value in dicResult.items():
                                #   if isinstance(value, np.int32):
                                #       dic[key] = int(value)
                                sendbuf = json.dumps(dicResult)
                                SendKeepAlive(sendbuf)
                                lastLogTime = getDateTime()

                            # Draw marker borders
                            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                            # if not runFromLaunch:
                            #     #image = cv2.resize(frame, (853, 480))
                            #     image = resize_maintain_aspect(frame)
                            #     cv2.imshow(windowName, image)
                            #     key = cv2.waitKey(1) & 0xFF
                            #     if key == ord("q"):
                            #       break
                        elif isKeepAlive:
                            SendKeepAliveOnly(windowName)
                        else:
                            print("카메라 동작중이 아닌경우")
                            print(video_width, video_height)

                    except Exception as e:
                        message = traceback.format_exc()
                        print(message)
                        pass
            # 'ESC' or 'q' for quit
            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord("q"):
                bBreak = True
                break

        except Exception as e:
            bReturn = False
            prtMsg(traceback.format_exc())
            rospy.signal_shutdown(e)
            # sCmd = '/root/.rrStart -&'
            # os.system(sCmd)
    rate.sleep()

for camIdx in range(cam_count):
    strCam = str(lsAlives[camIdx])
    try:
        prtMsg(f"Trying to stop_video_mode : {strCam}")
        InstanceVC[camIdx].release()
        if cv2.getWindowProperty(strCam, cv2.WND_PROP_VISIBLE) >= 1:
            cv2.destroyWindow(strCam)
    except Exception as e:
        prtMsg(traceback.format_exc())
