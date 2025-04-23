from flask import Blueprint, request, jsonify
from flask.views import MethodView
from ..ros.publishers import RosPublisher
from ..ros.subscribers import RosSubscriber
from ..ros.service_clients import RosServiceClient
from ..utils import *
import inspect
from flask_socketio import emit
from app import socketio
seq = 1
jsonGlobal = ""
pan_blueprint = Blueprint("pan", __name__)
ros_publisher = RosPublisher()
ros_subscriber = RosSubscriber()
ros_service_client = RosServiceClient()

@socketio.on('connect')
def handle_connect():
    emit('server_message', {'text': 'Connected to WebSocket server'})

@socketio.on('custom_event')
def handle_custom_event(data):
    print(f"Received data: {data}")
    emit('response', {'text': 'Message received'})

ros = roslibpy.Ros(host=IP_MASTER, port=9090)
if not ros.is_connected:
  ros.run()
  
robot_status_reply_keeptime = 2
def handle_request_common(info,req=None, cn = 'NONE'):
    #return GetActionReponse(APIBLB_ACTION_REPLY.E104)
    try:
        logmsg = f"{info} from {cn} - {sys._getframe(0).f_code.co_name}"
        rospy.loginfo(logmsg)
        #log_all_frames()
        #print(info)
        rpResult = ros_service_client.call_service(info)
        print(rpResult)
        bOK = rpResult.success 
        bMsg = rpResult.message
        dict_object = json.loads(bMsg)
    except Exception as e:
        message = traceback.format_exc()
        rospy.loginfo(message)
        return GetExceptionReponse(req)    
    #ros_publisher.publish_message(info)
    #return GetActionReponseOK(request)
    return GetFlaskReponse(dict_object,req)
 
def handle_request_QBIARD(info,req=None, cn = 'NONE'):
    #return GetActionReponse(APIBLB_ACTION_REPLY.E104)
    try:
        isRealMachine = get_hostname().find(UbuntuEnv.ITX.name) >= 0
        logmsg = f"{info} from {cn} - {sys._getframe(0).f_code.co_name}"
        rospy.loginfo(logmsg)
        #log_all_frames()
        #print(info)
        if isRealMachine:
            rpResult = ros_service_client.call_service_CMDARD_QBI(info)
        else:
            rpResult = ros_service_client.call_service_CMDARD_ITX(info)
        # #print(rpResult)
        # bOK = rpResult.success 
        # bMsg = rpResult.message
        dict_object = json.loads(rpResult)
    except Exception as e:
        message = traceback.format_exc()
        rospy.loginfo(message)
        return GetExceptionReponse(req)    
    #ros_publisher.publish_message(info)
    #return GetActionReponseOK(request)
    return GetFlaskReponse(dict_object,req)
 
        # print(request.args.to_dict())
        # return GetFlaskReponse(request.args,None)
class robot_CurrTask_info(MethodView):
    def handle_request(self):
        # 요청 메서드 출력
        print(f"Request method: {request.method}")

    def get(self):
        self.handle_request()
        print(type(request.args))
        
        dataReturn= request.args.to_dict()
        print(f"GET params: {dataReturn}")
        info = json.dumps(dataReturn)
        ros_publisher.publish_message(info)
        return GetFlaskReponse(dataReturn,request)
        #return handle_request_common(APIBLB_FIELDS_ACTION.resume.name,request,cn=self.__class__.__name__)

class robot_TaskChain_List(MethodView):
    def handle_request(self):
        # 요청 메서드 출력
        print(f"Request method: {request.method}")

    def get(self):
        self.handle_request()
        print(type(request.args))
        
        dataReturn= request.args.to_dict()
        info = json.dumps(dataReturn)
        ros_publisher.publish_message(info)
        lsDic = dataReturn['chainlist']
        df = pd.DataFrame(lsDic)
        df.to_csv(strCSV_TaskChain, index=False, sep=sDivTab) 
        df = df[pd.to_numeric(df[APIBLB_FIELDS_TASK.tasktype.name], errors="coerce").notna()]
        print(f"GET params: {lsDic}")
        return GetFlaskReponse(dataReturn,request)
        #return handle_request_common(APIBLB_FIELDS_ACTION.resume.name,request,cn=self.__class__.__name__)

    def post(self):
        self.handle_request()
        if request.is_json:
            print(f"POST JSON data: {request.json}")
            return GetFlaskReponse(request.json,request)
        else:
            dataReturn= request.form.to_dict()
            df = pd.DataFrame(dataReturn)
            df = df[pd.to_numeric(df[APIBLB_FIELDS_TASK.tasktype.name], errors="coerce").notna()]
            df.to_csv(strCSV_TaskChain, index=False, sep=sDivTab) 
            #print(f"GET params: {dataReturn}")
            info = json.dumps(dataReturn)
            
            print(f"POST form data: {dataReturn}")
            return GetFlaskReponse(request.form.to_dict(),request)
    
class robot_CurrTask_info2(MethodView):
    def get(self):
        #dataReturn ={'target' : [{"mastercode":"241010024758773","detailcode":"241010024758783","simulayoutcd":"1001","startnode":"1","endnode":"2","direction":"N","distance":"5000","nodetype":"HR","tasktype":"1","ordertype":"1","orderstatus":"1","workstatus":"1","comments":"","iconxval":"0.00","iconyval":"0.00","linexval":"0.00","lineyval":"0.00"},{"mastercode":"241010024758773","detailcode":"241010024758805","simulayoutcd":"1001","startnode":"2","endnode":"3","direction":"N","distance":"4000","nodetype":"R","tasktype":"1","ordertype":"1","orderstatus":"1","workstatus":"1","comments":"","iconxval":"0.00","iconyval":"0.00","linexval":"0.00","lineyval":"0.00"},{"mastercode":"241010024758773","detailcode":"241010024758808","simulayoutcd":"1001","startnode":"3","endnode":"4","direction":"S","distance":"9500","nodetype":"N","tasktype":"1","ordertype":"1","orderstatus":"1","workstatus":"1","comments":"","iconxval":"0.00","iconyval":"0.00","linexval":"0.00","lineyval":"0.00"},{"mastercode":"241010024758773","detailcode":"241010024758812","simulayoutcd":"1001","startnode":"4","endnode":"5","direction":"N","distance":"5300","nodetype":"NR","tasktype":"1","ordertype":"1","orderstatus":"1","workstatus":"1","comments":"","iconxval":"0.00","iconyval":"0.00","linexval":"0.00","lineyval":"0.00"},{"mastercode":"241010024758773","detailcode":"241010024758815","simulayoutcd":"1001","startnode":"5","endnode":"11","direction":"W","distance":"5700","nodetype":"R","tasktype":"1","ordertype":"1","orderstatus":"1","workstatus":"1","comments":"","iconxval":"0.00","iconyval":"0.00","linexval":"0.00","lineyval":"0.00"},{"mastercode":"241010024758773","detailcode":"241010024758818","simulayoutcd":"1001","startnode":"11","endnode":"T16","direction":"S","distance":"990.00","nodetype":"","tasktype":"1","ordertype":"1","orderstatus":"1","workstatus":"1","comments":"","iconxval":"0.00","iconyval":"0.00","linexval":"0.00","lineyval":"0.00"}] }
        dataReturn={'target': [{"mastercode":"241010033315357","detailcode":"241010033315367","simulayoutcd":"1001","startnode":"9","endnode":"8","direction":"W","distance":"7500","nodetype":"N","tasktype":"1","ordertype":"1","orderstatus":"1","workstatus":"1","comments":"","iconxval":"0.00","iconyval":"0.00","linexval":"0.00","lineyval":"0.00"},{"mastercode":"241010033315357","detailcode":"241010033315382","simulayoutcd":"1001","startnode":"8","endnode":"7","direction":"W","distance":"5500","nodetype":"N","tasktype":"1","ordertype":"1","orderstatus":"1","workstatus":"1","comments":"","iconxval":"0.00","iconyval":"0.00","linexval":"0.00","lineyval":"0.00"},{"mastercode":"241010033315357","detailcode":"241010033315386","simulayoutcd":"1001","startnode":"7","endnode":"6","direction":"W","distance":"7000","nodetype":"N","tasktype":"1","ordertype":"1","orderstatus":"1","workstatus":"1","comments":"","iconxval":"0.00","iconyval":"0.00","linexval":"0.00","lineyval":"0.00"},{"mastercode":"241010033315357","detailcode":"241010033315389","simulayoutcd":"1001","startnode":"6","endnode":"2","direction":"W","distance":"8000","nodetype":"NR","tasktype":"1","ordertype":"1","orderstatus":"1","workstatus":"1","comments":"","iconxval":"0.00","iconyval":"0.00","linexval":"0.00","lineyval":"0.00"},{"mastercode":"241010033315357","detailcode":"241010033315394","simulayoutcd":"1001","startnode":"2","endnode":"1","direction":"N","distance":"5000","nodetype":"R","tasktype":"1","ordertype":"1","orderstatus":"1","workstatus":"1","comments":"","iconxval":"0.00","iconyval":"0.00","linexval":"0.00","lineyval":"0.00"},{"mastercode":"241010033315357","detailcode":"241010033315398","simulayoutcd":"1001","startnode":"1","endnode":"H1","direction":"E","distance":"5900","nodetype":"N","tasktype":"1","ordertype":"1","orderstatus":"1","workstatus":"1","comments":"","iconxval":"0.00","iconyval":"0.00","linexval":"0.00","lineyval":"0.00"}]} #{"mastercode":"241010033315357","detailcode":"241010033315401","simulayoutcd":"1001","startnode":"12","endnode":"13","direction":"E","distance":"3500","nodetype":"N","tasktype":"1","ordertype":"1","orderstatus":"1","workstatus":"1","comments":"","iconxval":"0.00","iconyval":"0.00","linexval":"0.00","lineyval":"0.00"},{"mastercode":"241010033315357","detailcode":"241010033315405","simulayoutcd":"1001","startnode":"13","endnode":"T23","direction":"S","distance":"808.00","nodetype":"","tasktype":"1","ordertype":"1","orderstatus":"1","workstatus":"1","comments":"","iconxval":"0.00","iconyval":"0.00","linexval":"0.00","lineyval":"0.00"}
        #print(dataReturn)
        info = json.dumps(dataReturn)
        ros_publisher.publish_message(info)
        #return handle_request_common(info,request)
        return jsonify(dataReturn)

    def post(self):
        dataReturn = [{"mastercode":"241010033315357","detailcode":"241010033315367","simulayoutcd":"1001","startnode":"1","endnode":"2","direction":"S","distance":"5000","nodetype":"HR","tasktype":"1","ordertype":"1","orderstatus":"1","workstatus":"1","comments":"","iconxval":"0.00","iconyval":"0.00","linexval":"0.00","lineyval":"0.00"},{"mastercode":"241010033315357","detailcode":"241010033315382","simulayoutcd":"1001","startnode":"2","endnode":"3","direction":"S","distance":"4000","nodetype":"R","tasktype":"1","ordertype":"1","orderstatus":"1","workstatus":"1","comments":"","iconxval":"0.00","iconyval":"0.00","linexval":"0.00","lineyval":"0.00"},{"mastercode":"241010033315357","detailcode":"241010033315386","simulayoutcd":"1001","startnode":"3","endnode":"4","direction":"S","distance":"9500","nodetype":"N","tasktype":"1","ordertype":"1","orderstatus":"1","workstatus":"1","comments":"","iconxval":"0.00","iconyval":"0.00","linexval":"0.00","lineyval":"0.00"},{"mastercode":"241010033315357","detailcode":"241010033315389","simulayoutcd":"1001","startnode":"4","endnode":"5","direction":"S","distance":"5300","nodetype":"NR","tasktype":"1","ordertype":"1","orderstatus":"1","workstatus":"1","comments":"","iconxval":"0.00","iconyval":"0.00","linexval":"0.00","lineyval":"0.00"},{"mastercode":"241010033315357","detailcode":"241010033315394","simulayoutcd":"1001","startnode":"5","endnode":"11","direction":"E","distance":"5700","nodetype":"R","tasktype":"1","ordertype":"1","orderstatus":"1","workstatus":"1","comments":"","iconxval":"0.00","iconyval":"0.00","linexval":"0.00","lineyval":"0.00"},{"mastercode":"241010033315357","detailcode":"241010033315398","simulayoutcd":"1001","startnode":"11","endnode":"12","direction":"E","distance":"5900","nodetype":"N","tasktype":"1","ordertype":"1","orderstatus":"1","workstatus":"1","comments":"","iconxval":"0.00","iconyval":"0.00","linexval":"0.00","lineyval":"0.00"},{"mastercode":"241010033315357","detailcode":"241010033315401","simulayoutcd":"1001","startnode":"12","endnode":"13","direction":"E","distance":"3500","nodetype":"N","tasktype":"1","ordertype":"1","orderstatus":"1","workstatus":"1","comments":"","iconxval":"0.00","iconyval":"0.00","linexval":"0.00","lineyval":"0.00"},{"mastercode":"241010033315357","detailcode":"241010033315405","simulayoutcd":"1001","startnode":"13","endnode":"T23","direction":"S","distance":"808.00","nodetype":"","tasktype":"1","ordertype":"1","orderstatus":"1","workstatus":"1","comments":"","iconxval":"0.00","iconyval":"0.00","linexval":"0.00","lineyval":"0.00"}]
        return jsonify(dataReturn)
        
class robot_alarm_status(MethodView):
    def handle_request(self):
        iAlmCD = int(dictGlobal.get(APIBLB_FIELDS_STATUS.error_code.name,-1))
        iAlmNM = dictGlobal.get(APIBLB_FIELDS_STATUS.alarm_msg.name,"")
        resultJS = {}
        resultJS2 = {}
        returnls = []
        if iAlmCD > 10:
          resultJS[APIBLB_FIELDS_STATUS.error_code.name] = f'E{str(iAlmCD).zfill(5)}'
          resultJS[APIBLB_FIELDS_STATUS.alarm_msg.name] = iAlmNM
          resultJS2[APIBLB_FIELDS_STATUS.error_code.name] = f'E50004'
          resultJS2[APIBLB_FIELDS_STATUS.alarm_msg.name] = '10,Motor Overload'
        
        if len(resultJS) > 0:
          returnls.append(resultJS)
        if len(resultJS2) > 0:
          returnls.append(resultJS2)
        return GetFlaskReponse(returnls,request=request)
    def get(self):
        return self.handle_request()
    def post(self):
        return self.handle_request()
      
class robot_expand_action(MethodView):
    def handle_request(self):
        tmpHalf = request.args.get(APIBLB_FIELDS_ACTION.half.name, type=int)
        tmpFull = request.args.get(APIBLB_FIELDS_ACTION.full.name, type=int)
        tmpFold = request.args.get(APIBLB_FIELDS_ACTION.fold.name, type=int)
        # non_zero_count = sum(1 for x in [tmpFold,tmpFull,tmpHalf] if x == None or x != 0)
        # if non_zero_count >= 2:
        #     return GetActionReponse(APIBLB_ACTION_REPLY.E104)
        cmd_expand = '0,0,0'    
        if isTrue(tmpFull):
            cmd_expand = '1800,0,0'
        elif isTrue(tmpHalf):
            cmd_expand = '1075,0,0'
        info = cmd_expand
        return handle_request_common(info,req=request,cn=self.__class__.__name__)
    def get(self):
        return self.handle_request()
    def post(self):
        return self.handle_request()
            
class robot_QBIARD(MethodView):
    def handle_request(self):
        tmpHalf = request.args.get('q', None)
        cmd_expand = ""
        if tmpHalf is not None:
            cmd_expand = tmpHalf
        info = cmd_expand
        return handle_request_QBIARD(info,req=request,cn=self.__class__.__name__)
    def get(self):
        return self.handle_request()
    def post(self):
        return self.handle_request()

class robot_pushText(MethodView):
    def handle_request(self):
        text = request.args.get('text', None)
        info= jsonify({"status": "success", "pushed_text": text})  
        #return handle_request_common(info,req=request,cn=self.__class__.__name__)
        return info
    def get(self):
        return self.handle_request()
    def post(self):
        return self.handle_request()
            
class robot_ITXControl(MethodView):
    def handle_request(self):
        profileString = request.args.get('q', None)
        cmd_expand = ""
        if profileString is not None:
            cmd_expand = profileString
        info = cmd_expand
        return handle_request_common(info,req=request,cn=self.__class__.__name__)
    def get(self):
        return self.handle_request()
    def post(self):
        return self.handle_request()
      
class robot_task_action(MethodView):
    def handle_request(self):
        '''
        data: "{\"ID\": \"Bumblebee1\", \"TRAY_A\": \"2\", \"TRAY_B\": \"1\", \"LEVEL\": \"0\",\
          \ \"STATE\": \"CONFIRM\", \"TIME\": \"20240207_141000\"}"
        '''
        rospy.loginfo(request.args)
        tmpTableGo = request.args.get(APIBLB_FIELDS_ACTION.target.name ,"0", type=str)
        tmpState = request.args.get(BLB_CMD.STATE.name, BLB_CMD_STATUS.MOVE.name,type=str)
        nodeOnlyMode = request.args.get(BLB_CMD.STATE.name, BLB_CMD.MODE.name,type=str)
        tableNo = try_parse_int(tmpTableGo)
        if tmpState != BLB_CMD_STATUS.MOVE.name:
            tableNo = 0
        elif tableNo == 0:
            tableNo = tmpTableGo[1:]
        
        data_out = {BLB_CMD.STATE.name : tmpState,
                  BLB_CMD.ID.name:get_hostname(),
                  BLB_CMD.TRAY_A.name : tableNo,
                  BLB_CMD.TRAY_B.name : 0,
                  BLB_CMD.LEVEL.name : nodeOnlyMode
                  }
        sendbuf = json.dumps(data_out)
        ros_publisher.publish_message(sendbuf)
        return GetFlaskReponse(data_out,request=request)
    def get(self):
        return self.handle_request()
    def post(self):
        return self.handle_request()
      
class robot_clearalarm_action(MethodView):
    def handle_request(self):
        #return GetFlaskReponse(dataReturn,request)
        return handle_request_common(self.name,cn=self.__class__.__name__)
    def get(self):
        return self.handle_request()
    def post(self):
        return self.handle_request()
   
class robot_activity_action(MethodView):
    def handle_request(self):
        #print(request.args.to_dict())
        tmpPause = request.args.get(APIBLB_FIELDS_ACTION.pause.name, type=int)
        tmpResume = request.args.get(APIBLB_FIELDS_ACTION.resume.name, type=int)
        #tmpCancel = request.args.get(APIBLB_FIELDS_ACTION.cancel.name, type=int)
        tmpHome = request.args.get(APIBLB_FIELDS_ACTION.home.name, type=int)
        cmd_expand = APIBLB_FIELDS_ACTION.cancel.name
        if isTrue(tmpResume):
            cmd_expand = APIBLB_FIELDS_ACTION.resume.name
        elif isTrue(tmpPause):
            cmd_expand = APIBLB_FIELDS_ACTION.pause.name
        elif isTrue(tmpHome):
            cmd_expand = APIBLB_FIELDS_ACTION.home.name
        info = cmd_expand
        # data_out = {BLB_CMD.STATE.name : tmpState,
        #           BLB_CMD.ID.name:get_hostname(),
        #           BLB_CMD.TRAY_A.name : 0,
        #           BLB_CMD.TRAY_B.name : 0,
        #           BLB_CMD.LEVEL.name : 0
        #           }
        # sendbuf = json.dumps(data_out)
        # ros_publisher.publish_message(sendbuf)
        # return GetFlaskReponse(data_out,request=request)
        return handle_request_common(info,cn = self.__class__.__name__)
    def get(self):
        return self.handle_request()
    def post(self):
        return self.handle_request()

class robot_tray_action(MethodView):
    def handle_request(self):
        tmpDistance = request.args.get(APIBLB_FIELDS_INFO.distance.name, type=int)
        tmpUp = request.args.get(APIBLB_FIELDS_ACTION.up.name, type=int)
        tmpDown = request.args.get(APIBLB_FIELDS_ACTION.down.name, type=int)
        if tmpUp == tmpDown:
            return GetActionReponse(APIBLB_ACTION_REPLY.E104)
        iBLD_PROFILE_CMD_Action = BLD_PROFILE_CMD.balLiftDown.value
        if isTrue(tmpUp):
            iBLD_PROFILE_CMD_Action = BLD_PROFILE_CMD.balLiftUp.value
        info = f"{iBLD_PROFILE_CMD_Action},{tmpDistance}"
        return handle_request_common(info,request,cn=self.__class__.__name__)
    def get(self):
        return self.handle_request()
    def post(self):
        return self.handle_request()
    
class robot_move_action(MethodView):
    def handle_request(self):
        tmpDistance = request.args.get(APIBLB_FIELDS_INFO.distance.name, type=int, default=200)
        tmpForward = request.args.get(APIBLB_FIELDS_ACTION.forward.name, type=int, default=1)
        tmpBackward = request.args.get(APIBLB_FIELDS_ACTION.backward.name, type=int, default=0)
        if tmpForward == tmpBackward:
            return GetActionReponse(APIBLB_ACTION_REPLY.E104)
        iBLD_PROFILE_CMD_Action = BLD_PROFILE_CMD.MOVE_MOTOR_H.value
        if isTrue(tmpBackward):
            tmpDistance = tmpDistance * -1
        info = f"{iBLD_PROFILE_CMD_Action},{tmpDistance}"
        return handle_request_common(info,request,cn=self.__class__.__name__)
    def get(self):
        return self.handle_request()
    def post(self):
        return self.handle_request()
    
class robot_door_action(MethodView):
    def handle_request(self):
        tmpOpen = request.args.get(APIBLB_FIELDS_ACTION.open.name, type=int)
        tmpClose = request.args.get(APIBLB_FIELDS_ACTION.close.name, type=int)
        if tmpOpen == tmpClose:
            return GetActionReponse(APIBLB_ACTION_REPLY.E104)
        iBLD_PROFILE_CMD_Action = BLD_PROFILE_CMD.DoorDown.value
        if isTrue(tmpOpen):
            iBLD_PROFILE_CMD_Action = BLD_PROFILE_CMD.DoorUp.value
        info = f"{iBLD_PROFILE_CMD_Action}"
        return handle_request_common(info,cn=self.__class__.__name__)
    def get(self):
        return self.handle_request()
    def post(self):
        return self.handle_request()

class robot_task_status(MethodView):
    robot_status_reply_timestamp = DATETIME_OLD
    robot_status_recv_lastValue = BLB_STATUS_FIELD.IDEAL
    robot_status_reply_lastValue = APIBLB_STATUS_TASK.Ideal
    
    def get(self):
        robot_status_reply_timestamp = self.__class__.robot_status_reply_timestamp
        robot_status_reply_lastValue = self.__class__.robot_status_reply_lastValue
        robot_status_recv_lastValue= self.__class__.robot_status_recv_lastValue        
        resultJS = {}
        dictReplyTable2 = {}
        dictReplyTable2[BLB_STATUS_FIELD.READY]=APIBLB_STATUS_TASK.Completed
        dictReplyTable2[BLB_STATUS_FIELD.IDEAL]=APIBLB_STATUS_TASK.Ideal
        dictReplyTable2[BLB_STATUS_FIELD.MOVING]=APIBLB_STATUS_TASK.Running
        dictReplyTable2[BLB_STATUS_FIELD.LIFTING_DOWN]=APIBLB_STATUS_TASK.Running
        dictReplyTable2[BLB_STATUS_FIELD.LIFTING_UP]=APIBLB_STATUS_TASK.Running
        dictReplyTable2[BLB_STATUS_FIELD.CONFIRM]=APIBLB_STATUS_TASK.Completed
        dictReplyTable2[BLB_STATUS_FIELD.FOLDING]=APIBLB_STATUS_TASK.Running
        dictReplyTable2[BLB_STATUS_FIELD.EXPANDING]=APIBLB_STATUS_TASK.Running
        dictReplyTable2[BLB_STATUS_FIELD.ROTATING_MAIN]=APIBLB_STATUS_TASK.Running
        dictReplyTable2[BLB_STATUS_FIELD.CHARGING]=APIBLB_STATUS_TASK.Ideal
        dictReplyTable2[BLB_STATUS_FIELD.WAITING_CROSS]=APIBLB_STATUS_TASK.Paused
        dictReplyTable2[BLB_STATUS_FIELD.OBSTACLE_DETECTED]=APIBLB_STATUS_TASK.Paused
        dictReplyTable2[BLB_STATUS_FIELD.PAUSED]=APIBLB_STATUS_TASK.Paused
        
        iCurrTable = int(dictGlobal.get(BLB_STATUS.NODE_CURRENT.name,-1))
        strStatus = dictGlobal.get(BLB_STATUS.STATUS.name,"")
        blbStatus = getEnumInstanceFromName(BLB_STATUS_FIELD,strStatus)
        replyStatus = dictReplyTable2.get(blbStatus,robot_status_reply_lastValue)
        isKeepTimeOvered = isTimeExceeded(robot_status_reply_timestamp,robot_status_reply_keeptime*1000)
        isKitchen = True if iCurrTable == NODE_SPECIAL_VALUE.KITCHEN_STATION.value else False
        iCurrTable = int(dictGlobal.get(BLB_STATUS.NODE_CURRENT.name,-1))
        print(iCurrTable,strStatus,replyStatus.name,isKeepTimeOvered,isKitchen,iCurrTable)
        print(robot_status_recv_lastValue,robot_status_reply_lastValue)
        #다른곳에서 서빙해서 내려진 상태이거나 부엌에서 내려진 상태
        if blbStatus == BLB_STATUS_FIELD.LIFTING_UP:
          if robot_status_recv_lastValue != BLB_STATUS_FIELD.LIFTING_UP:
            self.__class__.robot_status_reply_timestamp = getDateTime()
            isKeepTimeOvered = False
          if not isKeepTimeOvered:
            replyStatus = APIBLB_STATUS_TASK.Started if isKitchen else APIBLB_STATUS_TASK.Completed        
        
        resultJS[APIBLB_FIELDS_STATUS.robot_status_code.name] = replyStatus.value
        resultJS[APIBLB_FIELDS_STATUS.robot_status.name] = replyStatus.name
        
        self.__class__.robot_status_reply_lastValue = replyStatus
        self.__class__.robot_status_recv_lastValue = blbStatus
        json_string = json.dumps(resultJS)
        ros_publisher.publish_message_status(json_string)
        #print(resultJS)
        return GetFlaskReponse(resultJS,request=request)

# class robot_navigation_info(MethodView):
#     robot_navi_reply_timestamp = DATETIME_OLD
#     robot_navi_recv_lastValue = BLB_STATUS_FIELD.IDEAL
#     robot_navi_lasttask = ""
#     #recvDF = pd.DataFrame()
#     jsonGlobal = ""
#     # def __init__(self):
#     #     self.robot_navi_reply_timestamp = DATETIME_OLD
#     #     self.robot_navi_recv_lastValue = BLB_STATUS_FIELD.IDEAL
#     #     self.robot_navi_lasttask = ""
#     #     self.jsonGlobal = ""
    
#     def get(self):
#         #started 로 된 첫행을 추출하고
#         #마지막행을 추출한다. (Table)
#         '''
#         current_task = detailcode
#         current_node = 첫행의 start_station
#         final_station = 마지막 행의 endnode
#         task_status = started 가 있으면 running, else waiting
        
#         {   
#         "current_task": “202410011",    "start_station": “1",    "current_node": “1",
#         "final_statoin": “A3",    "task_status": "1 
#         }        
#         '''
#         dictReplyTable = {field.name: "" for field in APIBLB_FIELDS_NAVI}
#         #print(self.recvDF)
#         json_str = dictGlobal.get(APIBLB_FIELDS_STATUS.df.name,[])
#         df = safe_read_json(json_str)
#         if len(df) == 0:
#             print(f'DF is empty : {json_str}')
#             df = safe_read_json(self.__class__.jsonGlobal)
#         else:
#             self.__class__.jsonGlobal = json_str[:]
#             print(f'DF is not empty : {self.__class__.jsonGlobal}')
                    
#         if len(df) > 0:
#             df_started = df[df[APIBLB_FIELDS_TASK.workstatus.name] == APIBLB_STATUS_TASK.Started.value]
#             dicLast = df.iloc[-1]
#             dicFirst = df.iloc[0]
#             current_taskmst = dicFirst[APIBLB_FIELDS_TASK.mastercode.name]
#             start_station = dicFirst[APIBLB_FIELDS_TASK.startnode.name]
#             simulayoutcd = dicFirst[APIBLB_FIELDS_NAVI.simulayoutcd.name]    
#             current_node = start_station
#             current_node = dicFirst[APIBLB_FIELDS_TASK.startnode.name]
#             current_task = dicFirst[APIBLB_FIELDS_TASK.detailcode.name]
            
#             if df_started is not None and len(df_started) > 0:
#                 columns_to_keep = [field.name for field in APIBLB_FIELDS_TASK if field.name in df_started.columns]
#                 print(df[columns_to_keep])
#                 current_node = df_started.iloc[0][APIBLB_FIELDS_TASK.startnode.name]
#                 current_task = df_started.iloc[0][APIBLB_FIELDS_TASK.detailcode.name]
            
#             mst_status = dicFirst[APIBLB_FIELDS_TASK.workstatus.name]
#             if mst_status == APIBLB_STATUS_TASK.Completed.value:
#                 mst_status = APIBLB_STATUS_TASK.Running.value
                
#             final_statoin = dicLast[APIBLB_FIELDS_TASK.endnode.name]
#             endnode = dicLast[APIBLB_FIELDS_TASK.startnode.name]
#             task_status = dicLast[APIBLB_FIELDS_TASK.workstatus.name]
#             if task_status != APIBLB_STATUS_TASK.Completed.value:
#                 if df_started is not None and len(df_started) > 0:
#                     task_status = APIBLB_STATUS_TASK.Started.value
#                 else:
#                     task_status = APIBLB_STATUS_TASK.Ideal.value
#             else:
#                 mst_status = APIBLB_STATUS_TASK.Completed.value
#                 dictDF[current_taskmst] = df.to_json(orient='records')
            
#             dictReplyTable[APIBLB_FIELDS_NAVI.current_taskmst.name] = current_taskmst
#             dictReplyTable[APIBLB_FIELDS_NAVI.current_taskdtl.name] = current_task
#             dictReplyTable[APIBLB_FIELDS_NAVI.start_station.name] = start_station
#             dictReplyTable[APIBLB_FIELDS_NAVI.simulayoutcd.name] = simulayoutcd
#             dictReplyTable[APIBLB_FIELDS_NAVI.endnode.name] = endnode
#             dictReplyTable[APIBLB_FIELDS_NAVI.current_station.name] = current_node
#             dictReplyTable[APIBLB_FIELDS_NAVI.final_station.name] = final_statoin
#             dictReplyTable[APIBLB_FIELDS_NAVI.workstatus.name] = task_status
#             dictReplyTable[APIBLB_FIELDS_NAVI.orderstatus.name] = mst_status
        
#             # iCurrTable = int(dictGlobal.get(BLB_STATUS.NODE_CURRENT.name,-1))
#             # strStatus = dictGlobal.get(BLB_STATUS.STATUS.name,"")
#             # blbStatus = getEnumInstanceFromName(BLB_STATUS_FIELD,strStatus)
#             # replyStatus = dictReplyTable.get(blbStatus,robot_navi_reply_lastValue)
#             isKeepTimeOver = isTimeExceeded(self.__class__.robot_navi_reply_timestamp,robot_status_reply_keeptime*1000)
#             #isKeepTimeOvered = isTimeExceeded(robot_status_reply_timestamp,robot_status_reply_keeptime*1000)
#             # isKitchen = True if iCurrTable == NODE_SPECIAL_VALUE.KITCHEN_STATION.value else False
#             # iCurrTable = int(dictGlobal.get(BLB_STATUS.NODE_CURRENT.name,-1))
#             # print(iCurrTable,strStatus,replyStatus.name,isKeepTimeOver,isKitchen,iCurrTable)
#             # print(robot_navi_recv_lastValue,robot_navi_reply_lastValue)
#             if task_status == APIBLB_STATUS_TASK.Started.value:
#                 if self.__class__.robot_navi_lasttask  == current_task:
#                     if isKeepTimeOver:
#                         task_status = APIBLB_STATUS_TASK.Running.value
#                         dictReplyTable[APIBLB_FIELDS_NAVI.workstatus.name] = task_status
#                         if dictReplyTable[APIBLB_FIELDS_NAVI.orderstatus.name] == APIBLB_STATUS_TASK.Started.value:
#                             dictReplyTable[APIBLB_FIELDS_NAVI.orderstatus.name] = task_status
#                 else:
#                     self.__class__.robot_navi_reply_timestamp = getDateTime()
#             self.__class__.robot_navi_lasttask = current_task
#             self.__class__.robot_navi_recv_lastValue = task_status
#         return GetFlaskReponse(ConvertInt64toInt_Dict(dictReplyTable),request=request)

# class robot_navigation_info(MethodView):
#     robot_navi_reply_timestamp = DATETIME_OLD
#     robot_navi_recv_lastValue = BLB_STATUS_FIELD.IDEAL
#     robot_navi_lasttask = ""
#     #recvDF = pd.DataFrame()
#     jsonGlobal = ""
#     # def __init__(self):
#     #     self.robot_navi_reply_timestamp = DATETIME_OLD
#     #     self.robot_navi_recv_lastValue = BLB_STATUS_FIELD.IDEAL
#     #     self.robot_navi_lasttask = ""
#     #     self.jsonGlobal = ""
    
#     def get(self):
#         #started 로 된 첫행을 추출하고
#         #마지막행을 추출한다. (Table)
#         '''
#         current_task = detailcode
#         current_node = 첫행의 start_station
#         final_station = 마지막 행의 endnode
#         task_status = started 가 있으면 running, else waiting
        
#         {   
#         "current_task": “202410011",    "start_station": “1",    "current_node": “1",
#         "final_statoin": “A3",    "task_status": "1 
#         }        
#         '''
#         dictReplyTable = {field.name: "" for field in APIBLB_FIELDS_NAVI}
#         #print(self.recvDF)
#         json_str = dictGlobal.get(APIBLB_FIELDS_STATUS.df.name,[])
#         df = safe_read_json(json_str)
#         if len(df) == 0:
#             print(f'DF is empty : {json_str}')
#             df = safe_read_json(self.__class__.jsonGlobal)
#         else:
#             self.__class__.jsonGlobal = json_str[:]
#             #print(f'DF is not empty : {self.__class__.jsonGlobal}')
                    
#         if len(df) > 0:
#             df_started = df[df[APIBLB_FIELDS_TASK.workstatus.name] == APIBLB_STATUS_TASK.Started.value]
#             dicLast = df.iloc[-1]
#             dicFirst = df.iloc[0]
#             current_taskmst = dicFirst[APIBLB_FIELDS_TASK.mastercode.name]
#             start_station = dicFirst[APIBLB_FIELDS_TASK.startnode.name]
#             simulayoutcd = dicFirst[APIBLB_FIELDS_NAVI.simulayoutcd.name]    
#             current_node = start_station
#             current_node = dicFirst[APIBLB_FIELDS_TASK.startnode.name]
#             current_task = dicFirst[APIBLB_FIELDS_TASK.detailcode.name]
#             endnode = dicLast[APIBLB_FIELDS_TASK.startnode.name]
#             if df_started is not None and len(df_started) > 0:
#                 columns_to_keep = [field.name for field in APIBLB_FIELDS_TASK if field.name in df_started.columns]
#                 print(df[columns_to_keep])
#                 current_node = df_started.iloc[0][APIBLB_FIELDS_TASK.startnode.name]
#                 start_station = current_node
#                 current_task = df_started.iloc[0][APIBLB_FIELDS_TASK.detailcode.name]
#                 endnode = df_started.iloc[0][APIBLB_FIELDS_TASK.endnode.name]
            
#             mst_status = dicFirst[APIBLB_FIELDS_TASK.workstatus.name]
#             if mst_status == APIBLB_STATUS_TASK.Completed.value:
#                 mst_status = APIBLB_STATUS_TASK.Running.value
                
#             final_statoin = dicLast[APIBLB_FIELDS_TASK.endnode.name]
            
#             task_status = dicLast[APIBLB_FIELDS_TASK.workstatus.name]
#             if task_status != APIBLB_STATUS_TASK.Completed.value:
#                 if df_started is not None and len(df_started) > 0:
#                     task_status = APIBLB_STATUS_TASK.Started.value
#                 else:
#                     task_status = APIBLB_STATUS_TASK.Ideal.value
#             else:
#                 mst_status = APIBLB_STATUS_TASK.Completed.value
#                 current_node = dicLast[APIBLB_FIELDS_TASK.startnode.name]
#                 current_task = dicLast[APIBLB_FIELDS_TASK.detailcode.name]
#                 start_station = current_node
#                 dictDF[current_taskmst] = df.to_json(orient='records')
            
#             dictReplyTable[APIBLB_FIELDS_NAVI.current_taskmst.name] = current_taskmst
#             dictReplyTable[APIBLB_FIELDS_NAVI.current_taskdtl.name] = current_task
#             dictReplyTable[APIBLB_FIELDS_NAVI.start_station.name] = start_station
#             dictReplyTable[APIBLB_FIELDS_NAVI.simulayoutcd.name] = simulayoutcd
#             dictReplyTable[APIBLB_FIELDS_NAVI.endnode.name] = endnode
#             dictReplyTable[APIBLB_FIELDS_NAVI.current_station.name] = current_node
#             dictReplyTable[APIBLB_FIELDS_NAVI.final_station.name] = final_statoin
#             dictReplyTable[APIBLB_FIELDS_NAVI.workstatus.name] = task_status
#             dictReplyTable[APIBLB_FIELDS_NAVI.orderstatus.name] = mst_status
#             isKeepTimeOver = isTimeExceeded(self.__class__.robot_navi_reply_timestamp,robot_status_reply_keeptime*1000)
#             if task_status == APIBLB_STATUS_TASK.Started.value:
#                 if self.__class__.robot_navi_lasttask  == current_task:
#                     if isKeepTimeOver:
#                         task_status = APIBLB_STATUS_TASK.Running.value
#                         dictReplyTable[APIBLB_FIELDS_NAVI.workstatus.name] = task_status
#                         if dictReplyTable[APIBLB_FIELDS_NAVI.orderstatus.name] == APIBLB_STATUS_TASK.Started.value:
#                             dictReplyTable[APIBLB_FIELDS_NAVI.orderstatus.name] = task_status
#                 else:
#                     self.__class__.robot_navi_reply_timestamp = getDateTime()
#             self.__class__.robot_navi_lasttask = current_task
#             self.__class__.robot_navi_recv_lastValue = task_status
#         return GetFlaskReponse(ConvertInt64toInt_Dict(dictReplyTable),request=request)
      
class robot_status(MethodView):
    robot_status_reply_timestamp = DATETIME_OLD
    robot_status_recv_lastValue = BLB_STATUS_FIELD.IDEAL
    robot_status_reply_lastValue = APIBLB_STATUS.Ideal
    
    def get(self):
        robot_status_reply_timestamp = self.__class__.robot_status_reply_timestamp
        robot_status_reply_lastValue = self.__class__.robot_status_reply_lastValue
        robot_status_recv_lastValue= self.__class__.robot_status_recv_lastValue        
        resultJS = {}
        dictReplayTable = {}
        dictReplayTable[BLB_STATUS_FIELD.READY]=APIBLB_STATUS.Tray_Rack_opening
        dictReplayTable[BLB_STATUS_FIELD.IDEAL]=APIBLB_STATUS.Ideal
        dictReplayTable[BLB_STATUS_FIELD.MOVING]=APIBLB_STATUS.Running
        dictReplayTable[BLB_STATUS_FIELD.LIFTING_DOWN]=APIBLB_STATUS.Tray_moving_down
        dictReplayTable[BLB_STATUS_FIELD.LIFTING_UP]=APIBLB_STATUS.Tray_Moving_Up
        dictReplayTable[BLB_STATUS_FIELD.CONFIRM]=APIBLB_STATUS.Tray_Rack_opening
        dictReplayTable[BLB_STATUS_FIELD.FOLDING]=APIBLB_STATUS.Telescope_Shrinking
        dictReplayTable[BLB_STATUS_FIELD.EXPANDING]=APIBLB_STATUS.Telescope_Expanding
        dictReplayTable[BLB_STATUS_FIELD.ROTATING_MAIN]=APIBLB_STATUS.Main_Rotation
        dictReplayTable[BLB_STATUS_FIELD.CHARGING]=APIBLB_STATUS.Ideal
        dictReplayTable[BLB_STATUS_FIELD.WAITING_CROSS]=APIBLB_STATUS.Wait
        
        iCurrTable = int(dictGlobal.get(BLB_STATUS.NODE_CURRENT.name,-1))
        strStatus = dictGlobal.get(BLB_STATUS.STATUS.name,"")
        blbStatus = getEnumInstanceFromName(BLB_STATUS_FIELD,strStatus)
        replyStatus = dictReplayTable.get(blbStatus,robot_status_reply_lastValue)
        isKeepTimeOvered = isTimeExceeded(robot_status_reply_timestamp,robot_status_reply_keeptime*1000)
        isKitchen = True if iCurrTable == NODE_SPECIAL_VALUE.KITCHEN_STATION.value else False
        iCurrTable = int(dictGlobal.get(BLB_STATUS.NODE_CURRENT.name,-1))
        print(iCurrTable,strStatus,replyStatus.name,isKeepTimeOvered,isKitchen,iCurrTable)
        print(robot_status_recv_lastValue,robot_status_reply_lastValue)
        
        #다른곳에서 서빙해서 내려진 상태이거나 부엌에서 내려진 상태
        if blbStatus == BLB_STATUS_FIELD.LIFTING_UP:
          if robot_status_recv_lastValue != BLB_STATUS_FIELD.LIFTING_UP:
            self.__class__.robot_status_reply_timestamp = getDateTime()
            isKeepTimeOvered = False
          if not isKeepTimeOvered:
            replyStatus = APIBLB_STATUS.Started if isKitchen else APIBLB_STATUS.Food_Served_Confirmed
        
        if blbStatus == BLB_STATUS_FIELD.FOLDING:
          if robot_status_recv_lastValue != BLB_STATUS_FIELD.FOLDING:
            self.__class__.robot_status_reply_timestamp = getDateTime()
            isKeepTimeOvered = False
          if not isKeepTimeOvered:
            replyStatus = APIBLB_STATUS.Tray_mounted_to_bumblebee
        
        if blbStatus == BLB_STATUS_FIELD.ROTATING_MAIN:
          if robot_status_recv_lastValue == BLB_STATUS_FIELD.MOVING:
            self.__class__.robot_status_reply_timestamp = getDateTime()
            isKeepTimeOvered = False
          if not isKeepTimeOvered:
            replyStatus = APIBLB_STATUS.target_reached
        #     if isKeepTimeOvered:
        #         replyStatus = dictTimerTable.get(blbStatus,APIBLB_STATUS.Ideal)
        #     else:
        #         replyStatus = APIBLB_STATUS.Started
        # #다른곳에서 서빙해서 내려진 상태이거나 부엌에서 내려진 상태
        # elif blbStatus == BLB_STATUS_FIELD.LIFTING_UP or blbStatus == BLB_STATUS_FIELD.MOVING:
        #     if not isKeepTimeOvered:
                
        #         replyStatus = APIBLB_STATUS.Tray_Rack_opening
            
        resultJS[APIBLB_FIELDS_STATUS.robot_status_code.name] = replyStatus.value
        resultJS[APIBLB_FIELDS_STATUS.robot_status.name] = replyStatus.name
        
        self.__class__.robot_status_reply_lastValue = replyStatus
        self.__class__.robot_status_recv_lastValue = blbStatus
        #print(resultJS)
        return GetFlaskReponse(resultJS,request=request)
        
class robot_speed_info(MethodView):
    def get(self):
        resultJS = {}
        #moveSPD = float(dictGlobal.get(MonitoringField.CUR_SPD.name,-1))
        resultJS[APIBLB_FIELDS_INFO.vy.name] = (dictGlobal.get(APIBLB_FIELDS_INFO.vy.name,-1))
        resultJS[APIBLB_FIELDS_INFO.w.name] = (dictGlobal.get(APIBLB_FIELDS_INFO.w.name,-1))
        resultJS[APIBLB_FIELDS_INFO.vx.name] = (dictGlobal.get(APIBLB_FIELDS_INFO.vx.name,-1))
        return GetFlaskReponse(resultJS,request=request)

class robot_trayload_status(MethodView):
    def get(self):
        resultJS = {}
        iKg = float(dictGlobal.get(CARRIER_STATUS.LOAD1.name,-1))
        jKg = float(dictGlobal.get(CARRIER_STATUS.LOAD2.name,-1))
        resultJS[APIBLB_FIELDS_STATUS.R1.name] = 1 if iKg >= 100 else 0
        resultJS[APIBLB_FIELDS_STATUS.R2.name] = 1 if jKg >= 100 else 0
        return GetFlaskReponse(resultJS,request=request)

class robot_battery_status(MethodView):
    def get(self):
        resultJS = {}
        resultJS[APIBLB_FIELDS_STATUS.robot_charge_node.name] = NODE_SPECIAL_VALUE.CHARGING_STATION.value

        curcadc = float(dictGlobal.get(MonitoringField_BMS.CurCadc.name, -1))
        charging = reply_yes if curcadc <= 0 else reply_no
        resultJS[APIBLB_FIELDS_STATUS.charging.name] = charging
        #resultJS[APIBLB_FIELDS_STATUS.charging.name] = reply_no
        
        watt = dictGlobal.get(MonitoringField_BMS.WATT.name, -1)
        i_home_sensor = int(dictGlobal.get(CARRIER_STATUS.I_DOOR_1_HOME.name,-1))
        auto_sensor_triggered = True if i_home_sensor == 1 else False
        voltMinCell = float(dictGlobal.get(MonitoringField_BMS.Vmin.name, -1))
        resultJS[APIBLB_FIELDS_STATUS.auto_charging.name] = reply_yes if charging == reply_yes and auto_sensor_triggered else reply_no
        resultJS[APIBLB_FIELDS_STATUS.manual_charging.name] = reply_yes if charging == reply_yes and not auto_sensor_triggered else reply_no
        #resultJS[APIBLB_FIELDS_STATUS.manual_charging.name] = reply_no
        battery_percents = min(100,round(voltMinCell if voltMinCell == -1 else mapRange(voltMinCell,3.2,4.1,0,100)))
        if curcadc < 0 and battery_percents == 100:
            battery_percents = 99
        
        resultJS[APIBLB_FIELDS_STATUS.battery_level.name] = battery_percents
        resultJS[APIBLB_FIELDS_STATUS.voltage.name] = dictGlobal.get(MonitoringField_BMS.Voltage.name, -1)
        resultJS[APIBLB_FIELDS_STATUS.battery_temp.name] = dictGlobal.get(MonitoringField_BMS.Tmax.name, -1)
        resultJS[APIBLB_FIELDS_STATUS.watt.name] = watt
        
        return GetFlaskReponse(resultJS,request=request)

class robot_table_info(MethodView):
    def get(self):
        StateInfo: Dict[str, list] = {}
        file_list = getLines_FromFile(strFileCross)
        for i in file_list:
            checkIDX = i.find("#")
            if checkIDX >= 0 or len(i) < 2:
                continue
            splitTmp = i.split(sDivTab)
            if len(splitTmp) > 4:
                nodeID_tmp = (int)(splitTmp[0])
                stateNodeCur = [
                    (int)(splitTmp[1]),
                    (int)(splitTmp[2]),
                    (int)(splitTmp[3]),
                    (int)(splitTmp[4]),
                ]
                StateInfo[nodeID_tmp] = stateNodeCur
            else:
                rospy.loginfo(f"잘못된 형식의 분기기 정보! {i} - 알람에 추가할 것")
        
        graph, bgraphOK,node_seq = LoadGraph(strFileShortCut)
        if not bgraphOK:
            rospy.loginfo(
                f"맵 무결성 검사 실패, 도달할 수 없는 노드가 있습니다 - {graph}- 알람에 추가할 것"
            )
        lsReturn,node_xy = LoadFullGraph2(graph, StateInfo,strCSV_NodeInfo,strCSV_RrailInfo)
        df = pd.DataFrame(lsReturn)
        # 데이터 추출
        x_values = df[APIBLB_FIELDS_INFO.st_xval.name]
        y_values = df[APIBLB_FIELDS_INFO.st_yval.name]
        node_labels = df[APIBLB_FIELDS_INFO.start.name]

        # 플롯 생성
        plt.figure(figsize=(10, 8))
        plt.scatter(x_values, y_values, c='blue', label='Node', zorder=1)

        # 각 노드에 라벨 추가
        for x, y, label in zip(x_values, y_values, node_labels):
            plt.text(x, y, str(label), fontsize=6, ha='right', va='bottom', zorder=3)
        
        resultJS = []
        print(node_xy)
        try:
            # Simulating with the uploaded file
            file_path = strFileTableNodeEx
            df_manager = DataFrameManager(file_path)
            # Using the new transform_to_custom_dict_list method
            custom_dict_list = df_manager.transform_to_custom_dict_list()
            #custom_df = pd.DataFrame(custom_dict_list)
            #tmpReturn = DataFrameManager.sort_by_keys(None, custom_df, "tableno", "end", True,True)
            #print(tmpReturn.to_dict(orient='records'))
            lsX = []
            lsY = []
            for dicTable in custom_dict_list:
                end = int(dicTable[APIBLB_FIELDS_INFO.end.name])
                tableName = dicTable[APIBLB_FIELDS_INFO.tableno.name]
                node_x = node_xy[end]
                tb_xval = dicTable[APIBLB_FIELDS_INFO.tb_xval.name] + int(node_x[0])
                tb_yval = dicTable[APIBLB_FIELDS_INFO.tb_yval.name] + int(node_x[1])
                dicTable[APIBLB_FIELDS_INFO.tb_xval.name] = tb_xval
                dicTable[APIBLB_FIELDS_INFO.tb_yval.name] = tb_yval
                lsX.append(tb_xval)
                lsY.append(tb_yval)
                resultJS.append(dicTable)
                plt.text(tb_xval, tb_yval,tableName , fontsize=10, ha='right', va='bottom', zorder=3)                
            plt.scatter(lsX, lsY, c='red', label='Table', zorder=2)
            #resultJS = custom_dict_list
            # resultJS.append({
            #   "tableno": "H1",
            #   "end": "98",
            #   "distance": "10",
            #   "time": "2",
            #   "direction": "N"
            # })
            #resultJS = return jsonify(custom_dict_list)
            
        except Exception as e:
            print(traceback.format_exc())
        
        # 축과 배경 설정
        plt.axhline(0, color='black', linewidth=0.5, zorder=1)
        plt.axvline(0, color='black', linewidth=0.5, zorder=1)
        plt.grid(color='gray', linestyle='--', linewidth=0.5, zorder=0)
        plt.title('Node&Table Location')
        plt.xlabel('Loc X')
        plt.ylabel('Loc Y')
        # Loc X와 Loc Y 축 범위를 동일하게 설정
        min_val = min(min(lsX), min(lsY), min(x_values), min(y_values))
        max_val = max(max(lsX), max(lsY), max(x_values), max(y_values))
        # Loc X와 Loc Y 축 범위를 동일하게 설정하며, 마진 추가
        margin = (max_val - min_val) * 0.1  # 전체 범위의 10%를 마진으로 추가
        plt.xlim(min_val - margin, max_val + margin)
        plt.ylim(min_val - margin, max_val + margin)        
        # plt.xlim(min_val, max_val)
        # plt.ylim(min_val, max_val)        
        plt.legend()
        plt.gca().set_aspect('equal', adjustable='box')

        if isFileExist(strPNG_NodeInfo):
            os.remove(strPNG_NodeInfo)
        try:
          plt.savefig(strPNG_NodeInfo)
        except Exception as e:
            print(e)          
        plt.close()        
        return GetFlaskReponse(resultJS,request=request)

class robot_online_status(MethodView):
    def get(self):
        # return jsonify(getCurrentTime())
        return GetFlaskReponse({APIBLB_FIELDS_STATUS.onlineStatus.name: reply_yes},request=request)

class robot_loc_info(MethodView):
    def get(self):
        x = dictGlobal.get(APIBLB_FIELDS_INFO.x.name,-1)
        y = dictGlobal.get(APIBLB_FIELDS_INFO.y.name,-1)
        angle = dictGlobal.get(APIBLB_FIELDS_INFO.angle.name,-1)
        dicReturn = {}
        dicReturn[APIBLB_FIELDS_INFO.x.name] = x #3.5069
        dicReturn[APIBLB_FIELDS_INFO.y.name] = y #0.00687
        dicReturn[APIBLB_FIELDS_INFO.angle.name] = angle #-0.0064
        return GetFlaskReponse(dicReturn,request=request)

class robot_node_info(MethodView):
    def get(self):
        StateInfo: Dict[str, list] = {}.update(GetCrossInfo())
        
        graph, bgraphOK,node_seq = LoadGraph(strFileShortCut)
        if not bgraphOK:
            sMsg = f"맵 무결성 검사 실패, 도달할 수 없는 노드가 있습니다 {graph}"
            #SendAlarmHTTP(sMsg)
            rospy.loginfo(sMsg)
        lsReturn,node_xy = LoadFullGraph2(graph, StateInfo,strCSV_NodeInfo,strCSV_RrailInfo)
        #visualize_graph(lsReturn)
        # tmpDF = DataFrameManager(pd.DataFrame(lsReturn))
        # rospy.loginfo(tmpDF)
        return GetFlaskReponse(lsReturn,request=request)
    
class robot_rfid_event(MethodView):
    def post(self):
        data = request.json
        print(data)
        pause = data.get("pause")
        resume = data.get("resume")
        suspended = data.get("suspended")

        # 실제 로직 구현

        return jsonify(data)

    def get(self):
        global seq
        data = request.args
        EPC_Array = data.get(RFID_RESULT.EPC.name)
        ip_addr = data.get(CALLBELL_FIELD.IP.name)
        topic_name = f'/{TopicName.RFID.name}' 
        # 현재 활성화된 토픽 리스트 가져오기
        topics = ros.get_topics()
        if topic_name in topics and ip_addr is not None:
            # 토픽 발행
            seq += 1
            talker = roslibpy.Topic(ros, topic_name, 'std_msgs/String')
            keys2 = [key.strip() for key in EPC_Array.split(sDivItemComma) if len(key.strip()) > 1]
            # Create a list of dictionaries
            keys = list(dict.fromkeys(keys2))
            result = [{RFID_RESULT.EPC.name: key, RFID_RESULT.DEVID.name : ip_addr, RFID_RESULT.SEQ.name: seq} for key in keys]
            message = json.dumps(result, ensure_ascii=False)
            talker.publish(roslibpy.Message({'data': message}))
            print(f"Published message to '{topic_name}': {message}")        
        
        print(data)
        return jsonify(data)
    

class PavControlView(MethodView):
    def post(self):
        data = request.json
        print(data)
        pause = data.get("pause")
        resume = data.get("resume")
        suspended = data.get("suspended")

        # 실제 로직 구현

        return jsonify(data)

    def get(self):
        data = request.args
        print(data)
        return jsonify(data)

class robot_topic_publish(MethodView):
    def post(self):
        try:
            # GET 요청의 쿼리 파라미터를 파싱
            data = request.get_json()
            #is_json()
            talker = roslibpy.Topic(ros, data['topic'], 'std_msgs/String')
            talker.publish(roslibpy.Message({'data': json.dumps(data)}))
            return jsonify(data),200
        except Exception as e:
            return jsonify({"error": str(e)}), 500    
    def get(self):
        try:
            # GET 요청의 쿼리 파라미터를 파싱
            data = request.args.to_dict()['data']
            dictData = json.loads(data)
            topicName = dictData.get('topic')
            talker = roslibpy.Topic(ros, topicName, 'std_msgs/String')
            talker.publish(roslibpy.Message({'data': json.dumps(dictData)}))            
            return jsonify(data),200
        except Exception as e:
            return jsonify({"error": str(e)}), 500
        
class PavControlView2(MethodView):
    def get(self):
      """Generator to simulate real-time messages."""
      while True:
          # 메시지 생성
        return Response(getCurrentTime(), content_type='text/event-stream')  
      data = request.json
      return self.handle_request()
    
    
    def post(self):
      data = request.json
      return self.handle_request()


registered_views = []


# 자동으로 모든 MethodView 클래스를 등록하는 함수
def register_views(blueprint):
    for name, obj in globals().items():
        if inspect.isclass(obj) and issubclass(obj, MethodView) and obj != MethodView:
            #view_name = name.lower().replace("view", "")
            view_name = name.replace("view", "")
            url = f"/{view_name}"
            blueprint.add_url_rule(url, view_func=obj.as_view(view_name))
            registered_views.append((name, url))


# 등록된 모든 뷰와 URL을 출력하는 함수
def print_registered_views():
    print("Registered views and their URLs:")
    for view_name, url in registered_views:
        print(f"- {view_name}: {url}")


# 모든 뷰 자동 등록
register_views(pan_blueprint)

# 등록된 뷰와 URL 출력
print_registered_views()


# 서비스 URL을 반환하는 새로운 뷰 추가
class ServiceUrlsView(MethodView):
    def get(self):
        urls = {name: url for name, url in registered_views}
        return jsonify(urls)


pan_blueprint.add_url_rule(
    "/service_urls2", view_func=ServiceUrlsView.as_view("service_urls2")
)
