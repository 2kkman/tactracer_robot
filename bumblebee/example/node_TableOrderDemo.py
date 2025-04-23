#!/usr/bin/env python3
import copy
import math
import rospy
import roslib
from varname import *
from std_msgs.msg import Header
from std_msgs.msg import String
import argparse, socket, time, json, datetime, platform, psutil, requests, pprint, uuid, sys
from Util import *
from UtilBLB import *

# items = [
#     {
#         "name": "아메리카노",
#         "englishName": "Americano",
#         "price": 4500,
#         "quantity": 1,
#         "totalPrice": 4500,
#         "tableID": 1,
#         "cmd": "additem",
#     },
#     {
#         "name": "아이스아메리카노",
#         "englishName": "Americano",
#         "price": 4500,
#         "quantity": 2,
#         "totalPrice": 4500,
#         "tableID": 1,
#         "cmd": "additem",
#     },
#     {
#         "name": "아이스아메리카노",
#         "englishName": "Americano",
#         "price": 4500,
#         "quantity": 3,
#         "totalPrice": 4500,
#         "tableID": 1,
#         "cmd": "additem",
#     },
# ]

# # "아이스아메리카노"인 요소의 "quantity" 합계를 계산
# quantity_sum = sum(
#     item["quantity"] for item in items if item["name"] == "아이스아메리카노"
# )

# print(quantity_sum)


# strTmp = '[{"\\uc544\\uc774\\uc2a4 \\uc544\\uba54\\ub9ac\\uce74\\ub178": {"id": 1, "name": "\\uc544\\uc774\\uc2a4 \\uc544\\uba54\\ub9ac\\uce74\\ub178", "englishName": "Iced Americano", "price": "4500", "count": 2, "price_total": "9000"}, "\\ud56b \\uc544\\uba54\\ub9ac\\uce74\\ub178": {"id": 2, "name": "\\ud56b \\uc544\\uba54\\ub9ac\\uce74\\ub178", "englishName": "Hot Americano", "price": "4000", "count": 2, "price_total": "8000"}, "total_price": "17000", "total_count": "4", "timestamp": "140343"}]'
# recvDataMap = json.loads(strTmp)
# print(recvDataMap)


grouped = {}
orderList = []
orderTmp = {}
runFromLaunch = False
hostname = socket.gethostname()


def getTotalPriceCount(dicList: dict):
    totalprice = 0
    totalcount = 0
    for dicTmp in dicList:
        price_cur = dicTmp["price_total"].replace(",", "")
        cnt_cur = dicTmp["count"]
        totalprice += int(price_cur)
        totalcount += int(cnt_cur)
    return totalprice, totalcount


def getOrderSummary(dicList):
    groupedTmp = {}
    totalprice = 0
    totalcount = 0
    listSeqMapOrg_B = copy.deepcopy(dicList)
    for dicItem in listSeqMapOrg_B:
        key = dicItem["name"]
        # Convert price string to integer for aggregation
        price = dicItem["price"]
        quantity = dicItem["quantity"]
        if isinstance(price, str):
            price = int(price.replace(",", ""))
        if isinstance(quantity, str):
            quantity = int(price.replace(",", ""))

        curprice = price * quantity
        # price = int(dicItem["price"].replace(",", ""))
        # price = int(dicItem["price"])
        totalprice += curprice
        totalcount += quantity
        if key in groupedTmp:
            # Increment price and count if item already exists

            groupedTmp[key]["price_total"] += curprice
            groupedTmp[key]["quantity"] += quantity
        else:
            # Create new entry for unique item
            groupedTmp[key] = dicItem
            groupedTmp[key][
                "price"
            ] = price  # Temporarily hold price as integer for calculation
            groupedTmp[key]["quantity"] = 1
            groupedTmp[key]["price_total"] = price
    listReturn = []
    # for key, dicItem in groupedTmp.items():
    #     price_tmp = dicItem["price"]
    #     pricetotal_tmp = dicItem["price_total"]
    #     dicItem["price"] = f"{price_tmp:,}"
    #     dicItem["price_total"] = f"{pricetotal_tmp:,}"
    for key, dicItem in groupedTmp.items():
        listReturn.append(dicItem)

    dicExtraInfo = {}
    dicExtraInfo["total_price"] = totalprice
    dicExtraInfo["total_count"] = totalcount
    # groupedTmp["total_price"] = f"{totalprice:,}"
    # groupedTmp["total_count"] = f"{totalcount:,}"
    dicExtraInfo["timestamp"] = getCurrentTime("")
    listReturn.append(dicExtraInfo)
    return listReturn


def prtMsg(sendbuf):
    if runFromLaunch:
        rospy.loginfo(sendbuf)
    else:
        print(sendbuf)


def talker():
    pubTopic = TopicName.TABLE_ORDER_STATUS.name
    pub = rospy.Publisher(pubTopic, String, queue_size=10)
    rate = rospy.Rate(2)  # send 0.5 time per second
    total_price = 0
    total_count = 0
    returnDic = {}
    while not rospy.is_shutdown():
        if len(orderList) > 0 and pub is not None:
            groupedCurrent = getOrderSummary(orderList)
            # total_price, total_count = getTotalPriceCount(grouped.values)
            sendbuf = json.dumps(groupedCurrent)
            pub.publish(sendbuf)
            prtMsg(sendbuf)

        rate.sleep()


def callbackCmd2(data):
    global grouped
    global orderList
    recvDataMap = None
    try:
        recvData = data.data
        if is_json(recvData):
            recvDataMap = json.loads(recvData)
            print(recvDataMap)
    except Exception as e:
        message = traceback.format_exc()
        rospy.logdebug(message)


def callbackCmd(data):
    global grouped
    global orderList
    recvDataMap = None
    try:
        recvData = data.data
        if is_json(recvData):
            recvDataMap = json.loads(recvData)
        else:
            # recvDataMap = getDic_strArr(recvData, sDivFieldColon, sDivItemComma)
            recvDataMap = {}
            recvDataMap["addItem"] = [
                {
                    "id": 1,
                    "name": "아이스 아메리카노",
                    "englishName": "Iced Americano",
                    "price": "4,500",
                },
                {
                    "id": 1,
                    "name": "아이스 아메리카노",
                    "englishName": "Iced Americano",
                    "price": "4,500",
                },
                {
                    "id": 2,
                    "name": "핫 아메리카노",
                    "englishName": "Hot Americano",
                    "price": "4,000",
                },
                {
                    "id": 2,
                    "name": "핫 아메리카노",
                    "englishName": "Hot Americano",
                    "price": "4,000",
                },
            ]
        # strMsg = '"addItem": [{"id": 1,"name": "아이스 아메리카노","englishName": "Iced Americano", "price": "4,500"},{"id": 1,"name": "아이스 아메리카노","englishName": "Iced Americano", "price": "4,500"}, {"id": 2,"name": "핫 아메리카노","englishName": "Hot Americano", "price": "4,000"}, {"id": 2,"name": "핫 아메리카노","englishName": "Hot Americano", "price": "4,000"}]'
        # json_str_corrected = "{" + strMsg.replace("'", '"') + "}"
        # 문자열을 파이썬 딕셔너리로 변환
        # data = json.loads(json_str_corrected)
        logmsg = f"{recvData} from {sys._getframe(0).f_code.co_name} - {sys._getframe(1).f_code.co_name}"
        rospy.loginfo(logmsg)
        cmdTmp = recvDataMap.get("cmd", None)
        cmdQuantity = recvDataMap.get("quantity", None)
        cmdName = recvDataMap.get("name", None)
        if cmdTmp == "additem" and cmdQuantity == 0:
            removeDictFromList("name", cmdName, orderList)
        elif cmdTmp == "additem":
            orderList.append(recvDataMap)
        elif cmdTmp == "delcart":
            orderList.clear()
            return
        else:
            print("이도저도 아님")
            print(recvDataMap)
        # "아이스아메리카노"인 요소의 "quantity" 합계를 계산
        quantity_sum = sum(
            item["quantity"] for item in orderList if item["name"] == cmdName
        )
        if quantity_sum <= 0:
            removeDictFromList("name", cmdName, orderList)

        # for key, value in recvDataMap.items():
        #     # key 값은 명령어, value 는 해당 아이템의 dict 개체의 배열.
        #     # print(f"Key: {key}, Value: {value}")

        #     '''아래 부분은 addItem - dictArray 일때 (구 메세지)
        #     if key == "addItem":
        #         orderList.extend(value)
        #     elif key == "delItem":
        #         delItem = value[0]["name"]
        #         for i in range(len(orderList) - 1, -1, -1):
        #             if orderList[i]["name"] == delItem:
        #                 del orderList[i]
        #     '''
    except Exception as e:
        message = traceback.format_exc()
        print(message)
        # rospy.logdebug(message)
        # SendFeedback(e)
    return grouped


if __name__ == "__main__":
    try:
        rospy.init_node(f"node_TableOrderDemo", anonymous=True)
        rospy.Subscriber(TopicName.TABLE_ORDER_REQ.name, String, callbackCmd)
        # rospy.Subscriber(TopicName.TABLE_ORDER_STATUS.name, String, callbackCmd2)
        runFromLaunch = rospy.get_param("~startReal", default=False)
        print(f"runFromLaunch : {runFromLaunch}")
        talker()
    except rospy.ROSInterruptException:
        pass
