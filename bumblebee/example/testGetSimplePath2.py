from node_CtlCenter_import import *
import node_CtlCenter_globals

lsTest = [
    {
        "START_NODE": 99,
        "START_STATUS": -1,
        "END_NODE": 98,
        "END_STATUS": -1,
        "DISTANCE": 500000,
    },
    {
        "START_NODE": 98,
        "START_STATUS": -1,
        "END_NODE": 6,
        "END_STATUS": -1,
        "DISTANCE": 500000,
    },
    {
        "START_NODE": 6,
        "START_STATUS": -1,
        "END_NODE": 100,
        "END_STATUS": 1,
        "DISTANCE": 500000,
    },
    {
        "START_NODE": 100,
        "START_STATUS": 0,
        "END_NODE": 7,
        "END_STATUS": -1,
        "DISTANCE": 500000,
    },
    {
        "START_NODE": 7,
        "START_STATUS": -1,
        "END_NODE": 4,
        "END_STATUS": -1,
        "DISTANCE": 500000,
    },
    {
        "START_NODE": 4,
        "START_STATUS": -1,
        "END_NODE": 9,
        "END_STATUS": -1,
        "DISTANCE": 500000,
    },
    {
        "START_NODE": 9,
        "START_STATUS": -1,
        "END_NODE": 10,
        "END_STATUS": -1,
        "DISTANCE": 500000,
    },
    {
        "START_NODE": 10,
        "START_STATUS": -1,
        "END_NODE": 102,
        "END_STATUS": 0,
        "DISTANCE": 500000,
    },
    {
        "START_NODE": 102,
        "START_STATUS": 1,
        "END_NODE": 1,
        "END_STATUS": -1,
        "DISTANCE": 500000,
    },
]


# print(lsTest)

# if len(lsTest) <= 0:
#   return
lsFinal = []


def getSimplePath(lsTest):
    lsReturn = []
    startDic = lsTest.pop(0)
    while len(lsTest) > 0:
        curDic = lsTest.pop(0)
        startDic[SeqMapField.DISTANCE.name] += curDic[SeqMapField.DISTANCE.name]
        startDic[SeqMapField.END_NODE.name] = curDic[SeqMapField.END_NODE.name]

        if curDic[SeqMapField.END_STATUS.name] != -1:
            startDic[SeqMapField.END_STATUS.name] = curDic[SeqMapField.END_STATUS.name]
            lsReturn.append(startDic)
            startDic = lsTest.pop(0)

    lsReturn.append(startDic)
    lsFinal2 = []
    dicCrossState = {}
    print(f"1차 필터링 : {lsReturn}")
    return lsReturn


def getSimplePath2(lsReturn):
    if len(lsReturn) < 2:
        return lsReturn

    lsFinal2 = []
    dicCrossState = {}
    startDic2 = lsReturn.pop(0)
    startDic2StartStatus = startDic2[SeqMapField.START_STATUS.name]
    startDic2EndStatus = startDic2[SeqMapField.END_STATUS.name]
    dicCrossState[startDic2[SeqMapField.START_NODE.name]] = startDic2StartStatus
    dicCrossState[startDic2[SeqMapField.END_NODE.name]] = startDic2EndStatus

    while len(lsReturn) > 0:
        curDic = lsReturn.pop(0)
        curDicStartNode = curDic[SeqMapField.START_NODE.name]
        curDicStartStatus = curDic[SeqMapField.START_STATUS.name]
        curDicEndNode = curDic[SeqMapField.END_NODE.name]
        curDicEndStatus = curDic[SeqMapField.END_STATUS.name]
        dicCurNodeState = {}
        dicCurNodeState[curDicStartNode] = curDicStartStatus
        dicCurNodeState[curDicEndNode] = curDicEndStatus
        bSplit = False
        for nodeTmp, nodeStatus in dicCurNodeState.items():
            # print(nodeTmp, nodeStatus)
            # print(dicCrossState)
            paramedStatus = dicCrossState.get(nodeTmp, -1)
            if paramedStatus >= 0 and paramedStatus != nodeStatus:
                bSplit = True
                startDic2[SeqMapField.CROSS_STATUS.name] = {
                    k: v for k, v in dicCrossState.items() if v != -1
                }

            if paramedStatus == -1:
                dicCrossState[nodeTmp] = nodeStatus

        if bSplit:
            lsFinal2.append(startDic2.copy())
            dicCrossState.clear()
            curDic[SeqMapField.CROSS_STATUS.name] = {
                k: v for k, v in dicCurNodeState.items() if v != -1
            }
            dicCrossState = dicCurNodeState
            startDic2 = curDic.copy()
            # lsFinal2.append(curDic)
        else:
            if len(startDic2) > 0:
                startDic2[SeqMapField.DISTANCE.name] += curDic[
                    SeqMapField.DISTANCE.name
                ]
                startDic2[SeqMapField.END_NODE.name] = curDic[SeqMapField.END_NODE.name]
            else:
                startDic2 = curDic
        if len(lsReturn) == 0:
            if len(dicCrossState) > 0:
                startDic2[SeqMapField.CROSS_STATUS.name] = {
                    k: v for k, v in dicCrossState.items() if v != -1
                }
            lsFinal2.append(startDic2.copy())
            dicCrossState.clear()

    print(f"2차 필터링 : {lsFinal2}")
    return lsFinal2


# lsFinal = getSimplePath(lsTest)
lsFinal2 = getSimplePath2(lsTest)
# print(lsFinal2)
