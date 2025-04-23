from UtilBLB import *
from enum import Enum
from flask import Flask, Response
import json
import logging
from logging.handlers import RotatingFileHandler
from std_srvs.srv import *
from rospy_tutorials.srv import *
from turtlesim.srv import *
from tta_blb.srv import *

lock = threading.Lock()
dictGlobal = {}
dictDF = {}

def visualize_graph(bidirectional_list):
    G = nx.DiGraph()
    
    # Add edges to the graph
    for item in bidirectional_list:
        start = item[APIBLB_FIELDS_INFO.start.name]
        end = item[APIBLB_FIELDS_INFO.end.name]
        G.add_edge(start, end)
        
        # Add node positions
        G.nodes[start][MotorWMOVEParams.POS.name] = (float(item[APIBLB_FIELDS_INFO.st_xval.name]), float(item[APIBLB_FIELDS_INFO.st_yval.name]))
        G.nodes[end][MotorWMOVEParams.POS.name] = (float(item[APIBLB_FIELDS_INFO.et_xval.name]), float(item[APIBLB_FIELDS_INFO.et_yval.name]))

    # Get node positions
    pos = nx.get_node_attributes(G, MotorWMOVEParams.POS.name)

    # Create figure and axis
    fig, ax = plt.subplots(figsize=(12, 8))

    # Draw nodes
    nx.draw_networkx_nodes(G, pos, node_size=700, node_color='lightblue', ax=ax)

    # Draw edges
    nx.draw_networkx_edges(G, pos, edge_color='gray', arrows=True, arrowsize=20, ax=ax)

    # Draw labels
    nx.draw_networkx_labels(G, pos, ax=ax)

    # Add direction labels to edges
    edge_labels = {(item[APIBLB_FIELDS_INFO.start.name], item[APIBLB_FIELDS_INFO.end.name]): item[APIBLB_FIELDS_INFO.direction.name] for item in bidirectional_list if item[APIBLB_FIELDS_INFO.direction.name]}
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, ax=ax)

    # Set plot limits
    x_values, y_values = zip(*pos.values())
    x_margin = (max(x_values) - min(x_values)) * 0.1
    y_margin = (max(y_values) - min(y_values)) * 0.1
    plt.xlim(min(x_values) - x_margin, max(x_values) + x_margin)
    plt.ylim(min(y_values) - y_margin, max(y_values) + y_margin)

    # Remove axis
    plt.axis('off')

    # Show plot
    plt.tight_layout()
    plt.show()
    
def GetFlaskReponse(objReturn, request=None):
    rt = Response()
    if isinstance(objReturn, pd.DataFrame):
        objReturn = objReturn.to_dict(orient='records')
    if request is None:
        rospy.loginfo(f'Request is None, Parma type is {type(objReturn)}')
    else:
        # IP 주소 얻기
        client_ip = request.remote_addr

        # X-Forwarded-For 헤더에서 원래 클라이언트 IP를 얻는 방법 (리버스 프록시 사용 시)
        if request.headers.getlist("X-Forwarded-For"):
            client_ip = request.headers.getlist("X-Forwarded-For")[0]
        paramInput = {}
        # 기타 메타 정보
        method = request.method
        if method == 'GET':
            paramInput = request.args.to_dict()
        else:
            if request.is_json:
                method += ' Json'
                paramInput = request.json
            else:
                method += ' Form'
                paramInput = request.form.to_dict()
        user_agent = request.user_agent.string
        requested_url = request.url
        if client_ip != IP_MASTER:
          rospy.loginfo(f"IP:{client_ip},Method:{method},URL:{requested_url},Param:{paramInput}")
    
    try:
        rt= Response(
            json.dumps(objReturn, sort_keys=False),
            status=200,
            mimetype='application/json'
        )    
    except Exception as e:
        log_all_frames(traceback.format_exc())
    return (rt)

def GetExceptionReponse(request=None):
    return GetActionReponse(APIBLB_ACTION_REPLY.E500, request)

def GetActionReponse(objReturn : APIBLB_ACTION_REPLY, request=None):
    return GetFlaskReponse({APIBLB_ACTION_REPLY.code.name : objReturn.name, APIBLB_ACTION_REPLY.message.name : objReturn.value}, request)
       
def CheckBLBActionAlarm(StrAction):
    return True