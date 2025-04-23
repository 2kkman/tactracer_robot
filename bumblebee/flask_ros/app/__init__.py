from flask import Flask
from flask_cors import CORS
from .config import Config
#from .ros.ros_node import init_ros_node, shutdown_ros_node
from .ros import init_ros
from .utils import *
from flask_socketio import SocketIO
import json
messageGlobal = ''
socketio = SocketIO()  # Create a global SocketIO object
def generate_messages():
    global messageGlobal
    while True:
        # 메시지 생성
        message = getStr_fromDic(dictGlobal, sDivFieldColon, sDivItemComma)
        if messageGlobal != message:
          messageGlobal = message
          yield f"data: {messageGlobal}\n\n"
          time.sleep(0.1)  # 0.1초마다 새로운 메시지 전송    

def stream():
    """Route to stream messages."""
    
    return Response(generate_messages(), content_type='text/event-stream')
  

def create_app():
    app = Flask(__name__)
    app.config.from_object(Config)
    CORS(app)
    init_ros()
    socketio.init_app(app)
    from .views.pan_views import pan_blueprint
    app.register_blueprint(pan_blueprint)
    app.add_url_rule('/stream', view_func=stream)
    return app