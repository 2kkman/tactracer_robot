from flask import Blueprint, request, jsonify
from flask.views import MethodView
from ..utils import *
import inspect
from ..ros.publishers import RosPublisher
from ..ros.subscribers import RosSubscriber
from ..ros.service_clients import RosServiceClient

nav_blueprint = Blueprint('nav', __name__)
ros_publisher = RosPublisher()
ros_subscriber = RosSubscriber()
ros_service_client = RosServiceClient()

class NavStateView(MethodView):
    def post(self):
        data = request.json
        pause = data.get('pause')
        resume = data.get('resume')
        suspended = data.get('suspended')
        
        # 실제 로직 구현
        
        return jsonify(getCurrentTime())
    def get(self):
        return jsonify(getCurrentTime())

class NavControlView(MethodView):
    def post(self):
        data = request.json
        speed = data.get('speed')
        direction = data.get('direction')
        
        # 실제 로직 구현
        
        return jsonify(data)
    def get(self):
        tmpQ = request.args
        return jsonify(tmpQ)
        
class NavInfoView(MethodView):
    def get(self):
        tmpQ = request.args.get('q', type=int)
        # 실제 로직 구현 (예: 현재 위치, 배터리 상태 등을 조회)
        info = f'PROFILE:{tmpQ},1'
        print(info)
        ros_publisher.publish_message(info)
        #return jsonify(common_response("success", "Navigation info retrieved", info))
        return jsonify(info)

registered_views = []

# 자동으로 모든 MethodView 클래스를 등록하는 함수
def register_views(blueprint):
    for name, obj in globals().items():
        if inspect.isclass(obj) and issubclass(obj, MethodView) and obj != MethodView:
            view_name = name.lower().replace('view', '')
            url = f'/{view_name}'
            blueprint.add_url_rule(url, view_func=obj.as_view(view_name))
            registered_views.append((name, url))

# 등록된 모든 뷰와 URL을 출력하는 함수
def print_registered_views():
    print("Registered views and their URLs:")
    for view_name, url in registered_views:
        print(f"- {view_name}: {url}")

# 모든 뷰 자동 등록
register_views(nav_blueprint)

# 등록된 뷰와 URL 출력
print_registered_views()

# 서비스 URL을 반환하는 새로운 뷰 추가
class ServiceUrlsView(MethodView):
    def get(self):
        urls = {name: url for name, url in registered_views}
        return jsonify(urls)

nav_blueprint.add_url_rule('/service_urls', view_func=ServiceUrlsView.as_view('service_urls'))