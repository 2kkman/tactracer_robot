from flask import Flask, request, jsonify
from flask_cors import CORS
import ssl

app = Flask(__name__)
CORS(app)  # CORS 설정을 추가하여 크로스 도메인 요청을 허용합니다.

pathKey = '/root/key.pem'
pathCert = '/root/cert.pem'

@app.route('/nav_state', methods=['GET'])
def nav_state():
    print(f'Received parameters: {request.args}')
    
    # URL 파라미터에서 데이터를 가져옵니다.
    pause = request.args.get('pause', type=int)
    resume = request.args.get('resume', type=int)
    suspended = request.args.get('suspended', type=int)
   
    # 여기에 실제 로직을 구현합니다.
    # 예: AGV의 상태를 변경하는 코드
   
    response = {
  "onlineStatus": "yes"
    }
    return jsonify(response)

if __name__ == '__main__':
    context = ssl.SSLContext(ssl.PROTOCOL_TLSv1_2)
    context.load_cert_chain(pathCert, pathKey)  # SSL 인증서 파일
    app.run(host='0.0.0.0', port=9000, ssl_context=context, debug=False)