import ssl
from app import create_app, socketio
from app.config import Config
from UtilBLB import *
from flask import *

app = create_app()
pathKey = f'{getConfigPath(UbuntuEnv.ITX.name)}/key.pem'
pathCert = f'{getConfigPath(UbuntuEnv.ITX.name)}/cert.pem'

if __name__ == '__main__':
    context = ssl.SSLContext(ssl.PROTOCOL_TLSv1_2)
    context.load_cert_chain(pathCert, pathKey)
    app.run(host='0.0.0.0', port=9000, ssl_context=context)
    #socketio.run(app, host='0.0.0.0', port=9000, ssl_context=context)  # Use socketio.run