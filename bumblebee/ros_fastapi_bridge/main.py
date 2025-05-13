from fastapi import FastAPI
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from routers import control
from routers import MANAGEMENT_SBC
import uvicorn
from pathlib import Path
import rospy

try:
    rospy.init_node('fastapi_bridge_node', anonymous=True)
except rospy.exceptions.ROSException:
    pass  # Already initialized

app = FastAPI(title="ROS-FastAPI Bridge")
# 👉 절대 경로 기준으로 static 폴더 지정

BASE_DIR = Path(__file__).resolve().parent
FAVICON_PATH = BASE_DIR / "static" / "favicon.ico"
@app.get("/favicon.ico")
async def favicon():
    return FileResponse(FAVICON_PATH)
app.include_router(control.router)
app.include_router(MANAGEMENT_SBC.router)

@app.get("/")
def root():
    return {"message": "ROS-FastAPI Bridge is running"}

if __name__ == "__main__":
    # 이 block이 F5 실행의 핵심
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=6002,
        reload=False,
        log_level="info"
    )
