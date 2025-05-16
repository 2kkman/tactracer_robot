from fastapi import APIRouter, Query, HTTPException
from ros_publisher import publish_string
from UtilBLB import *

end_point = EndPoints.MANAGEMENT_SBC.name
router = APIRouter(prefix=f"/{end_point}", tags=[end_point])
# ✅ 허용된 명령어만 실행 (화이트리스트)
ALLOWED_COMMANDS = {
    "ls", "whoami", "uptime", "df", "free", "reboot", "shutdown"
}

def is_command_allowed(cmd: str) -> bool:
    parts = shlex.split(cmd)
    return parts[0] in ALLOWED_COMMANDS if parts else False

@router.get("/")
@router.get("")
def control_topic(
    cli: Optional[str] = Query(None, description="CLI command 1"),
    scr: Optional[str] = Query(None, description="script cmd in execution"),
):
    ret = False
    rMsg=  AlarmCodeList.OK.name
    try:
        if cli:
            ret = os.system(cli)
        elif scr:
            scr_dir = os.environ.get("SCR_DIR")
            if not scr_dir:
                rMsg = ALM_User.SCRIPT_ENV_NOT_FOUND.value
            else:
                # 스크립트가 허용된 명령어인지 확인
                script_path = os.path.join(scr_dir, scr)

                if not os.path.isfile(script_path):
                    rMsg = ALM_User.SCRIPT_NOT_FOUND.value
                # 실행 권한이 없다면 허용하지 않음
                elif not os.access(script_path, os.X_OK):
                    rMsg = ALM_User.NO_PERMISSION.value
                else:
                    with open("/dev/pts/0", "w") as pts:
                        process = subprocess.Popen(
                            f"{script_path} &",
                            shell=True,
                            stdout=pts,
                            stderr=pts
                        )
                    ret = True
                    rMsg = process.pid
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Execution failed: {str(e)}")    
    return {ret:rMsg }
    
