from fastapi import APIRouter, Query, HTTPException
import mysql
from ros_publisher import publish_string
from UtilBLB import *

end_point = 'DBQ'
router = APIRouter(prefix=f"/{end_point}", tags=[end_point])
# ✅ 허용된 명령어만 실행 (화이트리스트)
ALLOWED_COMMANDS = {
    "ls", "whoami", "uptime", "df", "free", "reboot", "shutdown"
}

def get_stored_procedures(hostName='172.30.1.4', userName='tact', pw='bconnect@123', db_name='TactServer'):
    # MySQL 연결 설정
    conn = mysql.connector.connect(
        host=hostName,
        user=userName,
        password=pw,
        database=db_name
    )
    cursor = conn.cursor(dictionary=True)

    # 프로시저 목록 가져오기
    cursor.execute("SELECT SPECIFIC_NAME AS procedure_name FROM information_schema.ROUTINES WHERE ROUTINE_TYPE='PROCEDURE' AND ROUTINE_SCHEMA=%s", (conn.database,))
    procedures = cursor.fetchall()

    result = []

    # 각 프로시저의 파라미터 정보 가져오기
    for proc in procedures:
        proc_name = proc['procedure_name']
        cursor.execute("""
            SELECT PARAMETER_NAME, DATA_TYPE, DTD_IDENTIFIER, PARAMETER_MODE
            FROM information_schema.PARAMETERS
            WHERE SPECIFIC_NAME = %s AND SPECIFIC_SCHEMA = %s
        """, (proc_name, conn.database))
        params = cursor.fetchall()
        result.append({
            "procedure_name": proc_name,
            "parameters": params
        })

    cursor.close()
    conn.close()
    return result

def is_command_allowed(cmd: str) -> bool:
    parts = shlex.split(cmd)
    return parts[0] in ALLOWED_COMMANDS if parts else False

@router.get("/")
@router.get("")
def control_topic(
    cli: Optional[str] = Query(None, description="CLI command 1"),
    scr: Optional[str] = Query(None, description="script cmd in execution"),
    tts: Optional[str] = Query(None, description="Text to Speech"),
):
    ret = False
    rMsg=  AlarmCodeList.OK.name
    try:
        if cli:
            ret = os.system(cli)
        elif tts:
            ret = True
            rMsg= tts_service.handle_tts(tts, True)
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
        else:
            return get_stored_procedures()
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Execution failed: {str(e)}")    
    return {ret:rMsg }
    
