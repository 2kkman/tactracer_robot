from fastapi import APIRouter, Query, HTTPException
from ros_publisher import publish_string
from UtilBLB import *
end_point = EndPoints.MANAGEMENT_SBC.name
router = APIRouter(prefix=f"/{end_point}", tags=[end_point])

@router.get("/")
@router.get("")
def control_topic(
    cli: str = Query(..., description="Data to publish as String")
):
    try:
        ret = os.system(cli)
        return {ret: AlarmCodeList.OK.name}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"ROS publish failed: {str(e)}")
