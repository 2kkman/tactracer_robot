from fastapi import APIRouter, Query, HTTPException
from ros_publisher import publish_string

router = APIRouter(prefix="/control", tags=["control"])

@router.get("/")
@router.get("")
def control_topic(
    topicname: str = Query(..., description="ROS topic name"),
    datastr: str = Query(..., description="Data to publish as String")
):
    try:
        result = publish_string(topicname, datastr)
        return {"status": "success", "message": result}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"ROS publish failed: {str(e)}")
