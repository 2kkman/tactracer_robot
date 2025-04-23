
from dataclasses import dataclass

@dataclass
class BLB_CMD:
    DEVICEID: str
    TRAY: str
    LEVEL: str
    STATE: str
    TIME: str