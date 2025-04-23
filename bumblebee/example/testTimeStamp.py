#! /usr/bin/env python3
import tzlocal
import datetime

local_tz = tzlocal.get_localzone()
dt = datetime.datetime.now(local_tz)
#dt = datetime.datetime.now().timestamp()
#dt = datetime.datetime.now()
print(dt)
print(dt.isoformat(timespec="seconds"))