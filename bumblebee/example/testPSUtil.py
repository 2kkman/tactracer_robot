#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2009, Giampaolo Rodola'. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

"""
A clone of 'sensors' utility on Linux printing hardware temperatures.

$ python3 scripts/sensors.py
asus
    asus                 47.0 °C (high = None °C, critical = None °C)

acpitz
    acpitz               47.0 °C (high = 103.0 °C, critical = 103.0 °C)

coretemp
    Physical id 0        54.0 °C (high = 100.0 °C, critical = 100.0 °C)
    Core 0               47.0 °C (high = 100.0 °C, critical = 100.0 °C)
    Core 1               48.0 °C (high = 100.0 °C, critical = 100.0 °C)
    Core 2               47.0 °C (high = 100.0 °C, critical = 100.0 °C)
    Core 3               54.0 °C (high = 100.0 °C, critical = 100.0 °C)
"""

from __future__ import print_function

import collections
import os
import time
from psutil._compat import get_terminal_size
import sys
import psutil
cpu = psutil.cpu_freq()
print(cpu)  # cpu의 속도 출력
cpu=psutil.cpu_percent()
print('cpu usage :',cpu)
disk=psutil.disk_usage('/')
print('disk usage :', disk.used / 1024 / 1024 /1024 ,'%')
dic = {}

    
def getTemp():
    global dic
    if not hasattr(psutil, "sensors_temperatures"):
        sys.exit("platform not supported")
    temps = psutil.sensors_temperatures()
    if not temps:
        sys.exit("can't read any temperature")
    for name, entries in temps.items():
        print(name)
        for entry in entries:
            dic[entry.label or name] = entry.current
            print("    %-20s %s °C (high = %s °C, critical = %s °C)" % (
                entry.label or name, entry.current, entry.high,
                entry.critical))
        print()
    print(dic)
    
    disk = psutil.disk_partitions()
    for p in disk:
        print(p.mountpoint, p.fstype, end=' ')
        du = psutil.disk_usage(p.mountpoint)
        disk_total = round(du.total / 1024**3)
        if disk_total > 0:
            dic[p.mountpoint] = round(du.free / (2**30), 2)
        print(f'디스크크기: {disk_total}GB')
    
    return dic

if __name__ == '__main__':
    getTemp()