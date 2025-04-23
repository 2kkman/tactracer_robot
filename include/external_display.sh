#!/bin/bash
pulseaudio &
ufw allow from 172.30.1.0/24
#xrandr --newmode "400x1280_60.00"   42.00  400 424 464 528  1280 1283 1293 1327 -hsync +vsync
xrandr --newmode "400x1280_60.00"   40.54  400 432 472 544  1280 1281 1284 1325 -hsync +vsync
xrandr --addmode HDMI-2 "400x1280_60.00"
xrandr -s 400x1280 -r 60 --output HDMI-2 --preferred
xrandr -s 400x1280_60.00 -r 60 --output HDMI-2 --preferred
xinput set-prop "TeNizo TeNizo_R7Series_TC" --type=float "Coordinate Transformation Matrix" 0 1 0 -1 0 1 0 0 1
sleep 9
autorandr --change
xrandr -s 400x1280_60.00 -r 60 --output HDMI-2 --preferred