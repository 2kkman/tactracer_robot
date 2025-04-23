#!/bin/bash
killall -9 mpv
mpv --fs --loop-playlist=inf $1