#!/bin/bash

# 잠금 파일로 중복 실행 방지
LOCKFILE="/tmp/mpv_player.lock"
if [ -e "$LOCKFILE" ]; then
    echo "이미 실행 중입니다."
    exit 1
fi

touch "$LOCKFILE"
trap "rm -f $LOCKFILE /tmp/mpv.conf /tmp/playlist.txt" EXIT

# DISPLAY 설정
export DISPLAY=:0

# xhost 권한 부여
xhost +SI:localuser:$(whoami)

# 검은 배경
xsetroot -solid black

# MPV 설정파일 생성
cat > /tmp/mpv.conf << EOF
fullscreen=yes
border=no
keep-open=yes
force-window=yes
keepaspect=no
cursor-autohide=always
osd-level=0
osc=no
background="#000000"
geometry=100%x100%
video-aspect-override=no
video-unscaled=no
input-conf=/dev/null
no-input-default-bindings

loop-playlist=inf
reset-on-next-file=all

hwdec=vaapi
vo=gpu
gpu-context=x11

video-sync=display-resample
EOF

# 재생목록 생성
find /root/Downloads -type f \( -iname "*.mp4" -o -iname "*.mkv" -o -iname "*.avi" \) > /tmp/playlist.txt

# 파일이 없을 경우 종료
if [ ! -s /tmp/playlist.txt ]; then
    echo "재생할 영상이 없습니다."
    exit 1
fi

# MPV 실행
mpv --config-dir=/tmp \
    --playlist=/tmp/playlist.txt
