#!/bin/bash

# 비디오 플레이어 프로세스가 실행 중인지 확인하는 함수
check_player() {
    pgrep -x mpv >/dev/null
    return $?
}

# 기존 플레이어 프로세스 종료
if check_player; then
    killall mpv
    sleep 1
fi

# MPV 실행을 위한 디스플레이 설정
DISPLAY=:0
# xhost를 사용하여 권한 부여
xhost +SI:localuser:$(whoami)


# 검은 화면 배경을 설정
xsetroot -solid black

# MPV 설정 파일 생성
cat > /tmp/mpv.conf << EOF
fullscreen=yes
border=no
keep-open=always
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

# 기본 재생 설정
loop-playlist=inf
reset-on-next-file=all

# 하드웨어 가속 비활성화
hwdec=vaapi 
vo=gpu
gpu-context=x11

# 간단한 전환 효과
video-sync=display-resample
EOF

# 재생목록 생성
find /root/Downloads -name "*.mp4" -type f > /tmp/playlist.txt

# MPV로 재생
exec mpv \
    --config-dir=/tmp \
    --playlist=/tmp/playlist.txt \
    --video-aspect-override=no \
    --hr-seek=yes \
    --hr-seek-framedrop=no \
    --video-latency-hacks=yes

# 임시 파일 정리
trap 'rm -f /tmp/mpv.conf /tmp/playlist.txt' EXIT
