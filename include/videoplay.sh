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
# # 잠금 파일로 중복 실행 방지 
# LOCKFILE="/root/Downloads/mpv_player.lock" 
# if [ -e "$LOCKFILE" ]; then 
#     PID=$(cat $LOCKFILE)
#     if ps -p $PID > /dev/null; then
#         echo "$(date): 이미 PID $PID로 실행 중입니다." >> $LOGFILE
#         echo "이미 실행 중입니다. PID: $PID" 
#         exit 1 
#     else
#         echo "$(date): 오래된 잠금 파일 제거" >> $LOGFILE
#         rm -f $LOCKFILE
#     fi
# fi 
# # 현재 PID 기록
# echo $$ > "$LOCKFILE" 
# trap "rm -f $LOCKFILE /root/Downloads/mpv.conf /root/Downloads/playlist.txt; echo '$(date): 스크립트 종료' >> $LOGFILE" EXIT INT TERM
# MPV 실행을 위한 디스플레이 설정
DISPLAY=:0
# xhost를 사용하여 권한 부여
xhost +SI:localuser:$(whoami)


# 검은 화면 배경을 설정
xsetroot -solid black
# 시스템 자원 확보를 위한 캐시 정리
#echo "$(date): 시스템 캐시 정리" >> $LOGFILE
sync
echo 3 > /proc/sys/vm/drop_caches 2>/dev/null || echo "캐시 정리 권한 없음" >> $LOGFILE

# MPV 설정 파일 생성
cat > /tmp/mpv.conf << EOF
fullscreen=yes
border=no
keep-open=always
force-window=no
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

# 오디오 비활성화
no-audio

# 하드웨어 가속 (자동 감지 또는 no 로 설정해 테스트)
profile=low-latency
hwdec=no
scale=bilinear
dscale=bilinear
vo=gpu
gpu-context=x11

# 캐시 및 디코딩 리소스 줄이기
cache=yes
cache-secs=2
demuxer-max-bytes=20M
demuxer-max-back-bytes=5M
vd-lavc-threads=2
ad-lavc-threads=1
demuxer-lavf-o=lowres=1

# 비디오 동기화 최소화
video-sync=audio
framedrop=decoder
interpolation=no

# 간단한 전환 효과
video-sync=display-resample
EOF

# 재생목록 생성
find /root/Downloads -name "*.mp4" -type f > /tmp/playlist.txt

# MPV로 재생
#echo "$(date): MPV 실행" >> $LOGFILE
exec mpv \
    --config-dir=/tmp \
    --playlist=/tmp/playlist.txt \
    --video-aspect-override=no \
    --hr-seek=yes \
    --hr-seek-framedrop=no \
    --video-latency-hacks=yes \
    --hr-seek-framedrop=no

# 임시 파일 정리
trap 'rm -f /tmp/mpv.conf /tmp/playlist.txt' EXIT
