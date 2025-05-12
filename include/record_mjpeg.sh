#!/bin/bash

# 사용법 함수
usage() {
    echo "사용법: $0 {start|stop} [STREAM_URL] [SAVE_DIR]"
    echo "예시:"
    echo "  $0 start https://example.com:8080/stream /home/user/Videos"
    echo "  $0 stop"
    exit 1
}

# PID 저장 위치
PID_FILE="/tmp/mjpeg_recording.pid"

start_recording() {
    STREAM_URL="$1"
    SAVE_DIR="${2:-.}"

    if [ -z "$STREAM_URL" ]; then
        echo "에러: STREAM_URL이 필요합니다."
        usage
    fi

    TIMESTAMP=$(date +%Y%m%d_%H%M%S)
    OUTPUT_FILE="$SAVE_DIR/mjpeg_record_$TIMESTAMP.mp4"

    echo "녹화 시작: $OUTPUT_FILE"
    ffmpeg -use_wallclock_as_timestamps 1 -f mjpeg -i "$STREAM_URL" \
    -c:v libx264 -preset ultrafast -crf 23 -pix_fmt yuv420p "$OUTPUT_FILE" \
    > /dev/null 2>&1 &

    echo $! > "$PID_FILE"
    echo "PID: $(cat $PID_FILE)"
}

stop_recording() {
    if [ -f "$PID_FILE" ]; then
        PID=$(cat "$PID_FILE")
        echo "녹화 중지 (PID: $PID)..."
        kill "$PID" && rm -f "$PID_FILE"
    else
        echo "녹화 중인 프로세스가 없습니다."
    fi
}

# 명령 분기
case "$1" in
    start)
        start_recording "$2" "$3"
        ;;
    stop)
        stop_recording
        ;;
    *)
        usage
        ;;
esac
