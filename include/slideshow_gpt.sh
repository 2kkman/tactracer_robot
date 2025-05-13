#!/bin/bash

# 사용법 안내
if [ $# -ne 2 ]; then
    echo "사용법: $0 <이미지_폴더_경로> <초_간격>"
    exit 1
fi

IMG_DIR="$1"
INTERVAL="$2"

# 유효성 검사
if [ ! -d "$IMG_DIR" ]; then
    echo "오류: 디렉터리가 존재하지 않습니다: $IMG_DIR"
    exit 1
fi

if ! [[ "$INTERVAL" =~ ^[0-9]+$ ]]; then
    echo "오류: 간격은 정수여야 합니다."
    exit 1
fi

# feh 설치 확인
if ! command -v feh &> /dev/null; then
    echo "'feh'가 설치되어 있지 않습니다. 설치 중..."
    sudo apt update && sudo apt install -y feh
fi

# 이미지 목록 확인
IMAGES=("$IMG_DIR"/*.jp*)

if [ ${#IMAGES[@]} -eq 0 ]; then
    echo "해당 폴더에 *.jp* 이미지가 없습니다."
    exit 1
fi

# 무한 반복 전체화면 슬라이드쇼 시작
feh --quiet --fullscreen --auto-zoom --zoom fill \
    --slideshow-delay "$INTERVAL" --loop "${IMAGES[@]}"
