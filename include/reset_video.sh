#!/bin/bash

# 현재 스크립트의 절대 경로 기준 디렉토리 구하기
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# a.sh, b.sh를 해당 디렉토리 기준으로 실행
"$SCRIPT_DIR/videostop.sh"
"$SCRIPT_DIR/videoplay.sh"
