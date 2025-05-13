#!/bin/bash

# feh 프로세스 종료
PIDS=$(pgrep feh)

if [ -z "$PIDS" ]; then
    echo "실행 중인 feh 슬라이드쇼가 없습니다."
else
    echo "feh 슬라이드쇼 종료 중..."
    kill $PIDS
    echo "종료 완료."
fi
