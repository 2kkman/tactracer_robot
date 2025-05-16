#!/bin/bash

# 사용법 체크
if [ $# -lt 2 ]; then
    echo "사용법: $0 <이미지_폴더_경로> <표시_간격(초)>"
    exit 1
fi

FOLDER_PATH="$1"
INTERVAL="$2"

# 폴더 존재 여부 확인
if [ ! -d "$FOLDER_PATH" ]; then
    echo "오류: 폴더 '$FOLDER_PATH'가 존재하지 않습니다."
    exit 1
fi

# interval이 숫자인지 확인
if ! [[ "$INTERVAL" =~ ^[0-9]+(\.[0-9]+)?$ ]]; then
    echo "오류: 간격은 숫자여야 합니다."
    exit 1
fi

# feh 설치 여부 확인 및 설치
if ! command -v feh &> /dev/null; then
    echo "feh 이미지 뷰어가 설치되어 있지 않습니다. 설치를 시작합니다..."
    sudo apt-get update
    sudo apt-get install -y feh
    
    if [ $? -ne 0 ]; then
        echo "feh 설치에 실패했습니다. 수동으로 설치해주세요."
        exit 1
    fi
fi

# 이미지 파일 검색 (*.jpg, *.jpeg, *.jpe, *.jp2 등 모든 jp*를 포함)
IMAGE_FILES=()
while IFS= read -r -d $'\0' file; do
    IMAGE_FILES+=("$file")
done < <(find "$FOLDER_PATH" -type f -name "*.jp*" -print0 | sort -z)

# 이미지 파일 존재 여부 확인
if [ ${#IMAGE_FILES[@]} -eq 0 ]; then
    echo "오류: '$FOLDER_PATH' 폴더에 *.jp* 파일이 없습니다."
    exit 1
fi

echo "총 ${#IMAGE_FILES[@]}개의 이미지 파일을 찾았습니다."
echo "슬라이드쇼를 시작합니다... (종료하려면 q 키를 누르세요)"
echo "각 이미지는 $INTERVAL초 동안 표시됩니다."

# feh로 슬라이드쇼 실행
# -F: 전체화면 모드
# -Z: 이미지를 화면에 맞게 자동 확대 (스트레치)
# -D: 각 이미지 표시 시간 설정
# --cycle-once: 한 번 순환 후 종료 (제거하면 반복)
# --hide-pointer: 마우스 포인터 숨기기
#!/bin/bash
export DISPLAY=:0
export XAUTHORITY=/root/.Xauthority
xhost +SI:localuser:$(whoami)
feh -F -Z -D "$INTERVAL" --auto-rotate --cycle --hide-pointer "${IMAGE_FILES[@]}"