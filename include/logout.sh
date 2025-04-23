#!/bin/bash

# 현재 터미널의 TTY를 가져옵니다.
current_tty=$(tty)

# 'who' 명령어를 사용하여 로그인한 사용자의 세션 정보를 가져옵니다.
# 그리고 'awk'를 사용하여 TTY 정보만 추출합니다.
sessions=$(who | awk '{print $2}')

# 각 세션에 대해 반복하여 현재 세션을 제외한 모든 세션을 종료합니다.
for sess in $sessions; do
    if [ "/dev/$sess" != "$current_tty" ]; then
        # 'pkill'을 사용하여 세션에 연결된 모든 프로세스를 종료합니다.
        echo "Logging out session /dev/$sess"
        sudo pkill -9 -t $sess
    fi
done

echo "All other sessions have been logged out."
