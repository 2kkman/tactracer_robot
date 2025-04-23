#!/bin/bash
#set -xv

# 결과를 저장할 배열 선언
declare -a active_nics

# ip 명령어로 모든 인터페이스 정보를 가져옴
while IFS= read -r line; do
    # 인터페이스 이름이 포함된 줄 찾기
    if [[ $line =~ ^[0-9]+:\ ([^:]+): ]]; then
        current_nic="${BASH_REMATCH[1]}"
        
        # lo(루프백) 인터페이스 제외
        if [[ "$current_nic" != "lo" ]]; then
            # UP 상태이고 IP가 할당된 인터페이스 확인
            ip_addr=$(ip addr show "$current_nic" | grep "inet " | grep -v "scope link" | awk '{print $2}')
            if [[ -n "$ip_addr" ]] && ip link show "$current_nic" | grep -q "UP"; then
                active_nics+=("$current_nic")
            fi
        fi
    fi
done < <(ip link show)

# 결과 출력
if [ ${#active_nics[@]} -eq 0 ]; then
    echo "IP가 할당된 활성 네트워크 인터페이스를 찾을 수 없습니다."
    # e로 시작하는 모든 네트워크 인터페이스 찾기
    for interface in $(ip -o link show | awk -F': ' '{print $2}' | grep '^e'); do
        # 해당 인터페이스가 DOWN 상태인 경우
        if [[ $(ip link show "$interface" | grep 'state DOWN') ]]; then
            echo "$interface is down. Bringing it up..."
            ip link set "$interface" up
            sleep 5
        fi
        
        # 물리적으로 연결되어 있는지 확인
        if ethtool "$interface" | grep -q "Link detected: yes"; then
            echo "$interface is physically connected. Requesting IP via DHCP..."
            dhclient "$interface"
        else
            echo "$interface is not physically connected. Skipping DHCP request."
        fi
    done
fi

echo "활성 네트워크 인터페이스 목록:"
for nic in "${active_nics[@]}"; do
    ip_addr=$(ip addr show "$nic" | grep "inet " | grep -v "scope link" | awk '{print $2}')
    echo "$nic (IP: $ip_addr)"
done

export PKG_CONFIG_PATH=/usr/lib/x86_64-linux-gnu/pkgconfig/pcl_common-1.10.pc:$PKG_CONFIG_PATH
param=$(echo "$1" | tr '[:lower:]' '[:upper:]')
param_master=$(echo "$2" | tr '[:lower:]' '[:upper:]')
#echo $0
#echo $1
echo $param
full_path=$(realpath "${BASH_SOURCE[0]}")
script_dir=$(dirname "$full_path")
#source $script_dir/start_common.sh
# n을 설정 (여기서는 2)
n=2
# '/'를 기준으로 경로를 분할하고 처음 n개의 디렉토리를 추출
IFS='/' read -ra ADDR <<< "$script_dir"
WS_PATH=""
for ((i=1; i<=n; i++)); do
    WS_PATH+="/${ADDR[i]}"
done

m=4
# '/'를 기준으로 경로를 분할하고 처음 n개의 디렉토리를 추출
IFS='/' read -ra ADDR <<< "$script_dir"
export TACT_PATH=""
for ((i=1; i<=m; i++)); do
    TACT_PATH+="/${ADDR[i]}"
done

export WS_SRC="${WS_PATH}/src"
export SCR_DIR=$script_dir

# Ethernet 인터페이스 확인 및 IP 주소 확인
ethernet_nic=$(ip -o link show | awk -F': ' '{print $2}' | grep -v lo | (grep '^e' || true) | head -n 1)
ethernet_ip_addr=""
#!/bin/bash

if [[ -n $ethernet_nic ]]; then
    # Ethernet 인터페이스가 있는 경우 물리적 연결 상태 확인
    link_state=$(cat /sys/class/net/$ethernet_nic/operstate)
    if [[ "$link_state" == "up" ]]; then
        # 최대 10초간 IP가 할당될 때까지 대기
        for i in {1..10}; do
            ethernet_ip_addr=$(ip -o -4 addr list $ethernet_nic | awk '{print $4}' | cut -d/ -f1)
            if [[ -n $ethernet_ip_addr && "$ethernet_ip_addr" != "127.0.0.1" ]]; then
                break
            fi
            echo "Waiting for IP address on $ethernet_nic..."
            sleep 1
        done
    fi
fi

# Ethernet 인터페이스가 있고 IP 주소가 할당된 경우
if [[ -n $ethernet_ip_addr && "$ethernet_ip_addr" != "127.0.0.1" ]]; then
    nic=$ethernet_nic
    ip_addr=$ethernet_ip_addr
else
    # Ethernet 인터페이스가 없거나 IP 주소가 할당되지 않은 경우 Wi-Fi 인터페이스 확인
    nic=$(ip -o link show | awk -F': ' '{print $2}' | grep -v lo | (grep '^w' || true) | head -n 1)
    
    # Wi-Fi 인터페이스가 있는 경우 IP 주소가 할당될 때까지 대기
    while true; do
        ip_addr=$(ip -o -4 addr list $nic | awk '{print $4}' | cut -d/ -f1)
        if [[ -n $ip_addr && "$ip_addr" != "127.0.0.1" ]]; then
            break
        fi
        echo "Waiting for IP address on $nic..."
        sleep 1
    done
fi

echo "NIC: $nic, IP Address: $ip_addr"

export CONFIG=$param
export HOST=$(hostname)
export ROS_HOSTNAME="${ip_addr}"
export ROS_MASTER_URI="http://${ip_addr}:11311"
export BOOTPY_SRC="${script_dir}/boot_qbi.py -&"
#실제 운영모드 QBI
if [ "$CONFIG" == "QBI" ]; then
  if [ ${#param_master} -gt 1 ]; then
    echo "Set Master URI to $param_master"
    export ROS_MASTER_URI="http://${param_master}:11311"
  fi
  #export ROS_MASTER_URI=http://172.30.1.40:11311
  #export ROS_MASTER_URI=http://ROSITX:11311
#실제 운영모드 + 개발모드 겸용 ITX
elif [ "$CONFIG" == "ITX" ]; then
  #export ROS_MASTER_URI=http://ROSITX:11311
  export BOOTPY_SRC="${script_dir}/boot_itx.py -&"
  #export ROS_HOSTNAME="ROSITX"
#개발모드 DEV
else
  #export CONFIG=DEV
  export BOOTPY_SRC="${script_dir}/boot_dev.py -&"
  if [ ${#param_master} -gt 1 ]; then
    echo "Set Master URI to $param_master"
    export ROS_MASTER_URI="http://${param_master}:11311"
  fi

fi

# # 호스트네임 설정
# hostname_to_ping="ROSITX.local"

# # ping 명령어 실행 및 결과 확인
# ping -c 4 $hostname_to_ping &> /dev/null

# # ping 명령의 성공 여부에 따라 메시지 출력
# if [ $? -eq 0 ]; then
#     echo "Ping to $hostname_to_ping - Success"
#     export ROS_MASTER_URI=http://$hostname_to_ping:11311
# else
#     echo "Ping to $hostname_to_ping - Failed"
# fi

# 결과 출력
echo "MASTER ADDRESS param : $param_master"
echo "Now execute : $full_path"
echo "GetMachineStr() : $param"
echo "Tactracer Path : $TACT_PATH"
echo "HOSTNAME : $HOST"
echo "SCR_DIR : $SCR_DIR"
echo "WS_PATH : $WS_PATH"
echo "ROS_MASTER_URI : $ROS_MASTER_URI"
echo "ROS_HOSTNAME : $ROS_HOSTNAME"
echo "BOOTPY_SRC : $BOOTPY_SRC"
echo "DISPLAY : $DISPLAY"

#export WS_PATH=~/catkin_ws/
source /opt/ros/noetic/setup.bash
source $WS_PATH/devel/setup.bash
#export ROS_PACKAGE_PATH=~/catkin_ws/:$ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=$WS_PATH:$ROS_PACKAGE_PATH
export PYTHONPATH=$PYTHONPATH:$script_dir
export ROS_LOG_DIR="${WS_PATH}/log/"
echo "ROS_PACKAGE_PATH : $ROS_PACKAGE_PATH"
echo "ROS_LOG_DIR : $ROS_LOG_DIR"
#source ~/ydlidar_ros_ws/devel/setup.bash
#source ~/catkin_ws/devel/setup.bash
#source /opt/ros/noetic/share/catkin/cmake/devel/setup.sh
export ROSCONSOLE_FORMAT='[${severity}][${file}][${line}][${time:%m-%d %H:%M:%S]}${message}'
#export ROSCONSOLE_FORMAT='[${severity}][${file}][${line}][${time:%Y-%m-%d %H:%M:%S.$f]}${message}'
#export ROSCONSOLE_FORMAT='[${severity}] [${time, seconds_since_epoch}%f]: ${message}'
#export DISPLAY="localhost:10.0"
alias hh="history"
alias nb="nano ~/.bashrc"
alias sb="source ~/.bashrc"
alias cw='cd '$WS_PATH
alias cs='cd '$WS_SRC
alias startitx='roslaunch tta_blb BLB_start_itx.launch'
alias startqbi='roslaunch tta_blb BLB_start_qbi.launch'
alias tt='cd $script_dir ;./run_gui_ardUI.sh'
alias rt='cd $script_dir ;./run_tester.sh'
alias rg='cd $script_dir ;./run_gui.sh'
alias ci='cd '$script_dir
#alias mdif='killall -9 python3;cd $script_dir;cd ../script;python3 modbus_IF.py'
alias cm='cd ${WS_PATH} && catkin_make'
alias kk='killall -9 python3 ; killall -9 rosmaster ; killall -9 rosout ; killall -9 roscore'
alias rr='killall -9 python3 ; killall -9 rosmaster ; killall -9 rosout ; killall -9 roscore ; roslaunch rosbridge_server rosbridge_websocket.launch'
alias rrmb='killall -9 python3 ; killall -9 rosmaster ; killall -9 rosout ; killall -9 roscore ; roslaunch tta_blb BLB_MODBUS_FAST.launch'
alias rrcenter='killall -9 python3 ; killall -9 rosmaster ; killall -9 rosout ; killall -9 roscore ; roslaunch tta_blb BLB_MODBUS_CTL.launch'
alias rard='cd "$SCR_DIR" && ./reset_ard.sh'
alias kard='cd "$SCR_DIR" && ./kill_ard.sh'
alias ul='ls /dev/ttyU* -la'
alias rs='rosservice list'
alias rc='rosnode cleanup'
alias uul='ls /dev/ttC* -la'
alias 88='vi '$full_path    #start_common_scr.sh 수정 88
alias 77='vi /etc/samba/smb.conf';service smbd restart  #삼바SMB 설정 수정 77
alias 99='vi /etc/udev/rules.d/99-usb-serial.rules' #usb rule 수정 99
alias cls='clear'
alias vv='cd "$SCR_DIR" && ./videoplay.sh'
alias vs='cd "$SCR_DIR" && ./videostop.sh'
alias rsd='cd "$SCR_DIR" && ./reset_sd.sh'
alias rsse='cd "$SCR_DIR" && ./reset_sse.sh 6001 9000 9001 6000'
alias ksse='cd "$SCR_DIR" && ./kill_sse.sh 6001 9000 9001 6000'

alias rse='cd "$SCR_DIR" && ./reset_sse.sh 6001'
alias kse='cd "$SCR_DIR" && ./kill_sse.sh 6001'

alias rblb='cd "$SCR_DIR" && ./reset_blbsvr.sh 9000'
alias kblb='cd "$SCR_DIR" && ./kill_sse.sh 9000'

alias rrfid='cd "$SCR_DIR" && ./reset_rfid.sh 6000'
alias krfid='cd "$SCR_DIR" && ./kill_sse.sh 6000'
alias kmb='cd "$SCR_DIR" && ./kill_mb.sh'
alias rmb='cd "$SCR_DIR" && ./reset_modbus.sh'
alias rbms='cd "$SCR_DIR" && ./reset_bms.sh'
alias edp='export DISPLAY="localhost:10.0"'
alias tempc='cat /sys/class/thermal/thermal_zone0/temp'
#ufw allow from 172.30.1.0/24
#xinput set-prop "TeNizo TeNizo_R7Series_TC" --type=float "Coordinate Transformation Matrix" 0 1 0 -1 0 1 0 0 1
#echo "Call script"
#python3 /root/catkin_ws/test.py
#/root/catkin_ws/src/tta_blb/scripts/touch.sh
alias rl='rostopic list'
alias rp='rosclean purge -y'
python3 $BOOTPY_SRC
