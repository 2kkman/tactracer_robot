#!/bin/bash

echo "========================"
echo " System Hardware Info"
echo "========================"

# CPU 정보
echo "🔹 CPU Information"
lscpu | grep -E "Model name|Architecture|CPU MHz|CPU max MHz|CPU min MHz" | grep -v -E "None|Unknown"

# RAM 정보
echo ""
echo "🔹 RAM Information"
RAM_INFO=$(free -h | awk 'NR==1{print $1, $2} NR==2{print $1, $2, "used:", $3, "available:", $7}')
[ -n "$RAM_INFO" ] && echo "$RAM_INFO"

RAM_DETAILS=$(dmidecode -t memory | grep -E "Size|Speed" | grep -v -E "No Module Installed|Unknown|None" | uniq)
[ -n "$RAM_DETAILS" ] && echo "$RAM_DETAILS"

# 저장 장치 정보
echo ""
echo "🔹 Storage Information"
STORAGE_INFO=$(lsblk -o NAME,SIZE,ROTA,TYPE,MOUNTPOINT | grep -E "disk|part" | grep -v "UNKNOWN")
[ -n "$STORAGE_INFO" ] && echo "$STORAGE_INFO"

echo ""
echo "SSD/HDD Details:"
for DISK in $(lsblk -nd -o NAME | awk '{print "/dev/"$1}'); do
    DRIVE_INFO=$(sudo smartctl -i "$DISK" 2>/dev/null | grep -E "Model Number|Serial Number|Rotation Rate" | grep -v -E "Unknown|None")
    [ -n "$DRIVE_INFO" ] && echo "$DRIVE_INFO"
done

# GPU 정보 (NVIDIA/AMD/Intel)
echo ""
echo "🔹 GPU Information"
GPU_INFO=$(lspci | grep -i --color 'vga\|3d\|2d' | grep -v -E "Unknown|None")
[ -n "$GPU_INFO" ] && echo "$GPU_INFO"

# 네트워크 인터페이스 정보
echo ""
echo "🔹 Network Interfaces"
NET_INFO=$(ip -brief address | grep -v "UNKNOWN")
[ -n "$NET_INFO" ] && echo "$NET_INFO"

# 운영체제 정보
echo ""
echo "🔹 OS Information"
OS_INFO=$(lsb_release -a 2>/dev/null && uname -r)
[ -n "$OS_INFO" ] && echo "$OS_INFO"
