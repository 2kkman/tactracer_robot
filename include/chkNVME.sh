#!/bin/bash

# NVMe 디바이스 개수 확인
NVME_COUNT=$(lsblk -d -n -o NAME | grep -c '^nvme')

# 추가적으로 PCIe 슬롯에서 NVMe 장치 정보 확인
PCI_NVME_COUNT=$(lspci | grep -i nvme | wc -l)

# /sys/class/nvme 기반으로 NVMe 디바이스 수 확인
SYS_NVME_COUNT=$(ls /sys/class/nvme | wc -l)

echo "==== NVMe 슬롯 개수 확인 ===="
echo "lsblk 기반 NVMe 디바이스 개수: $NVME_COUNT"
echo "lspci 기반 NVMe 컨트롤러 개수: $PCI_NVME_COUNT"
echo "/sys/class/nvme 기반 NVMe 개수: $SYS_NVME_COUNT"

# 최종 판단
if [ "$NVME_COUNT" -gt "$PCI_NVME_COUNT" ]; then
    echo "사용 가능한 NVMe 장치 개수: $NVME_COUNT"
else
    echo "사용 가능한 NVMe 컨트롤러 개수: $PCI_NVME_COUNT"
fi

