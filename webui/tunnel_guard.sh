#!/bin/bash
# SSH 反向隧道守护 — 断开自动重连
# 用法: bash tunnel_guard.sh

TARGET="admin@39.105.71.173"
PASS="Pi3.1514927"

while true; do
    echo "[$(date '+%H:%M:%S')] 建立隧道..."
    sshpass -p "$PASS" ssh \
        -o StrictHostKeyChecking=no \
        -o ServerAliveInterval=30 \
        -o ServerAliveCountMax=3 \
        -o ExitOnForwardFailure=yes \
        -N \
        -R 8000:localhost:8000 \
        -R 9090:localhost:9091 \
        "$TARGET" 2>&1
    echo "[$(date '+%H:%M:%S')] 隧道断开，5秒后重连..."
    sleep 5
done
