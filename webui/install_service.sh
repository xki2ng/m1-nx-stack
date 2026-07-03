#!/bin/bash
set -e
sudo cp /home/robot/webui/m1-webui.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl stop m1-webui 2>/dev/null
pkill -f control_server 2>/dev/null; sleep 1
sudo systemctl start m1-webui
sleep 3
echo "=== 状态 ==="
systemctl status m1-webui --no-pager | head -8
echo ""
curl -s --connect-timeout 3 http://localhost:8000/ | head -1 && echo "✅ WebUI OK"
