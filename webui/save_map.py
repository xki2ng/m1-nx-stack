#!/usr/bin/env python3
"""独立地图保存脚本 — ros2 bag record"""
import subprocess, json, os, time, sys

MAP_DIR = os.path.expanduser("~/webui/data/maps")
os.makedirs(MAP_DIR, exist_ok=True)

ts = os.environ.get("MAP_NAME", "").strip() or time.strftime("%Y%m%d_%H%M%S")
name = f"map_{ts}"
bag_path = os.path.join(MAP_DIR, name)

# 用 bash -c source 确保完整 ROS2 环境
cmd = f"source /opt/runtime/env.bash && ros2 bag record -o {bag_path} /Laser_map --max-bag-duration 2"
try:
    subprocess.run(["bash", "-c", cmd], timeout=15)
except subprocess.TimeoutExpired:
    pass

# 检查结果
ok = False
if os.path.isdir(bag_path):
    if any(f.endswith('.db3') for f in os.listdir(bag_path)):
        ok = True
        # 也复制 pcd（如果 FAST-LIO pcd_save_en 开启）
        pcd_path = os.path.expanduser("~/map.pcd")
        if os.path.exists(pcd_path):
            import shutil
            shutil.copy(pcd_path, os.path.join(MAP_DIR, f"{name}.pcd"))

if ok:
    print(json.dumps({"ok": True, "name": name}))
else:
    print(json.dumps({"ok": False, "error": "bag record failed"}))
