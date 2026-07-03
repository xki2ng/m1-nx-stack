#!/usr/bin/env python3
"""独立地图保存脚本 — ros2 bag record"""
import subprocess, json, os, time, sys

MAP_DIR = os.path.expanduser("~/webui/data/maps")
os.makedirs(MAP_DIR, exist_ok=True)

ts = os.environ.get("MAP_NAME", "").strip() or time.strftime("%Y%m%d_%H%M%S")
name = f"map_{ts}"
bag_path = os.path.join(MAP_DIR, name)

env = os.environ.copy()
env["RMW_IMPLEMENTATION"] = "rmw_zenoh_cpp"
env["ROS_DOMAIN_ID"] = "66"

try:
    subprocess.run(
        ["ros2", "bag", "record", "-o", bag_path, "/Laser_map", "--max-bag-duration", "2"],
        env=env, timeout=15
    )
except subprocess.TimeoutExpired:
    pass

if os.path.exists(bag_path) or any(f.endswith('.db3') for f in os.listdir(bag_path) if os.path.isdir(bag_path)):
    # 也检查 map.pcd 是否存在（FAST-LIO pcd_save_en）
    pcd_path = os.path.expanduser("~/map.pcd")
    if os.path.exists(pcd_path):
        import shutil
        shutil.copy(pcd_path, os.path.join(MAP_DIR, f"{name}.pcd"))
    print(json.dumps({"ok": True, "name": name}))
else:
    print(json.dumps({"ok": False, "error": "bag record failed"}))
