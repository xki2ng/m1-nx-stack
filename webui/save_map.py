#!/usr/bin/env python3
"""地图保存 — rclpy 订阅 /Laser_map 一帧保存 PCD"""
import os, time, sys, json, struct

MAP_DIR = os.path.expanduser("~/webui/data/maps")
os.makedirs(MAP_DIR, exist_ok=True)
ts = os.environ.get("MAP_NAME", "").strip() or time.strftime("%Y%m%d_%H%M%S")
name = f"map_{ts}"
pcd_path = os.path.join(MAP_DIR, f"{name}.pcd")

try:
    sys.path.insert(0, '/opt/ros/humble/lib/python3.10/site-packages')
    import rclpy
    from sensor_msgs.msg import PointCloud2
    rclpy.init(args=[])
    node = rclpy.create_node('pcd_saver')
    latest = [None]

    def cb(msg):
        latest[0] = msg

    node.create_subscription(PointCloud2, '/Laser_map', cb, 10)
    deadline = time.time() + 5.0
    while time.time() < deadline and latest[0] is None:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()

    msg = latest[0]
    if msg is None:
        print(json.dumps({"ok": False, "error": "no /Laser_map data"}))
        sys.exit(1)

    fields = {f.name: f.offset for f in msg.fields}
    ps = msg.point_step
    n = msg.width * msg.height
    data = msg.data
    ox, oy, oz = fields['x'], fields['y'], fields['z']
    oi = fields.get('intensity', oz)

    lines = [
        "# .PCD v0.7", "VERSION 0.7",
        "FIELDS x y z intensity normal_x normal_y normal_z curvature",
        "SIZE 4 4 4 4 4 4 4 4",
        "TYPE F F F F F F F F",
        "COUNT 1 1 1 1 1 1 1 1",
        f"WIDTH {n}", "HEIGHT 1",
        "VIEWPOINT 0 0 0 1 0 0 0",
        f"POINTS {n}", "DATA ascii",
    ]
    for i in range(n):
        base = i * ps
        x = struct.unpack_from('f', data, base + ox)[0]
        y = struct.unpack_from('f', data, base + oy)[0]
        z = struct.unpack_from('f', data, base + oz)[0]
        intensity = struct.unpack_from('f', data, base + oi)[0] if 'intensity' in fields else 0.0
        lines.append(f"{x:.4f} {y:.4f} {z:.4f} {intensity:.1f} 0.0 0.0 0.0 0.0")

    with open(pcd_path, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print(json.dumps({"ok": True, "name": name}))

except Exception as e:
    print(json.dumps({"ok": False, "error": str(e)}))
