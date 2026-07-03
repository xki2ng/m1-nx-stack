#!/usr/bin/env python3
"""
M1 WebUI 后端 API Server
端口 8001，处理状态、路径、轨迹、地图、服务控制
"""
import json, os, sys, time, signal, glob, subprocess
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn
from urllib.parse import urlparse, parse_qs

class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True

PORT = 8000
DATA_DIR = os.path.expanduser("~/webui/data")
PATH_FILE = os.path.join(DATA_DIR, "gps_path.json")
TRAJ_DIR = os.path.join(DATA_DIR, "trajectories")
MAP_DIR = os.path.join(DATA_DIR, "maps")
FASTLIO_DIR = os.path.expanduser("~/fastlio")
WEBUI_DIR = os.path.dirname(os.path.abspath(__file__))  # 静态文件目录

os.makedirs(TRAJ_DIR, exist_ok=True)
os.makedirs(MAP_DIR, exist_ok=True)

# ── 进程状态检测 ──
def check_process(name, patterns):
    try:
        for pat in patterns:
            result = subprocess.run(
                ["pgrep", "-f", pat],
                capture_output=True, text=True, timeout=3
            )
            if result.stdout.strip():
                pids = result.stdout.strip().split()
                return {"running": True, "pid": int(pids[0])}
        return {"running": False, "pid": None}
    except Exception:
        return {"running": False, "pid": None}


def check_zenoh():
    """检查 Zenoh router 运行状态 + 跨机 peering"""
    router = check_process("zenohd", ["rmw_zenohd"])
    # 检查跨机 peering: 看是否有对端连接 (M1: 192.168.168.168, 工作站: 192.168.168.50)
    peers = []
    try:
        r = subprocess.run(["ss", "-tnp"], capture_output=True, text=True, timeout=3)
        for line in r.stdout.splitlines():
            if ":7447" in line and "ESTAB" in line:
                if "192.168.168.168" in line:
                    peers.append("M1")
                elif "192.168.168.50" in line:
                    peers.append("工作站")
    except:
        pass
    return {
        "running": router["running"],
        "pid": router["pid"],
        "peers": list(set(peers))
    }


def get_all_status():
    return {
        "rosbridge": check_process("rosbridge", ["rosbridge_websocket"]),
        "controller": check_process("controller", ["m1_full_controller"]),
        "webui": {"running": True, "pid": None},
        "zenoh": check_zenoh(),
        "fastlio": check_process("FAST-LIO", ["fastlio_mapping"]),
        "nvblox": check_process("nvblox", ["m1_nvblox_br"]),
        "nav2": check_process("nav2", ["bt_navigator"]),
        "voxel": check_process("voxel", ["m1_voxel_flt"]),
    }


# ── 服务控制 ──
def start_voxel():
    """启动降采样节点"""
    env = os.environ.copy()
    env["RMW_IMPLEMENTATION"] = "rmw_zenoh_cpp"
    env["ROS_DOMAIN_ID"] = "66"
    subprocess.run(["pkill", "-f", "m1_voxel_flt"], capture_output=True)
    time.sleep(1)
    voxel_cmd = [
        "/home/robot/m1_ws/build/m1_voxel_filter/m1_voxel_flt",
        "--ros-args",
        "-p", "input_topic:=/front_lidar",
        "-p", "output_topic:=/front_lidar/filtered",
        "-p", "leaf_size:=0.05",
        "-r", "__name:=voxel_front"
    ]
    subprocess.Popen(voxel_cmd, env=env,
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return True

def stop_voxel():
    subprocess.run(["pkill", "-f", "m1_voxel_flt"], capture_output=True)
    return True


def start_fastlio():
    """启动 FAST-LIO（降采样需先启动）"""
    env = os.environ.copy()
    env["RMW_IMPLEMENTATION"] = "rmw_zenoh_cpp"
    env["ROS_DOMAIN_ID"] = "66"

    subprocess.run(["pkill", "-f", "fastlio_mapping"], capture_output=True)
    time.sleep(1)

    fastlio_cmd = [
        "/home/robot/m1_ws/install/fast_lio/lib/fast_lio/fastlio_mapping",
        "--ros-args",
        "--params-file", f"{FASTLIO_DIR}/m1_airy96_rot.yaml"
    ]
    subprocess.Popen(fastlio_cmd, env=env,
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return True


def stop_fastlio():
    subprocess.run(["pkill", "-f", "fastlio_mapping"], capture_output=True)
    return True


def start_nvblox():
    env = os.environ.copy()
    env["RMW_IMPLEMENTATION"] = "rmw_zenoh_cpp"
    env["ROS_DOMAIN_ID"] = "66"
    nvblox_cmd = [
        "/home/robot/m1_ws/install/nvblox_fastlio_bridge/lib/nvblox_fastlio_bridge/m1_nvblox_br",
        "--ros-args",
        "--params-file", "/home/robot/m1_ws/install/nvblox_fastlio_bridge/share/nvblox_fastlio_bridge/config/nvblox_airy96.yaml"
    ]
    subprocess.run(["pkill", "-f", "m1_nvblox_br"], capture_output=True)
    time.sleep(1)
    subprocess.Popen(nvblox_cmd, env=env,
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return True


def stop_nvblox():
    subprocess.run(["pkill", "-f", "m1_nvblox_br"], capture_output=True)
    return True


# ── 路径管理 ──
def load_path():
    if os.path.exists(PATH_FILE):
        with open(PATH_FILE) as f:
            return json.load(f)
    return []

def save_path(points):
    with open(PATH_FILE, "w") as f:
        json.dump(points, f)

def clear_path():
    with open(PATH_FILE, "w") as f:
        json.dump([], f)


# ── 轨迹管理 ──
def list_trajectories():
    trajs = []
    for fname in sorted(os.listdir(TRAJ_DIR), reverse=True):
        if fname.endswith(".json"):
            fpath = os.path.join(TRAJ_DIR, fname)
            try:
                with open(fpath) as f:
                    pts = json.load(f)
                good = sum(1 for p in pts if p.get("quality", "good") == "good")
                dist = 0.0
                for i in range(1, len(pts)):
                    lng1, lat1 = pts[i-1]["lng"], pts[i-1]["lat"]
                    lng2, lat2 = pts[i]["lng"], pts[i]["lat"]
                    dist += haversine(lng1, lat1, lng2, lat2)
                trajs.append({
                    "name": fname.replace(".json", ""),
                    "points": len(pts), "good": good,
                    "distance": round(dist, 1),
                    "duration": round((pts[-1].get("time",0)-pts[0].get("time",0))/1000,1) if len(pts)>=2 else 0,
                    "ts": pts[0].get("time",0) if pts else 0, "error": False,
                })
            except:
                trajs.append({"name": fname.replace(".json",""), "points":0,"good":0,
                              "distance":0,"duration":0,"ts":0,"error":True})
    return trajs

def save_trajectory(name):
    pts = load_path()
    if not pts: return False, "no path data"
    if not name: name = time.strftime("%Y%m%d_%H%M%S")
    fpath = os.path.join(TRAJ_DIR, f"{name}.json")
    with open(fpath, "w") as f: json.dump(pts, f)
    return True, name

def load_trajectory(name):
    fpath = os.path.join(TRAJ_DIR, f"{name}.json")
    if not os.path.exists(fpath): return False
    with open(fpath) as f: pts = json.load(f)
    save_path(pts)
    return True

def delete_trajectory(name):
    fpath = os.path.join(TRAJ_DIR, f"{name}.json")
    if os.path.exists(fpath): os.remove(fpath); return True
    return False


# ── 地图管理 ──
MAP_SAVE_PATH = os.path.expanduser("~/map.pcd")

def list_maps():
    maps = []
    seen = set()
    for fname in sorted(os.listdir(MAP_DIR), reverse=True):
        fpath = os.path.join(MAP_DIR, fname)
        # PCD 文件
        if fname.endswith(".pcd"):
            stat = os.stat(fpath)
            maps.append({
                "name": fname.replace(".pcd", ""),
                "size": stat.st_size,
                "size_mb": round(stat.st_size / (1024*1024), 2),
                "ts": int(stat.st_mtime),
                "date": time.strftime("%Y-%m-%d %H:%M", time.localtime(stat.st_mtime)),
            })
            seen.add(fname.replace(".pcd", ""))
        # Bag 目录
        elif os.path.isdir(fpath) and fname.startswith("map_"):
            db3_files = [f for f in os.listdir(fpath) if f.endswith('.db3')]
            if db3_files:
                total_size = sum(os.path.getsize(os.path.join(fpath, f)) for f in db3_files)
                stat = os.stat(fpath)
                if fname not in seen:
                    maps.append({
                        "name": fname,
                        "size": total_size,
                        "size_mb": round(total_size / (1024*1024), 2),
                        "ts": int(stat.st_mtime),
                        "date": time.strftime("%Y-%m-%d %H:%M", time.localtime(stat.st_mtime)),
                    })
    return maps


def save_map(custom_name=""):
    """后台保存地图，立即返回"""
    import threading
    def _do_save():
        env = os.environ.copy()
        env["MAP_NAME"] = custom_name if custom_name else ""
        try:
            subprocess.run(
                ["/usr/bin/python3.10", os.path.join(os.path.dirname(__file__), "save_map.py")],
                env=env, capture_output=True, text=True, timeout=20
            )
        except:
            pass
    threading.Thread(target=_do_save, daemon=True).start()
    ts = custom_name if custom_name else time.strftime("%Y%m%d_%H%M%S")
    return True, ts


def load_map(name):
    """设置 FAST-LIO 加载地图进行定位"""
    pcd_path = os.path.join(MAP_DIR, f"{name}.pcd")
    if not os.path.exists(pcd_path):
        return False, "PCD 文件未找到，请重新保存该地图"

    yaml_path = os.path.expanduser("~/fastlio/m1_airy96_rot.yaml")
    lines = []
    with open(yaml_path) as f:
        for line in f:
            if line.strip().startswith("map_file_path:"):
                lines.append(f'        map_file_path: "{pcd_path}"\n')
            elif line.strip().startswith("localization_mode:"):
                lines.append("        localization_mode: true\n")
            else:
                lines.append(line)
    with open(yaml_path, "w") as f:
        f.writelines(lines)
    subprocess.run(["pkill", "-9", "-f", "fastlio_mapping"], capture_output=True)
    return True, ""


def delete_map(name):
    # 尝试删 PCD
    fpath_pcd = os.path.join(MAP_DIR, f"{name}.pcd")
    if os.path.exists(fpath_pcd):
        os.remove(fpath_pcd)
        return True
    # 尝试删 bag 目录
    fpath_bag = os.path.join(MAP_DIR, name)
    if os.path.isdir(fpath_bag):
        import shutil
        shutil.rmtree(fpath_bag)
        return True
    return False


def haversine(lng1, lat1, lng2, lat2):
    import math
    R = 6371000
    dlat = math.radians(lat2 - lat1)
    dlng = math.radians(lng2 - lng1)
    a = math.sin(dlat/2)**2 + math.cos(math.radians(lat1))*math.cos(math.radians(lat2))*math.sin(dlng/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))


# ── HTTP Handler ──
class Handler(BaseHTTPRequestHandler):
    def _json(self, data, status=200):
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()
        self.wfile.write(json.dumps(data, ensure_ascii=False).encode())

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path
        qs = parse_qs(parsed.query)

        if path == "/status":
            self._json(get_all_status())
        elif path == "/path":
            self._json(load_path())
        elif path == "/trajectories":
            self._json(list_trajectories())
        elif path == "/trajectory/load":
            name = qs.get("name",[""])[0]
            if not name: self._json({"ok":False,"error":"missing name"},400); return
            ok = load_trajectory(name)
            self._json({"ok":ok,"name":name} if ok else {"ok":False,"error":"not found"})
        elif path == "/maps":
            self._json(list_maps())
        else:
            # 静态文件服务
            self._serve_static(path)

    def do_POST(self):
        parsed = urlparse(self.path)
        path = parsed.path
        qs = parse_qs(parsed.query)

        # ── 路径/轨迹 ──
        if path == "/path/clear":
            clear_path(); self._json({"ok":True})
        elif path == "/trajectory/save":
            length = int(self.headers.get("Content-Length",0))
            body = json.loads(self.rfile.read(length)) if length>0 else {}
            name = body.get("name","")
            ok, result = save_trajectory(name)
            self._json({"ok":True,"name":result} if ok else {"ok":False,"error":result}, 400 if not ok else 200)
        elif path == "/trajectory/delete":
            length = int(self.headers.get("Content-Length",0))
            body = json.loads(self.rfile.read(length)) if length>0 else {}
            name = body.get("name","")
            if not name: self._json({"ok":False,"error":"missing name"},400); return
            self._json({"ok": delete_trajectory(name)})

        # ── 服务控制 ──
        elif path == "/service/rb_ctrl/start":
            # 清残留
            subprocess.run(["pkill", "-9", "-f", "rosbridge_websocket"], capture_output=True)
            subprocess.run(["pkill", "-9", "-f", "m1_full_controller"], capture_output=True)
            time.sleep(1)
            # 启动 rosbridge
            subprocess.Popen(
                ["bash", "-c",
                 "export PATH=/usr/bin:$PATH && source /opt/runtime/env.bash && source ~/m1_ws/install/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9091"],
                preexec_fn=os.setpgrp)
            time.sleep(2)
            # 启动 controller
            subprocess.Popen(
                ["bash", "-c",
                 "source /opt/runtime/env.bash && source ~/m1_ws/install/setup.bash && export LD_LIBRARY_PATH=~/genisom_robot_sdk/lib/aarch64:$LD_LIBRARY_PATH && /home/robot/m1_ws/install/m1_sdk_controller/lib/m1_sdk_controller/m1_full_controller --ros-args --params-file ~/m1_ws/src/m1_sdk_controller/config/m1_sdk_controller.yaml"],
                preexec_fn=os.setpgrp)
            self._json(get_all_status())
        elif path == "/service/rb_ctrl/stop":
            subprocess.run(["pkill", "-9", "-f", "rosbridge_websocket"], capture_output=True)
            subprocess.run(["pkill", "-9", "-f", "m1_full_controller"], capture_output=True)
            self._json(get_all_status())
        elif path == "/service/voxel/start":
            start_voxel(); self._json(get_all_status())
        elif path == "/service/voxel/stop":
            stop_voxel(); self._json(get_all_status())
        elif path == "/service/fastlio/start":
            start_fastlio(); self._json(get_all_status())
        elif path == "/service/fastlio/stop":
            stop_fastlio(); self._json(get_all_status())
        elif path == "/service/nvblox/start":
            start_nvblox(); self._json(get_all_status())
        elif path == "/service/nvblox/stop":
            stop_nvblox(); self._json({"ok":True})

        # ── 地图管理 ──
        elif path == "/map/load":
            parsed = urlparse(self.path)
            name = parse_qs(parsed.query).get("name", [""])[0]
            ok, msg = load_map(name)
            self._json({"ok":ok,"error":"" if ok else msg})
        elif path == "/map/save":
            length = int(self.headers.get("Content-Length",0))
            body = json.loads(self.rfile.read(length)) if length>0 else {}
            custom_name = body.get("name","")
            ok, result = save_map(custom_name)
            self._json({"ok":ok,"name":result} if ok else {"ok":False,"error":result})
        elif path == "/map/delete":
            length = int(self.headers.get("Content-Length",0))
            body = json.loads(self.rfile.read(length)) if length>0 else {}
            name = body.get("name","")
            if not name: self._json({"ok":False,"error":"missing name"},400); return
            self._json({"ok": delete_map(name)})
        else:
            self._json({"error":"unknown path"}, 404)

    def _serve_static(self, path):
        """服务静态文件"""
        # 安全检查：防止目录穿越
        safe_path = os.path.normpath(path).lstrip("/")
        if safe_path == "" or safe_path == "/":
            safe_path = "index.html"
        file_path = os.path.join(WEBUI_DIR, safe_path)
        if not os.path.abspath(file_path).startswith(os.path.abspath(WEBUI_DIR)):
            self._json({"error": "forbidden"}, 403)
            return
        if not os.path.isfile(file_path):
            if path == "/favicon.ico":
                self.send_response(204); self.end_headers(); return
            self._json({"error": "not found"}, 404)
            return
        # MIME 类型
        ext = os.path.splitext(file_path)[1].lower()
        mime_map = {
            ".html": "text/html; charset=utf-8",
            ".css": "text/css",
            ".js": "application/javascript",
            ".json": "application/json",
            ".png": "image/png",
            ".jpg": "image/jpeg",
            ".svg": "image/svg+xml",
            ".ico": "image/x-icon",
            ".wasm": "application/wasm",
            ".map": "application/json",
        }
        content_type = mime_map.get(ext, "application/octet-stream")
        try:
            with open(file_path, "rb") as f:
                data = f.read()
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(data)
        except:
            self._json({"error": "read error"}, 500)

    def log_message(self, *args):
        pass


if __name__ == "__main__":
    server = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    print(f"[control_server] M1 WebUI API :{PORT}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        server.server_close()
