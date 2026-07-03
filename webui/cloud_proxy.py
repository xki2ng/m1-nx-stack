#!/usr/bin/env python3
"""云端 WebUI 代理 — socket 直连 tunnel"""
import socket, os, sys, json
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn

class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True

PORT = 8443
WEBUI_DIR = os.path.dirname(os.path.abspath(__file__))
API_PATHS = ['/status', '/path', '/trajector', '/maps', '/service/', '/map/']

def proxy_request(path, method='GET', body=None):
    """通过 socket 直连 tunnel 转发请求"""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(8)
    try:
        s.connect(('127.0.0.1', 8000))
        req = f"{method} {path} HTTP/1.0\r\nHost: 127.0.0.1\r\nConnection: close\r\n"
        if body:
            req += f"Content-Type: application/json\r\nContent-Length: {len(body)}\r\n"
        req += "\r\n"
        s.sendall(req.encode())
        if body:
            s.sendall(body if isinstance(body, bytes) else body.encode())
        
        # 读取响应
        data = b''
        while True:
            chunk = s.recv(4096)
            if not chunk:
                break
            data += chunk
        s.close()
        
        # 分离 header 和 body
        parts = data.split(b'\r\n\r\n', 1)
        if len(parts) < 2:
            return 502, json.dumps({"error": "no response body"})
        body = parts[1]
        
        # 检查 HTTP 状态码
        header = parts[0].decode()
        status = 200
        if header.startswith('HTTP/1.'):
            try:
                status = int(header.split()[1])
            except:
                pass
        return status, body.decode()
    except socket.timeout:
        return 502, json.dumps({"error": "tunnel timeout"})
    except Exception as e:
        return 502, json.dumps({"error": str(e)})
    finally:
        try: s.close()
        except: pass

class ProxyHandler(BaseHTTPRequestHandler):
    def _cors(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")

    def do_OPTIONS(self):
        self.send_response(204); self._cors(); self.end_headers()

    def do_GET(self): self._handle()
    def do_POST(self): self._handle()

    def _handle(self):
        path = self.path
        query_idx = path.find('?')
        clean_path = path[:query_idx] if query_idx > 0 else path
        print(f"[{self.command}] {clean_path} {self.client_address[0]}", file=sys.stderr)

        if any(clean_path.startswith(p) for p in API_PATHS):
            body = None
            if self.command == 'POST':
                length = int(self.headers.get('Content-Length', 0))
                body = self.rfile.read(length) if length > 0 else None
            status, resp = proxy_request(path, self.command, body)
            self.send_response(status)
            self._cors()
            self.send_header('Content-Type', 'application/json')
            self.send_header('Connection', 'close')
            self.end_headers()
            self.wfile.write(resp.encode())
            return

        # 静态文件
        safe = clean_path.lstrip("/") or "index.html"
        fp = os.path.join(WEBUI_DIR, safe)
        if not os.path.abspath(fp).startswith(os.path.abspath(WEBUI_DIR)):
            self.send_response(403); self.end_headers(); return
        if not os.path.isfile(fp):
            if clean_path == "/favicon.ico":
                self.send_response(204); self.end_headers(); return
            self.send_response(404); self.end_headers(); return
        ext = os.path.splitext(fp)[1].lower()
        mime = {".html":"text/html",".css":"text/css",".js":"application/javascript",
                ".json":"application/json",".png":"image/png",".svg":"image/svg+xml"}
        with open(fp, "rb") as f: data = f.read()
        self.send_response(200)
        self._cors()
        self.send_header("Content-Type", f"{mime.get(ext,'')}; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Connection", "close")
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, *a): pass

if __name__ == "__main__":
    import atexit, time as _time
    PID_FILE = os.path.expanduser("~/webui/.cloud_proxy.pid")
    if os.path.exists(PID_FILE):
        try:
            with open(PID_FILE) as f:
                os.kill(int(f.read().strip()), 15)
            _time.sleep(0.5)
        except: pass
    with open(PID_FILE, "w") as f:
        f.write(str(os.getpid()))
    atexit.register(lambda: os.remove(PID_FILE) if os.path.exists(PID_FILE) else None)
    ThreadingHTTPServer(("0.0.0.0", PORT), ProxyHandler).serve_forever()
