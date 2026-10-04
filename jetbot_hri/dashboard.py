"""Loopback-only status and emergency STOP, for use through an SSH tunnel."""
from http.server import BaseHTTPRequestHandler, HTTPServer
import json
import threading


class Dashboard:
    def __init__(self, supervisor, port):
        self.supervisor = supervisor
        self.status = {}
        self.lock = threading.Lock()
        dashboard = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path == "/status":
                    with dashboard.lock:
                        body = json.dumps(dashboard.status).encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                elif self.path == "/":
                    body = ("<html><body><h1>JetBot HRI</h1><pre id='status'></pre>"
                            "<button onclick=\"fetch('/estop',{method:'POST'})\">EMERGENCY STOP</button>"
                            "<script>setInterval(async()=>{document.getElementById('status').textContent="
                            "JSON.stringify(await(await fetch('/status')).json(),null,2)},500)</script>"
                            "</body></html>").encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                else:
                    self.send_error(404)
                    return
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_POST(self):
                if self.path != "/estop":
                    self.send_error(404)
                    return
                dashboard.supervisor.emergency_stop()
                body = b'{"emergency_stop":true}'
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, *_):
                pass

        self.server = HTTPServer(("127.0.0.1", port), Handler)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def update(self, **status):
        with self.lock:
            self.status = status

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=1)
