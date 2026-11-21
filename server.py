"""Live simulator.   python3 server.py   ->  http://localhost:8766
Page = 3D model of the R-P-P-R-R(-R) cylindrical arm + the 300 mm workspace sphere.
Optional: mirror the live pose into the FreeCAD window through the FreeCAD MCP RPC server (port 9875)."""
import json, os, sys, threading, xmlrpc.client
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import robot

FC = {"latest": None, "event": threading.Event()}
FC_SRC = open(os.path.join(HERE, "fc_scene.py")).read()
_ANA = {}


def fc_worker():
    srv = xmlrpc.client.ServerProxy("http://127.0.0.1:9875", allow_none=True)
    while True:
        FC["event"].wait(); FC["event"].clear()
        q = FC["latest"]
        if q is None: continue
        try:
            srv.execute_code(FC_SRC + "\nimport json\napply(json.loads(" + repr(json.dumps({"q": q})) + "))", 60)
        except Exception as e:
            print("FreeCAD push failed:", e)


threading.Thread(target=fc_worker, daemon=True).start()


class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass

    def _send(self, obj, code=200, ctype="application/json"):
        body = obj if isinstance(obj, bytes) else json.dumps(obj).encode()
        self.send_response(code); self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body))); self.end_headers(); self.wfile.write(body)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self._send(open(os.path.join(HERE, "index.html"), "rb").read(), ctype="text/html; charset=utf-8")
        else:
            self._send({"error": "not found"}, 404)

    def do_POST(self):
        n = int(self.headers.get("Content-Length", 0)); d = json.loads(self.rfile.read(n) or b"{}")
        try:
            if self.path == "/freecad":
                FC["latest"] = d["q"]; FC["event"].set(); r = {"ok": True}
            elif self.path == "/analyse":
                if not _ANA:
                    a = robot.analyse(1500); b = robot.analyse_positions(150, 60); _ANA.update(a); _ANA.update(b)
                r = _ANA
            else:
                r = {"error": "not found"}
            self._send(r)
        except Exception as e:
            self._send({"error": repr(e)}, 500)


if __name__ == "__main__":
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8766
    print(f"Simulator on http://localhost:{port}", flush=True)
    ThreadingHTTPServer(("127.0.0.1", port), H).serve_forever()
