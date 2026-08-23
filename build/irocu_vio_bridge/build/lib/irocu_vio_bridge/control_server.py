import json
import os
import subprocess
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


CONFIG_PATH = Path.home() / "irocu_ws/control_commands.json"
DEFAULT_CONFIG = {
    "start_mission": "",
    "stop_land": "",
    "start_image_sender": ""
}

LAST_RESULT = {
    "time": None,
    "action": None,
    "ok": None,
    "stdout": "",
    "stderr": ""
}


def load_config():
    if not CONFIG_PATH.exists():
        save_config(DEFAULT_CONFIG)
        return dict(DEFAULT_CONFIG)

    try:
        data = json.loads(CONFIG_PATH.read_text())
    except Exception:
        data = {}

    config = dict(DEFAULT_CONFIG)
    config.update(data)
    return config


def save_config(config):
    CONFIG_PATH.parent.mkdir(parents=True, exist_ok=True)
    clean = {
        "start_mission": str(config.get("start_mission", "")).replace("\\n", "\n").strip(),
        "stop_land": str(config.get("stop_land", "")).replace("\\n", "\n").strip(),
        "start_image_sender": str(config.get("start_image_sender", "")).replace("\\n", "\n").strip()
    }
    CONFIG_PATH.write_text(json.dumps(clean, indent=2))
    return clean


def run_command(command_key, timeout=3):
    global LAST_RESULT

    config = load_config()
    command = str(config.get(command_key, "")).replace("\\n", "\n").strip()

    if not command:
        LAST_RESULT = {
            "time": time.time(),
            "action": command_key,
            "ok": False,
            "stdout": "",
            "stderr": f"Command '{command_key}' is empty"
        }
        return LAST_RESULT

    try:
        process = subprocess.Popen(
            ["bash", "-lc", command],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            start_new_session=True
        )

        if command_key in ["start_mission", "start_image_sender"]:
            LAST_RESULT = {
                "time": time.time(),
                "action": command_key,
                "ok": True,
                "pid": process.pid,
                "stdout": f"{command_key} launched pid={process.pid}",
                "stderr": ""
            }
            return LAST_RESULT

        try:
            stdout, stderr = process.communicate(timeout=timeout)
            ok = process.returncode == 0
        except subprocess.TimeoutExpired:
            ok = True
            stdout = f"{command_key} sent pid={process.pid}"
            stderr = ""

        LAST_RESULT = {
            "time": time.time(),
            "action": command_key,
            "ok": ok,
            "pid": process.pid,
            "stdout": stdout[-5000:],
            "stderr": stderr[-5000:]
        }
        return LAST_RESULT

    except Exception as error:
        LAST_RESULT = {
            "time": time.time(),
            "action": command_key,
            "ok": False,
            "stdout": "",
            "stderr": str(error)
        }
        return LAST_RESULT


def status():
    return {
        "server": "isro_pi_control_server",
        "time": time.time(),
        "config_path": str(CONFIG_PATH),
        "config": load_config(),
        "last_result": LAST_RESULT
    }


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        return

    def send_json(self, data, code=200):
        body = json.dumps(data).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        self.send_json({"ok": True})

    def do_GET(self):
        if self.path == "/health":
            self.send_json({"ok": True, "server": "isro_pi_control_server"})
            return

        if self.path == "/api/status":
            self.send_json(status())
            return

        if self.path == "/api/config":
            self.send_json(load_config())
            return

        self.send_json({"ok": False, "error": "not_found"}, 404)

    def do_POST(self):
        if self.path == "/api/config":
            length = int(self.headers.get("Content-Length", "0"))
            body = self.rfile.read(length)

            try:
                data = json.loads(body.decode("utf-8"))
                saved = save_config(data)
                self.send_json({"ok": True, "config": saved})
            except Exception as error:
                self.send_json({"ok": False, "error": str(error)}, 400)

            return

        if self.path == "/api/run/start_mission":
            self.send_json(run_command("start_mission"))
            return

        if self.path == "/api/run/stop_land":
            self.send_json(run_command("stop_land"))
            return

        if self.path == "/api/run/start_image_sender":
            self.send_json(run_command("start_image_sender"))
            return

        self.send_json({"ok": False, "error": "not_found"}, 404)


class Server(ThreadingHTTPServer):
    allow_reuse_address = True


def main():
    host = os.environ.get("ISRO_CONTROL_HOST", "0.0.0.0")
    port = int(os.environ.get("ISRO_CONTROL_PORT", "9100"))
    server = Server((host, port), Handler)
    print(f"ISRO Pi control server running at http://{host}:{port}")
    print(f"Config: {CONFIG_PATH}")
    server.serve_forever()


if __name__ == "__main__":
    main()
