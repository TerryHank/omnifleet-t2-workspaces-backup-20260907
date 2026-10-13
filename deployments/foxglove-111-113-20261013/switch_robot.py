"""Select a robot connection and its matching native Foxglove layout. No ROS writes."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from urllib.request import Request, build_opener, ProxyHandler

robot = sys.argv[1]
profiles = {
    "111": ("robot-111-adapted.json", "111 Ackermann｜手动控制与传感器"),
    "113": ("local-113-control.json", "113 履带车｜导航与手动控制"),
}
filename, name = profiles[robot]
root = Path(__file__).resolve().parent
config_path = Path(os.environ["APPDATA"]) / "foxglove-studio-cn/demo-control.json"
http = build_opener(ProxyHandler({}))

def api(method, route, body=None):
    config = json.loads(config_path.read_text(encoding="utf-8-sig"))
    if config["url"] != "http://127.0.0.1:7334":
        raise RuntimeError("Expected the local Foxglove API")
    request = Request(config["url"] + route,
                      data=json.dumps(body).encode() if body is not None else None,
                      headers={"Authorization": "Bearer " + config["token"],
                               "Content-Type": "application/json"}, method=method)
    with http.open(request, timeout=10) as response:
        return json.load(response)

exe = Path(os.environ["LOCALAPPDATA"]) / "Programs/foxglove-studio-cn-desktop/Foxglove-Studio-CN.exe"
subprocess.Popen([str(exe), "--demo-control"], creationflags=subprocess.CREATE_NO_WINDOW)
for _ in range(40):
    try:
        api("GET", "/status")
        break
    except (OSError, ValueError):
        time.sleep(0.5)
else:
    raise RuntimeError("Foxglove API did not become ready")

# Remove control panels before changing the data source.
api("POST", "/layout", {"name": "正在连接 " + robot, "data": {
    "version": 1, "configById": {"SourceInfo!connecting": {}},
    "layout": "SourceInfo!connecting", "userNodes": {}, "globalVariables": {}}})
url = "ws://192.168.3." + robot + ":8765"
api("POST", "/connect", {"url": url})
for _ in range(40):
    status = api("GET", "/status")
    if status.get("presence") == "PRESENT" and status.get("source", "").strip().rstrip("/") == url:
        break
    time.sleep(0.5)
else:
    raise RuntimeError("Robot connection is unavailable; control panels remain absent")
data = json.loads((root / filename).read_text(encoding="utf-8"))
api("POST", "/layout", {"name": name, "data": data})
actual = api("GET", "/layout")["data"]
control = next(v for k, v in actual["configById"].items() if k.startswith("Teleop!"))
assert control["topic"] == ("/cmd_vel_teleop" if robot == "111" else "/robot_113/cmd_vel")
print("Connected:", url, "Control topic:", control["topic"])
