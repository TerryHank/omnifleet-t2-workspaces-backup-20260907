import json, math, time, websocket
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrow

WS = "ws://127.0.0.1:9090"

def quat_to_yaw(q):
    x, y, z, w = q["x"], q["y"], q["z"], q["w"]
    return math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

def quat_mul(a, b):
    ax, ay, az, aw = a["x"], a["y"], a["z"], a["w"]
    bx, by, bz, bw = b["x"], b["y"], b["z"], b["w"]
    return {
        "x": aw*bx + ax*bw + ay*bz - az*by,
        "y": aw*by - ax*bz + ay*bw + az*bx,
        "z": aw*bz + ax*by - ay*bx + az*bw,
        "w": aw*bw - ax*bx - ay*by - az*bz,
    }

def rotate(q, v):
    # rotate vector v by quaternion q
    x, y, z, w = q["x"], q["y"], q["z"], q["w"]
    vx, vy, vz = v
    tx = 2*(y*vz - z*vy); ty = 2*(z*vx - x*vz); tz = 2*(x*vy - y*vx)
    return (vx + w*tx + (y*tz - z*ty),
            vy + w*ty + (z*tx - x*tz),
            vz + w*tz + (x*ty - y*tx))

ws = websocket.create_connection(WS, timeout=20)
def sub(topic, mtype, **kw):
    op = {"op": "subscribe", "topic": topic, "type": mtype, "throttle_rate": 100}
    op.update(kw)
    ws.send(json.dumps(op))

sub("/map", "nav_msgs/msg/OccupancyGrid")
sub("/odom", "nav_msgs/msg/Odometry")
sub("/tf", "tf2_msgs/msg/TFMessage")
sub("/tf_static", "tf2_msgs/msg/TFMessage", durability="transient_local")

map_msg = odom_msg = None
tf_map_base = None   # transform with parent=map, child=base_link (latest by stamp)
tf_map_base_stamp = -1.0
t_end = time.time() + 6
while time.time() < t_end:
    try:
        m = json.loads(ws.recv())
    except Exception:
        break
    if m.get("op") != "publish":
        continue
    topic = m.get("topic")
    if topic == "/map" and map_msg is None:
        map_msg = m["msg"]
    elif topic == "/odom":
        odom_msg = m["msg"]  # keep latest
    elif topic in ("/tf", "/tf_static"):
        for t in m["msg"].get("transforms", []):
            if t["header"]["frame_id"].lstrip("/") == "map" and t["child_frame_id"].lstrip("/") == "base_link":
                st = t["header"]["stamp"]
                stamp = st["sec"] + st["nanosec"] * 1e-9
                if stamp > tf_map_base_stamp:
                    tf_map_base_stamp = stamp
                    tf_map_base = t["transform"]
for tp in ("/map", "/odom", "/tf", "/tf_static"):
    ws.send(json.dumps({"op": "unsubscribe", "topic": tp}))
ws.close()

assert map_msg is not None, "no /map received"
assert odom_msg is not None, "no /odom received"

info = map_msg["info"]
w, h, res = info["width"], info["height"], info["resolution"]
ox, oy = info["origin"]["position"]["x"], info["origin"]["position"]["y"]
data = np.array(map_msg["data"], dtype=np.int16).reshape(h, w)

# robot pose in map frame: prefer TF map->base_link (published by MOLA)
if tf_map_base is not None:
    t = tf_map_base["translation"]; q = tf_map_base["rotation"]
    rx, ry = t["x"], t["y"]
    ryaw = quat_to_yaw(q)
    tf_note = "pose from TF map->base_link"
else:
    p = odom_msg["pose"]["pose"]["position"]
    rx, ry = p["x"], p["y"]
    ryaw = quat_to_yaw(odom_msg["pose"]["pose"]["orientation"])
    tf_note = "no map TF found; raw /odom pose shown"

# render: flip vertically so +y is up, row 0 at bottom
grid = np.flipud(data).astype(float)
masked = np.ma.masked_where(grid < 0, grid)

fig, ax = plt.subplots(figsize=(11, 6), dpi=130)
ax.set_facecolor("#d9d9d9")  # unknown = light gray
cmap = plt.get_cmap("gray_r").copy()  # 0 -> white, 100 -> black
im = ax.imshow(masked, cmap=cmap, vmin=0, vmax=100,
               extent=[ox, ox + w*res, oy, oy + h*res], origin="lower")

# robot arrow
alen = 0.9
ax.add_patch(plt.Circle((rx, ry), 0.18, color="#e53935", zorder=5))
ax.annotate("", xy=(rx + alen*math.cos(ryaw), ry + alen*math.sin(ryaw)),
            xytext=(rx, ry),
            arrowprops=dict(arrowstyle="-|>", color="#e53935", lw=2.2), zorder=6)

ax.set_title(f"/map  ({w}x{h} cells @ {res:.2f} m, frame 'map', stamp {map_msg['header']['stamp']['sec']})\n"
             f"robot @ ({rx:.2f}, {ry:.2f}) m, yaw {math.degrees(ryaw):.1f}°  [{tf_note}]",
             fontsize=10)
ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
ax.grid(True, color="#888888", lw=0.3, alpha=0.4)
cb = fig.colorbar(im, ax=ax, fraction=0.025, pad=0.015)
cb.set_label("occupancy (0=free, 100=occupied, gray=unknown)", fontsize=8)
fig.tight_layout()
out = "/home/iecme/workspace/omnifleet_t2_ws/map_snapshot.png"
fig.savefig(out, bbox_inches="tight")
print("saved:", out)
print(f"robot in map: ({rx:.3f}, {ry:.3f}), yaw={math.degrees(ryaw):.2f} deg, {tf_note}")
