import json, time, websocket, collections
ws = websocket.create_connection("ws://127.0.0.1:9090", timeout=15)
for tp, dur in [("/tf", None), ("/tf_static", "transient_local")]:
    op = {"op": "subscribe", "topic": tp, "type": "tf2_msgs/msg/TFMessage", "throttle_rate": 200}
    if dur: op["durability"] = dur
    ws.send(json.dumps(op))
pairs = collections.Counter()
latest = {}
t_end = time.time() + 5
while time.time() < t_end:
    try: m = json.loads(ws.recv())
    except Exception: break
    if m.get("op") != "publish": continue
    for t in m["msg"].get("transforms", []):
        p, c = t["header"]["frame_id"], t["child_frame_id"]
        pairs[(p, c)] += 1
        latest[(p, c)] = t["transform"]
for tp in ("/tf", "/tf_static"):
    ws.send(json.dumps({"op": "unsubscribe", "topic": tp}))
ws.close()
for (p, c), n in pairs.most_common():
    tr = latest[(p, c)]
    t = tr["translation"]; q = tr["rotation"]
    print(f"{p} -> {c}  (x{n})  t=({t['x']:.3f},{t['y']:.3f},{t['z']:.3f}) q=({q['x']:.3f},{q['y']:.3f},{q['z']:.3f},{q['w']:.3f})")
