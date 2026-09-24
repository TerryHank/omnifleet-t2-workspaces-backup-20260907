import json, time, websocket, math
ws = websocket.create_connection("ws://127.0.0.1:9090", timeout=15)
ws.send(json.dumps({"op":"subscribe","topic":"/tf","type":"tf2_msgs/msg/TFMessage","throttle_rate":100}))
ws.send(json.dumps({"op":"subscribe","topic":"/tf_static","type":"tf2_msgs/msg/TFMessage","throttle_rate":100,"durability":"transient_local"}))
t_end=time.time()+6
seen=[]
while time.time()<t_end:
    try: m=json.loads(ws.recv())
    except Exception: break
    if m.get("op")!="publish": continue
    for t in m["msg"].get("transforms",[]):
        if t["header"]["frame_id"]=="map" and t["child_frame_id"]=="base_link":
            tr=t["transform"]; q=tr["rotation"]
            yaw=math.degrees(math.atan2(2*(q["w"]*q["z"]),1-2*q["z"]**2))
            seen.append((m["topic"], t["header"]["stamp"]["sec"], tr["translation"]["x"], tr["translation"]["y"], yaw))
ws.close()
for s in seen[-15:]:
    print(f"{s[0]:12s} stamp={s[1]} x={s[2]:.3f} y={s[3]:.3f} yaw={s[4]:.1f}°")
print("total:", len(seen))
