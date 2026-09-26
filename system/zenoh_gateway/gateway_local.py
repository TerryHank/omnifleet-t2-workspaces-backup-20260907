"""Local-only ROS observer/proxies. This process has no WAN Zenoh session."""
import json,os,threading,time,struct
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from gateway_common import *

robot=os.environ['OMNIFLEET_ROBOT_ID'];peer='robot_104' if robot=='robot_113' else 'robot_113'
rclpy.init();node=rclpy.create_node('fleet_local_gateway_'+robot)
group=ReentrantCallbackGroup();lock=threading.RLock();registry_lock=threading.RLock();discovery_lock=threading.Lock();goal_lock=threading.Lock();goal_cache={};cache={};subs={};pubs={};clients={};proxies={};catalog={};inputs=input_types(robot);remote_inputs={};tf_cache={}

def source_name(name):
    return (name.startswith('/'+robot+'/') or (robot=='robot_113' and name.startswith('/fleet/'))) and '/bond' not in name

def discover():
    if not discovery_lock.acquire(blocking=False):return
    try:discover_locked()
    finally:discovery_lock.release()

def discover_locked():
    topics={};services={}
    for name,types in node.get_topic_names_and_types():
        if not source_name(name) or name in inputs or len(types)!=1:continue
        publishers=node.get_publishers_info_by_topic(name)
        if not publishers:continue
        kind=types[0];latched=any(p.qos_profile.durability==DurabilityPolicy.TRANSIENT_LOCAL for p in publishers)
        topics[name]={'type':kind,'latched':latched}
        if name not in subs:
            try:
                cls=get_message(kind)
                def receive(message,key=name):
                    with lock:
                        if key.endswith(('/tf','/tf_static')):
                            frames=tf_cache.setdefault(key,{})
                            for transform in message.transforms:
                                if transform.child_frame_id.startswith(robot+'/') or transform.child_frame_id=='fleet_map':
                                    frames[transform.child_frame_id]=transform
                            if key.endswith('/tf'):
                                now=node.get_clock().now().nanoseconds/1e9
                                frames={k:t for k,t in frames.items() if now-(t.header.stamp.sec+t.header.stamp.nanosec/1e9)<2}
                                tf_cache[key]=frames
                            message.transforms=list(frames.values())
                        body=message if isinstance(message,bytes) else serialize_message(message)
                        cache[key]=(cache.get(key,(0,b''))[0]+1,body)
                subs[name]=node.create_subscription(cls,name,receive,qos(latched),callback_group=group,
                                                    raw=not name.endswith(('/tf','/tf_static')))
            except (ImportError,AttributeError):continue
    for name,types in node.get_service_names_and_types():
        if source_name(name) and len(types)==1:services[name]=types[0]
    with lock:
        catalog.clear();catalog.update(robot=robot,topics=topics,services=services,inputs=inputs)

def install_peer(data):
    with registry_lock:install_peer_locked(data)

def install_peer_locked(data):
    for name,info in data.get('topics',{}).items():
        if not (name.startswith('/'+peer+'/') or (robot=='robot_104' and name.startswith('/fleet/'))):continue
        if name not in pubs:
            try:pubs[name]=node.create_publisher(get_message(info['type']),name,pub_qos(info['latched']))
            except (ImportError,AttributeError):continue
    for name,kind in data.get('inputs',{}).items():
        if not name.startswith('/'+peer+'/') or name in remote_inputs:continue
        def send(msg,path=name,typ=kind):
            try:http(7602,'/command',serialize_message(msg),{'X-Topic':path,'X-Type':typ},timeout=1)
            except Exception:pass
        remote_inputs[name]=node.create_subscription(get_message(kind),name,send,10,callback_group=group)
    for name,kind in data.get('services',{}).items():
        if not (name.startswith('/'+peer+'/') or (robot=='robot_104' and name.startswith('/fleet/'))):continue
        if name in proxies:continue
        try:cls=get_service(kind)
        except (ImportError,AttributeError):continue
        def forward(request,response,path=name,typ=kind,cls=cls):
            try:
                raw=http(7602,'/rpc',serialize_message(request),{'X-Service':path,'X-Type':typ},timeout=180 if '/get_result' in path else 20)
                return deserialize_message(raw,cls.Response)
            except Exception as error:
                node.get_logger().warning('RPC '+path+': '+str(error))
                return error_response(response,error)
        proxies[name]=node.create_service(cls,name,forward,callback_group=group)

class Handler(BaseHTTPRequestHandler):
    def log_message(self,*args):pass
    def reply(self,body,status=200,headers=None):
        self.send_response(status)
        for key,value in (headers or {}).items():self.send_header(key,str(value))
        self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
    def do_GET(self):
        if self.path=='/health':return self.reply(json.dumps({'cache':len(cache),'publishers':len(pubs),'proxies':len(proxies),'inputs':len(remote_inputs)}).encode())
        if self.path!='/catalog':return self.reply(b'not found',404)
        with lock:data=json.dumps(catalog).encode()
        self.reply(data)
    def do_POST(self):
        try:
            size=int(self.headers.get('Content-Length','0'))
            if size>32_000_000:raise ValueError('payload too large')
            body=self.rfile.read(size);name=self.headers.get('X-Topic','');typ=self.headers.get('X-Type','')
            if self.path=='/sample':
                with lock:seq,value=cache.get(name,(0,b''))
                return self.reply(value,headers={'X-Sequence':seq})
            if self.path=='/samples':
                query=json.loads(body);parts=[]
                with lock:values=[(key,*cache[key]) for key in query['due'] if key in cache and cache[key][0]!=query['last'].get(key)]
                for key,seq,value in values:
                    key=key.encode();parts.extend([struct.pack('<IQI',len(key),seq,len(value)),key,value])
                return self.reply(b''.join(parts))
            if self.path=='/peer_catalog':install_peer(json.loads(body));return self.reply(b'ok')
            if self.path=='/rpc':
                name=self.headers['X-Service']
                with lock:expected=catalog.get('services',{}).get(name)
                if expected!=typ:raise ValueError('service not owned locally')
                if name.endswith('/_action/send_goal'):
                    goal_request=deserialize_message(body,get_service(typ).Request)
                    key=(name,bytes(goal_request.goal_id.uuid).hex())
                    # A forwarded goal UUID is idempotent, including simultaneous
                    # duplicate deliveries while discovery is being refreshed.
                    with goal_lock:
                        now=time.monotonic()
                        for old in list(goal_cache):
                            if now-goal_cache[old][0]>60:goal_cache.pop(old)
                        if key not in goal_cache:
                            if len(goal_cache)>=256:goal_cache.pop(next(iter(goal_cache)))
                            goal_cache[key]=(now,call_ros(node,clients,name,typ,body))
                        return self.reply(goal_cache[key][1])
                return self.reply(call_ros(node,clients,name,typ,body,timeout=170 if '/get_result' in name else 15))
            if self.path in ('/incoming','/command'):
                if self.path=='/command':
                    if inputs.get(name)!=typ:raise ValueError('input not allowed')
                elif not (name.startswith('/'+peer+'/') or (robot=='robot_104' and name.startswith('/fleet/'))):raise ValueError('foreign output required')
                latched=self.headers.get('X-Latched')=='1';cls=get_message(typ)
                with registry_lock:
                    if name not in pubs:pubs[name]=node.create_publisher(cls,name,pub_qos(latched))
                pubs[name].publish(body if self.path=='/incoming' else deserialize_message(body,cls));return self.reply(b'ok')
            self.reply(b'not found',404)
        except Exception as error:
            try:self.reply(str(error).encode(),503)
            except (BrokenPipeError,ConnectionResetError):pass

node.create_timer(2,discover,callback_group=group)
server=ThreadingHTTPServer(('127.0.0.1',7601),Handler);server.daemon_threads=True
threading.Thread(target=server.serve_forever,daemon=True).start()
executor=MultiThreadedExecutor(num_threads=12);executor.add_node(node)
try:executor.spin()
except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
finally:server.shutdown();executor.shutdown();node.destroy_node();rclpy.try_shutdown()
