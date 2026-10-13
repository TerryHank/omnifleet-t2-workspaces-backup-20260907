"""WAN ROS gateway in a separate process; it cannot block local navigation IO."""
import json,os,threading,time,http.client,socket,base64,struct
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
os.environ['ZENOH_SESSION_CONFIG_URI']='/etc/omnifleet_t2/zenoh-fleet-session.json5'
os.environ['ZENOH_ROUTER_CHECK_ATTEMPTS']='-1'
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from gateway_common import *

robot=os.environ['OMNIFLEET_ROBOT_ID'];peer='robot_104' if robot=='robot_113' else 'robot_113'
rclpy.init();node=rclpy.create_node('fleet_network_gateway_'+robot);group=ReentrantCallbackGroup()
pubs={};services={};subs={};inputs={};clients={};sequences={};last_sent={};local={};peer_seen=0.;closing=False;sample_lock=threading.Lock();discovery_lock=threading.Lock();peer_lock=threading.Lock()
catalog_pub=node.create_publisher(String,'/fleet_gateway/'+robot+'/catalog',qos(True))
heartbeat=node.create_publisher(String,'/fleet_gateway/'+robot+'/heartbeat',10)
command_pub=node.create_publisher(String,'/fleet_gateway/'+peer+'/input',10)
def receive_input(msg):
    try:
        data=json.loads(msg.data)
        if not 0<=time.time()-data['sent_at']<=.5:return
        if local.get('inputs',{}).get(data['topic'])!=data['type']:return
        http(7601,'/command',base64.b64decode(data['body']),{'X-Topic':data['topic'],'X-Type':data['type']},timeout=1)
    except Exception:pass
node.create_subscription(String,'/fleet_gateway/'+robot+'/input',receive_input,10,callback_group=group)
def peer_beat(_):
    global peer_seen
    peer_seen=time.monotonic()
node.create_subscription(String,'/fleet_gateway/'+peer+'/heartbeat',peer_beat,10,callback_group=group)

def peer_catalog(msg):
    with peer_lock:peer_catalog_locked(msg)

def peer_catalog_locked(msg):
    try:
        data=json.loads(msg.data);http(7601,'/peer_catalog',msg.data.encode())
        for name,info in data.get('topics',{}).items():
            if name in subs:continue
            kind=info['type'];latched=info['latched'];cls=get_message(kind)
            def receive(message,key=name,typ=kind,latch=latched):
                try:http(7601,'/incoming',message,{'X-Topic':key,'X-Type':typ,'X-Latched':str(int(latch))},timeout=1)
                except Exception:pass
            subs[name]=node.create_subscription(cls,name,receive,qos(latched),callback_group=group,raw=True)
    except Exception as error:node.get_logger().warning('peer catalog: '+str(error))
node.create_subscription(String,'/fleet_gateway/'+peer+'/catalog',peer_catalog,qos(True),callback_group=group)

def discover():
    if not discovery_lock.acquire(blocking=False):return
    try:discover_locked()
    finally:discovery_lock.release()

def discover_locked():
    global local
    try:local=json.loads(http(7601,'/catalog'))
    except Exception:return
    catalog_pub.publish(String(data=json.dumps(local)))
    for name,info in local.get('topics',{}).items():
        if name not in pubs:
            try:pubs[name]=node.create_publisher(get_message(info['type']),name,pub_qos(info['latched']))
            except (ImportError,AttributeError):continue
    for name,kind in local.get('services',{}).items():
        if name in services:continue
        try:cls=get_service(kind)
        except (ImportError,AttributeError):continue
        def forward(request,response,path=name,typ=kind,cls=cls):
            try:
                raw=http(7601,'/rpc',serialize_message(request),{'X-Service':path,'X-Type':typ},timeout=180 if '/get_result' in path else 20)
                return deserialize_message(raw,cls.Response)
            except Exception as error:
                node.get_logger().warning('RPC '+path+': '+str(error))
                return error_response(response,error)
        services[name]=node.create_service(cls,name,forward,callback_group=group)

def publish_samples():
    if not sample_lock.acquire(blocking=False):return
    try:
        now=time.monotonic();due=[]
        for name,info in list(local.get('topics',{}).items()):
            period=.5 if info['type'] in ('sensor_msgs/msg/PointCloud2','sensor_msgs/msg/Image') else .1
            if now-last_sent.get(name,0)>=period and name in pubs:due.append(name);last_sent[name]=now
        raw=http(7601,'/samples',json.dumps({'due':due,'last':sequences}).encode(),timeout=1);offset=0
        while offset<len(raw):
            key_size,seq,size=struct.unpack_from('<IQI',raw,offset);offset+=16
            name=raw[offset:offset+key_size].decode();offset+=key_size
            body=raw[offset:offset+size];offset+=size
            if name in pubs:pubs[name].publish(body);sequences[name]=seq
    except Exception:pass
    finally:sample_lock.release()

class Handler(BaseHTTPRequestHandler):
    def log_message(self,*args):pass
    def do_GET(self):
        body=json.dumps({'robot':robot,'peer_age':time.monotonic()-peer_seen if peer_seen else None,'publishers':len(pubs),'subscriptions':len(subs),'services':len(services),'samples':len(sequences)}).encode()
        self.send_response(200);self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
    def do_POST(self):
        try:
            if self.path=='/command':
                if not peer_seen or time.monotonic()-peer_seen>3:raise RuntimeError('peer is offline')
                topic=self.headers['X-Topic'];typ=self.headers['X-Type']
                if input_types(peer).get(topic)!=typ:raise ValueError('remote input not allowed')
                data=self.rfile.read(int(self.headers.get('Content-Length','0')))
                command_pub.publish(String(data=json.dumps({'topic':topic,'type':typ,'sent_at':time.time(),'body':base64.b64encode(data).decode()})))
                self.send_response(200);self.end_headers();return
            name=self.headers['X-Service'];typ=self.headers['X-Type']
            if not (name.startswith('/'+peer+'/') or (robot=='robot_104' and name.startswith('/fleet/'))):raise ValueError('remote namespace required')
            if not peer_seen or time.monotonic()-peer_seen>3:raise RuntimeError('peer is offline')
            size=int(self.headers.get('Content-Length','0'))
            if size>32_000_000:raise ValueError('payload too large')
            result=call_ros(node,clients,name,typ,self.rfile.read(size),timeout=170 if '/get_result' in name else 15)
            self.send_response(200);self.send_header('Content-Length',str(len(result)));self.end_headers();self.wfile.write(result)
        except Exception as error:
            try:self.send_response(503);self.end_headers();self.wfile.write(str(error).encode())
            except (BrokenPipeError,ConnectionResetError):pass

node.create_timer(1,discover,callback_group=group);node.create_timer(.05,publish_samples,callback_group=group)
node.create_timer(.5,lambda:heartbeat.publish(String(data=robot)),callback_group=group)
server=ThreadingHTTPServer(('127.0.0.1',7602),Handler);server.daemon_threads=True
threading.Thread(target=server.serve_forever,daemon=True).start()
def reconnect_guard():
    started=time.monotonic()
    while not closing:
        time.sleep(4+(0 if robot=='robot_113' else 1))
        if time.monotonic()-max(peer_seen,started)<15:continue
        try:
            with socket.create_connection(('192.168.3.'+peer.split('_')[-1],7600),timeout=.5):pass
        except OSError:continue
        # Recreate only the WAN process if transport has not recovered. Local
        # ROS sessions and navigation remain untouched by this recovery.
        os._exit(75)
threading.Thread(target=reconnect_guard,daemon=True).start()
executor=MultiThreadedExecutor(num_threads=12);executor.add_node(node)
try:executor.spin()
except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
finally:closing=True;server.shutdown();executor.shutdown();node.destroy_node();rclpy.try_shutdown()
