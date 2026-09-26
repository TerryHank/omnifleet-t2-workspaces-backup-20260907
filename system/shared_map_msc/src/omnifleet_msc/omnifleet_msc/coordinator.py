from fleet_scope import frame
import argparse,hmac,json,threading,time,math
from pathlib import Path
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
from rclpy.time import Time
from .core import Coordinator
from .protocol import signature
from .shared_map import encode_grid,QOS
from nav_msgs.msg import OccupancyGrid
from .planning import FleetGrid
from .panel_ros import FleetPanelIO

class CoordinatorNode(Node):
    def __init__(self,config):
        super().__init__('msc_coordinator')
        self.config=config;self.lock=threading.RLock();self.core=Coordinator(config,config['journal']);self.nonces={}
        self.shared_map=None;self.map_epoch=None;self.map_received=0.;self.map_source_process=''
        self.shared_pub=self.create_publisher(OccupancyGrid,'/fleet/map',QOS)
        self.map_status_pub=self.create_publisher(String,'/msc/shared_map_status',10)
        self.create_subscription(OccupancyGrid,'/map',self.on_map,QOS)
        topics=['/fleet/status','/fleet/events','/fleet/mission_state','/fleet/formation_state','/fleet/report']
        for robot in config['robots']:
            topics += ['/'+robot+'/'+name for name in ('heartbeat','health','mission_state','formation_error')]
        self.pubs={t:self.create_publisher(String,t,10) for t in topics};self.last_event=0
        self.markers=self.create_publisher(MarkerArray,'/fleet/robots',10);self.tf=TransformBroadcaster(self)
        self.static_tf=StaticTransformBroadcaster(self)
        reference=TransformStamped();reference.header.frame_id=frame('map');reference.child_frame_id='fleet_map'
        reference.header.stamp=self.get_clock().now().to_msg();reference.transform.rotation.w=1.;self.static_tf.sendTransform(reference)
        for operation in ('pause','resume','cancel','stop','release_stop'):
            def callback(req,res,op=operation):
                try:
                    with self.lock:result=self.core.operator({'op':op})
                    res.success=True;res.message=json.dumps(result,ensure_ascii=False)
                except Exception as error:res.success=False;res.message=str(error)
                return res
            self.create_service(Trigger,'/fleet/'+operation,callback)
        self.create_timer(.2,self.tick)
        owner=self
        class Handler(BaseHTTPRequestHandler):
            def log_message(self,*args):pass
            def do_POST(self):
                self.connection.settimeout(2.)
                identity=self.headers.get('X-MSC-ID','');keys=owner.config['keys'];key=keys.get(identity)
                nonce=self.headers.get('X-MSC-Nonce','');path=self.path
                try:
                    length=int(self.headers.get('Content-Length','0'))
                    if not key or not 0<length<=2_000_000:raise ValueError('invalid identity/request size')
                    body=self.rfile.read(length)
                    stamp=float(nonce.split(':')[0])
                    if not math.isfinite(stamp) or abs(time.time()-stamp)>5:raise ValueError('request timestamp out of window')
                    if not hmac.compare_digest(self.headers.get('X-MSC-Signature',''),signature(key,'POST '+path,nonce,body)):
                        raise ValueError('invalid signature')
                    with owner.lock:
                        owner.nonces={k:v for k,v in owner.nonces.items() if time.monotonic()-v<10}
                        if (identity,nonce) in owner.nonces:raise ValueError('replayed request')
                        owner.nonces[identity,nonce]=time.monotonic();data=json.loads(body)
                        if path=='/v1/heartbeat' and identity in owner.config['robots']:
                            transit=max(0.,time.time()-stamp)
                            for field in ('pose_age','velocity_age','obstacle_age'):data['state'][field]=data['state'].get(field,99.)+transit
                            result=owner.core.heartbeat(identity,data['boot'],int(data['seq']),data['state'])
                        elif path=='/v1/status' and identity=='operator':result=owner.core.status()
                        elif path=='/v1/report' and identity=='operator':result=owner.core.report()
                        elif path=='/v1/map' and identity in owner.config['robots']:
                            state=owner.core.robot(identity);ref=owner.core.robot(owner.config['reference_robot'])
                            valid=state.get('alignment_verified') and state.get('online') and ref.get('online') and owner.map_is_current(ref)
                            result={'map':owner.shared_map if valid else None,'alignment_valid':bool(valid),
                                'alignment':owner.core.history['alignments'].get(identity)}
                        elif path=='/v1/tasks' and identity=='operator':
                            result=owner.core.task_operator(data)
                        elif path=='/v1/command' and identity=='operator':result=owner.core.operator(data)
                        else:raise ValueError('operation not allowed for this identity')
                    status=200
                except Exception as error:status=400;result={'error':str(error)}
                payload=json.dumps(result,ensure_ascii=False,allow_nan=False).encode()
                self.send_response(status);self.send_header('Content-Type','application/json');self.send_header('Content-Length',str(len(payload)))
                if key:self.send_header('X-MSC-Signature',signature(key,'RESPONSE '+path,nonce,payload))
                self.end_headers();self.wfile.write(payload)
        self.panel=FleetPanelIO(self)
        self.http=ThreadingHTTPServer((config.get('bind','0.0.0.0'),config['port']),Handler)
        self.http.daemon_threads=True;threading.Thread(target=self.http.serve_forever,daemon=True).start()

    def on_map(self,message):
        if not message.data:return
        with self.lock:
            self.shared_map=encode_grid(message);self.map_received=time.monotonic()
            p=message.info.origin.position;q=message.info.origin.orientation
            self.core.grid=FleetGrid(message.info.width,message.info.height,message.info.resolution,
                [p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))],message.data)
            self.map_epoch=self.core.robot(self.config['reference_robot']).get('localization_epoch')
            processes=[]
            for process in Path('/proc').glob('[0-9]*'):
                try:
                    argv=(process/'cmdline').read_bytes().split(b'\0')
                    if argv and Path(argv[0].decode()).name=='mola-cli':processes.append(process.name+':'+(process/'stat').read_text().split()[21])
                except (OSError,ValueError,IndexError):pass
            self.map_source_process=Path('/proc/sys/kernel/random/boot_id').read_text().strip()+'|'+'|'.join(sorted(processes)) if processes else ''
            if self.map_epoch and self.map_epoch.rsplit(':',1)[0]!=self.map_source_process:self.map_epoch=None
            self.shared_pub.publish(message)

    def map_is_current(self,reference):
        # Relocalization changes the pose generation, not the loaded map file.
        # Require explicit alignment revalidation and the same MOLA process
        # that provided this map; a restarted process cannot reuse its old grid.
        epoch=reference.get('localization_epoch','')
        return bool(self.shared_map and self.map_source_process and
                    epoch.rsplit(':',1)[0]==self.map_source_process and reference.get('alignment_verified'))

    def publish(self,topic,data):
        m=String();m.data=json.dumps(data,ensure_ascii=False,allow_nan=False);self.pubs[topic].publish(m)

    def tick(self):
        with self.lock:
            reference=self.core.robot(self.config['reference_robot'])
            epoch=reference.get('localization_epoch','')
            if self.map_epoch is None and self.shared_map and self.map_source_process and epoch.rsplit(':',1)[0]==self.map_source_process:
                self.map_epoch=epoch
            map_state={'source_robot':self.config['reference_robot'],'sha256':self.shared_map['sha256'] if self.shared_map else '',
                'alignment_valid':self.map_is_current(reference),
                'source_age':time.monotonic()-self.map_received if self.map_received else None,
                'frame_id':'fleet_map'}
            self.core.shared_map_state=map_state
            self.map_status_pub.publish(String(data=json.dumps(map_state)))
            self.core.tick();state=self.core.status()
            self.publish('/fleet/status',state);self.publish('/fleet/mission_state',state['mission'])
            self.publish('/fleet/report',self.core.report())
            self.publish('/fleet/formation_state',{'leader_id':state['leader_id'],'type':state['formation_type'],'errors':state['formation_error']})
            for robot,data in state['robots'].items():
                self.publish('/'+robot+'/heartbeat',{'online':data['online'],'age':data.get('heartbeat_age')})
                self.publish('/'+robot+'/health',data)
                self.publish('/'+robot+'/mission_state',{'mission_id':state['mission_id'],'stage':state['current_stage'],'goal':data.get('goal')})
                self.publish('/'+robot+'/formation_error',state['formation_error'].get(robot))
            markers=MarkerArray()
            for index,(robot,data) in enumerate(state['robots'].items()):
                p=data.get('fleet_pose')
                for text_marker in (False,True):
                    marker=Marker();marker.header.frame_id='fleet_map';marker.header.stamp=self.get_clock().now().to_msg()
                    marker.ns='msc_robots';marker.id=index+(100 if text_marker else 0)
                    if p is None:marker.action=Marker.DELETE;markers.markers.append(marker);continue
                    marker.action=Marker.ADD;marker.type=Marker.TEXT_VIEW_FACING if text_marker else Marker.CUBE
                    marker.pose.position.x=p[0];marker.pose.position.y=p[1];marker.pose.position.z=.55 if text_marker else .175
                    marker.pose.orientation.z=math.sin(p[2]/2);marker.pose.orientation.w=math.cos(p[2]/2)
                    marker.scale.x=.5;marker.scale.y=.37;marker.scale.z=.18 if text_marker else .35
                    marker.color.a=1. if text_marker else .6
                    marker.color.r=1. if robot=='robot_104' else .1;marker.color.g=.4;marker.color.b=.1 if robot=='robot_104' else 1.
                    marker.text=robot+(' [leader]' if robot==state['leader_id'] else '')
                    marker.lifetime.nanosec=600000000;markers.markers.append(marker)
                if p is not None and data.get('pose_stamp_ns',0)>0:
                    transform=TransformStamped();transform.header.frame_id='fleet_map';transform.child_frame_id=robot+'/fleet_pose'
                    transform.header.stamp=Time(nanoseconds=int(data['pose_stamp_ns'])).to_msg()
                    transform.transform.translation.x=p[0];transform.transform.translation.y=p[1]
                    transform.transform.rotation.z=math.sin(p[2]/2);transform.transform.rotation.w=math.cos(p[2]/2);self.tf.sendTransform(transform)
            self.markers.publish(markers)
            for event in self.core.events:
                if event['sequence']>self.last_event:
                    self.publish('/fleet/events',event);self.last_event=event['sequence']

def main():
    p=argparse.ArgumentParser();p.add_argument('--config',required=True);args,ros=p.parse_known_args()
    config=json.load(open(args.config));rclpy.init(args=ros);node=CoordinatorNode(config)
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        node.panel.pool.shutdown(wait=False,cancel_futures=True)
        node.http.shutdown();node.http.server_close();node.destroy_node()
        rclpy.try_shutdown()
