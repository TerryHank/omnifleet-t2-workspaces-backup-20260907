from fleet_scope import frame
"""One authoritative grid, transferred without resampling its cells."""
import argparse,base64,hashlib,json,threading,time,zlib,struct
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from rclpy.serialization import serialize_message,deserialize_message
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from .protocol import request
import math

QOS=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)

def encode_grid(message):
    message.header.frame_id='fleet_map'
    raw=serialize_message(message)
    if len(raw)>16_000_000:raise ValueError('shared map exceeds 16 MB')
    p=message.info.origin.position;q=message.info.origin.orientation
    geometry=struct.pack('<fII7d',message.info.resolution,message.info.width,message.info.height,p.x,p.y,p.z,q.x,q.y,q.z,q.w)
    return {'sha256':hashlib.sha256(geometry+message.data.tobytes()).hexdigest(),
            'cdr_zlib':base64.b64encode(zlib.compress(raw,3)).decode(),
            'width':message.info.width,'height':message.info.height,'resolution':message.info.resolution,'frame_id':'fleet_map'}

def decode_grid(payload):
    decoder=zlib.decompressobj();raw=decoder.decompress(base64.b64decode(payload['cdr_zlib'],validate=True),16_000_001)
    if len(raw)>16_000_000 or not decoder.eof:raise ValueError('invalid or oversized shared map')
    message=deserialize_message(raw,OccupancyGrid)
    if message.header.frame_id!='fleet_map' or message.info.width*message.info.height!=len(message.data):raise ValueError('invalid grid geometry')
    if encode_grid(message)['sha256']!=payload['sha256']:raise ValueError('shared map checksum mismatch')
    return message

class MapReceiver(Node):
    def __init__(self,config):
        super().__init__('msc_shared_map_receiver');self.config=config;self.closed=False;self.lock=threading.Lock()
        self.pending=None;self.current=None;self.received=0.;self.error='not connected';self.digest=''
        self.map_pub=self.create_publisher(OccupancyGrid,'/fleet/map',QOS)
        self.state_pub=self.create_publisher(String,'/msc/shared_map_status',10);self.tf=TransformBroadcaster(self)
        self.create_timer(.1,self.tick);self.worker=threading.Thread(target=self.poll,daemon=True);self.worker.start()
    def poll(self):
        while not self.closed:
            try:
                result=request(self.config['coordinator'],self.config['robot_id'],self.config['key'],'/v1/map',{},timeout=2)
                message=decode_grid(result['map']) if result.get('map') else None
                with self.lock:self.pending=(result,message);self.error=''
            except Exception as error:
                with self.lock:self.error=str(error)
            for _ in range(10):
                if self.closed:break
                time.sleep(.1)
    def tick(self):
        now=time.monotonic()
        with self.lock:
            pending=self.pending;self.pending=None;error=self.error
        if pending:
            result,message=pending;self.current=result;self.received=now
            if message is not None:
                self.digest=result['map']['sha256']
        valid=bool(self.current and self.current.get('alignment_valid') and now-self.received<3 and not error)
        if valid:
            p=self.current['alignment']['transform'];t=TransformStamped();t.header.frame_id='fleet_map';t.child_frame_id=frame('map')
            t.header.stamp=self.get_clock().now().to_msg();t.transform.translation.x=p[0];t.transform.translation.y=p[1]
            t.transform.rotation.z=math.sin(p[2]/2);t.transform.rotation.w=math.cos(p[2]/2);self.tf.sendTransform(t)
        self.state_pub.publish(String(data=json.dumps({'source_robot':'robot_113','sha256':self.digest,
            'alignment_valid':valid,'transport_age':now-self.received if self.received else None,'error':error})))

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--config',required=True);args,ros=parser.parse_known_args()
    rclpy.init(args=ros);node=MapReceiver(json.load(open(args.config)))
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.closed=True;node.worker.join(timeout=3);node.destroy_node();rclpy.try_shutdown()
