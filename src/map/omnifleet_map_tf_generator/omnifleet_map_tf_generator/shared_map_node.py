from omnifleet_fleet_common.fleet_scope import frame
"""One authoritative grid, transferred without resampling its cells."""
import argparse,json,threading,time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from omnifleet_fleet_common.protocol import request

from omnifleet_map_tf_generator.grid_codec import QOS, encode_grid, decode_grid
from omnifleet_map_tf_generator.map_tf import planar_transform



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
            p=self.current['alignment']['transform']
            t=planar_transform('fleet_map',frame('map'),p,self.get_clock().now().to_msg());self.tf.sendTransform(t)
            if self.config.get('localization_source')=='odom':
                bridge=planar_transform(frame('map'),frame('odom'),[0.,0.,0.],t.header.stamp)
                self.tf.sendTransform(bridge)
        self.state_pub.publish(String(data=json.dumps({'source_robot':'robot_113','sha256':self.digest,
            'alignment_valid':valid,'transport_age':now-self.received if self.received else None,'error':error})))

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--config',required=True);args,ros=parser.parse_known_args()
    rclpy.init(args=ros);node=MapReceiver(json.load(open(args.config)))
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.closed=True;node.worker.join(timeout=3);node.destroy_node();rclpy.try_shutdown()
