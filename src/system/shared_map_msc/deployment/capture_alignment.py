"""Read-only stationary scan in base_link; does not assume shared map coordinates."""
import sys,time,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer,TransformListener
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
rclpy.init();node=rclpy.create_node('msc_alignment_capture');buffer=Buffer();listener=TransformListener(buffer,node)
clouds=[];velocity=[];poses=[];last=0.;pending=None
def receive(msg):
 global pending,last
 if time.monotonic()-last>.3:pending=msg;last=time.monotonic()
node.create_subscription(PointCloud2,'/rslidar_points',receive,qos_profile_sensor_data)
node.create_subscription(Odometry,'/odom',lambda m:velocity.append([m.twist.twist.linear.x,m.twist.twist.angular.z]),20)
start=time.monotonic()
while time.monotonic()-start<12:
 rclpy.spin_once(node,timeout_sec=.02)
 if pending is None:continue
 try:transform=buffer.lookup_transform('base_link',pending.header.frame_id,Time()).transform
 except Exception:continue
 m=pending;pending=None;fields={f.name:f.offset for f in m.fields}
 dtype=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,'offsets':[fields[k] for k in ['x','y','z']],'itemsize':m.point_step})
 a=np.ndarray((m.height,m.width),dtype=dtype,buffer=m.data,strides=(m.row_step,m.point_step));xyz=np.stack([a[k].ravel() for k in ['x','y','z']],axis=1)
 xyz=xyz[np.isfinite(xyz).all(axis=1)&(np.linalg.norm(xyz,axis=1)<8)&(np.linalg.norm(xyz,axis=1)>.4)]
 q=transform.rotation;t=transform.translation;xyz=Rotation.from_quat([q.x,q.y,q.z,q.w]).apply(xyz)+[t.x,t.y,t.z]
 xyz=xyz[(xyz[:,2]>.2)&(xyz[:,2]<1.8)]
 _,idx=np.unique(np.floor(xyz/.08).astype(np.int32),axis=0,return_index=True);clouds.append(xyz[idx].astype(np.float32))
 try:
  t=buffer.lookup_transform('map','base_link',Time()).transform;q=t.rotation
  poses.append([t.translation.x,t.translation.y,Rotation.from_quat([q.x,q.y,q.z,q.w]).as_euler('xyz')[2]])
 except Exception:pass
out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=True)
assert len(clouds)>=10 and velocity and max(abs(v) for row in velocity for v in row)<.03,'not stationary or insufficient scans'
xyz=np.concatenate(clouds);cells=np.floor(xyz/.08).astype(np.int32);unique,idx,count=np.unique(cells,axis=0,return_index=True,return_counts=True)
stable=xyz[idx[count>=max(3,len(clouds)*.4)]]
np.savez_compressed(out/'alignment.npz',points=stable)
meta={'frames':len(clouds),'stable_points':len(stable),'velocity_max':np.max(np.abs(velocity),axis=0).tolist(),'map_pose':poses[-1] if poses else None,'pose_span':np.ptp(poses,axis=0).tolist() if poses else None}
(out/'alignment.json').write_text(json.dumps(meta));print(json.dumps(meta));node.destroy_node();rclpy.shutdown()
