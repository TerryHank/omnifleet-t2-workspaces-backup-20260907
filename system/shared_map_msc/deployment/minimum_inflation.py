import json,tempfile,os,time
from pathlib import Path
import yaml,rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters,SetParametersAtomically
from rcl_interfaces.msg import Parameter,ParameterValue
rclpy.init();n=Node('msc_inflation_precheck');changes={}
def call(typ,path,req):
 c=n.create_client(typ,path);assert c.wait_for_service(timeout_sec=5)
 f=c.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=5);assert f.done();return f.result()
target=Path('/home/iecme/workspace/src/omnifleet_planner/config/nav2_t2.yaml')
old=target.read_bytes();cfg=yaml.safe_load(old)
backup=Path('/home/iecme/robot_backups/msc_v1_20260909')/('nav2-before-inflation-'+str(time.time_ns())+'.yaml');backup.write_bytes(old)
for scope in ('local','global'):
 node=f'/{scope}_costmap/{scope}_costmap';name='inflation_layer.inflation_radius'
 value=call(GetParameters,node+'/get_parameters',GetParameters.Request(names=[name])).values[0].double_value
 proposed=max(value,.6)
 reply=call(SetParametersAtomically,node+'/set_parameters_atomically',SetParametersAtomically.Request(parameters=[Parameter(name=name,value=ParameterValue(type=3,double_value=proposed))]))
 assert reply.result.successful,reply.result.reason
 actual=call(GetParameters,node+'/get_parameters',GetParameters.Request(names=[name])).values[0].double_value
 assert abs(actual-proposed)<1e-8
 cfg[scope+'_costmap'][scope+'_costmap']['ros__parameters']['inflation_layer']['inflation_radius']=proposed
 changes[scope]={'before':value,'after':actual}
fd,tmp=tempfile.mkstemp(prefix='.msc-inflation-',dir=target.parent)
with os.fdopen(fd,'w') as stream:yaml.safe_dump(cfg,stream,sort_keys=False,allow_unicode=True);stream.flush();os.fsync(stream.fileno())
os.chmod(tmp,target.stat().st_mode & 0o777);os.replace(tmp,target)
print(json.dumps(changes));n.destroy_node();rclpy.try_shutdown()
