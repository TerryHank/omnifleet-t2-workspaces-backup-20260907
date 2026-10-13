import argparse,json
from .protocol import request

def main():
    p=argparse.ArgumentParser(description='MSC-V1 fleet operations')
    p.add_argument('--config',default='/etc/omnifleet_msc/operator.json')
    p.add_argument('operation',choices=['status','report','tasks','start','pause','resume','cancel','stop','release_stop','leader','formation','alignment','zones','add_member','remove_member'])
    p.add_argument('--file');p.add_argument('--robot');p.add_argument('--type');p.add_argument('--pose',nargs=3,type=float)
    p.add_argument('--verified',action='store_true');a=p.parse_args();config=json.load(open(a.config))
    data={'op':'list' if a.operation=='tasks' else a.operation}
    if a.file:data.update(json.load(open(a.file)))
    if a.robot:data['robot_id']=a.robot
    if a.type:data['formation']=a.type
    if a.pose:data.update(transform=a.pose,verified=a.verified)
    endpoint='/v1/'+a.operation if a.operation in ('status','report','tasks') else '/v1/command'
    result=request(config['coordinator'],'operator',config['key'],endpoint,data,timeout=3)
    print(json.dumps(result,ensure_ascii=False,indent=2))
