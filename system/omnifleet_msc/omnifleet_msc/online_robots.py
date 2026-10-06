"""Discover live namespaced vehicles for display, without granting control."""
import copy,json,re,time

class OnlineRegistry:
    def __init__(self,clock=time.monotonic,timeout=2.):
        self.clock=clock;self.timeout=timeout;self.streams={}

    def observe(self,robot,topic):
        now=self.clock();key=(robot,topic);old=self.streams.get(key)
        first=old[0] if old and now-old[1]<self.timeout else now
        self.streams[key]=(first,now)

    def online(self):
        now=self.clock()
        self.streams={k:v for k,v in self.streams.items() if now-v[1]<self.timeout}
        return {robot for (robot,_),(first,last) in self.streams.items() if last-first>=.05}

    def decorate(self,state):
        result=copy.deepcopy(state);known={r['id']:r for r in result['robots']};rows=[]
        for robot in sorted(self.online()):
            if robot in known:
                row=known[robot];row['online']=True;row['registered']=True
                if not row.get('ready') and row.get('health')=='OFFLINE':
                    row.update(health='WAITING_FLEET',pose=None,reason='域内在线，等待协同状态连接')
            else:
                row={'id':robot,'name':robot,'online':True,'registered':False,'active':False,'rank':None,
                     'pose':None,'ready':False,'busy':False,'supports_route':False,
                     'health':'DISCOVERED','reason':'域内在线，尚未接入协同控制接口','planned_path':[]}
            rows.append(row)
        result['robots']=rows
        # This is display state only. Never release tasks or erase offline routes.
        config=result['config'];ids={r['id'] for r in rows}
        config['active']=[r for r in config['active'] if r in ids]
        if config.get('selected') not in ids:config['selected']=None
        if config.get('leader') not in ids:config['leader']=None
        return result

class DomainVehicleDiscovery(OnlineRegistry):
    def __init__(self,node):
        super().__init__();self.node=node;self.subscriptions={}
        node.create_timer(1.,self.scan)

    def scan(self):
        from std_msgs.msg import String
        from nav_msgs.msg import Odometry
        from rclpy.qos import qos_profile_sensor_data
        targets={}
        for topic,types in self.node.get_topic_names_and_types():
            match=re.fullmatch(r'/([A-Za-z_][A-Za-z0-9_]*)/(navigation/local_status|msc/local_status|odom)',topic)
            if not match:continue
            robot,kind=match.groups()
            expected='nav_msgs/msg/Odometry' if kind=='odom' else 'std_msgs/msg/String'
            if expected not in types:continue
            targets[topic]=(robot,kind)
            if topic in self.subscriptions:continue
            def receive(message,r=robot,t=topic,k=kind):
                if k!='odom':
                    try:
                        if json.loads(message.data).get('robot_id')!=r:return
                    except (ValueError,AttributeError):return
                self.observe(r,t)
            self.subscriptions[topic]=self.node.create_subscription(Odometry if kind=='odom' else String,topic,receive,qos_profile_sensor_data)
        for topic in list(self.subscriptions):
            if topic not in targets:self.node.destroy_subscription(self.subscriptions.pop(topic))

