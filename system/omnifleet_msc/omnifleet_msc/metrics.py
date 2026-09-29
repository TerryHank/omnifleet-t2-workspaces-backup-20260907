"""Measured-state bookkeeping. Missing contact feedback remains unknown, never zero."""
import copy,math,time

class Metrics:
    def __init__(self):self.previous_time=None;self.open_events={};self.contact_active={};self.contact_seen=set()
    def observe(self,metrics,states,errors,now,tracking_expected=True):
        dt=max(0.,min(1.,now-self.previous_time)) if self.previous_time is not None else 0.;self.previous_time=now
        metrics.setdefault('dropouts',{});metrics.setdefault('offline_periods',{});metrics.setdefault('fault_periods',{});metrics.setdefault('recovery_durations',[])
        valid=[s['fleet_pose'] for s in states.values() if s.get('fleet_pose') is not None]
        if len(valid)>1:
            distance=min(math.dist(a[:2],b[:2]) for i,a in enumerate(valid) for b in valid[i+1:])
            old=metrics.get('minimum_robot_distance');metrics['minimum_robot_distance']=distance if old is None else min(old,distance)
        for robot,state in states.items():
            conditions={'offline_periods':not state.get('online'),'dropouts':tracking_expected and errors.get(robot,[0])[0]>2.,
                        'fault_periods':bool(state.get('online') and (state.get('goal_error') or state.get('nav_ready') is False or state.get('fleet_pose') is None))}
            for kind,active in conditions.items():
                key=(kind,robot);record=metrics[kind].setdefault(robot,{'count':0,'duration_seconds':0.})
                if active:
                    if key not in self.open_events:self.open_events[key]=now;record['count']+=1
                    record['duration_seconds']+=dt
                elif key in self.open_events:
                    duration=now-self.open_events.pop(key)
                    metrics['recovery_durations'].append({'robot':robot,'kind':kind,'seconds':duration})
            contact=state.get('contact')
            if isinstance(contact,bool):
                self.contact_seen.add(robot)
                if contact and not self.contact_active.get(robot,False):metrics['contact_event_count']=metrics.get('contact_event_count',0)+1
                self.contact_active[robot]=contact
        metrics['physical_collision_count']=metrics.get('contact_event_count',0) if set(states).issubset(self.contact_seen) else None
        values=[v[0] for v in errors.values()]
        if values:metrics['maximum_formation_error']=max(metrics.get('maximum_formation_error',0.),max(values))
        if errors:metrics['maximum_heading_error']=max(metrics.get('maximum_heading_error',0.),max(v[1] for v in errors.values()))
        metrics['formation_error_time_integral']=metrics.get('formation_error_time_integral',0.)+(sum(values)/len(values)*dt if values else 0.)
        metrics['formation_error_observation_seconds']=metrics.get('formation_error_observation_seconds',0.)+(dt if values else 0.)
    def report(self,mission,metrics,states):
        result=copy.deepcopy(metrics);count=len(mission.get('waypoints',[]))
        result['mission_completion_rate']=1. if mission['state']=='COMPLETED' else 0.
        result['waypoint_completion_rate']={r:len(set(metrics.get('waypoint_checkpoints',{}).get(r,[])))/count if count else None for r in mission.get('members',[])}
        duration=metrics.get('formation_error_observation_seconds',0.)
        result['mean_formation_error']=metrics.get('formation_error_time_integral',0.)/duration if duration else None
        result['duration_seconds']=max(0.,time.time()-metrics.get('started_at',time.time()))
        result['final_velocities']={r:s.get('velocity') for r,s in states.items()}
        result['residual_goals']={r:bool(s.get('nav_active') or s.get('pending_goal')) for r,s in states.items()}
        result['measurement_limits']='Sampled localization and odometry; contact count requires an actual contact input. Simulation is not physical acceptance.'
        return result
