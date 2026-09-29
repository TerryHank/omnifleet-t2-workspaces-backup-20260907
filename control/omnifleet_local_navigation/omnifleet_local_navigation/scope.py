import os
def robot_id():return os.environ['OMNIFLEET_ROBOT_ID']
def frame(name):return name if name.startswith(robot_id()+'/') else robot_id()+'/'+name.lstrip('/')
def topic(name):
    if name.startswith('/fleet/') or name.startswith('/robot_'):return name
    return '/'+robot_id()+'/'+name.lstrip('/')
