"""Fixed T2 visual geometry from its authoritative URDF, not a second model."""
import math
import xml.etree.ElementTree as ET

def multiply(a,b):
 x,y,z,w=a;X,Y,Z,W=b
 return [w*X+x*W+y*Z-z*Y,w*Y-x*Z+y*W+z*X,w*Z+x*Y-y*X+z*W,w*W-x*X-y*Y-z*Z]
def rotate(q,v):return multiply(multiply(q,[*v,0]),[-q[0],-q[1],-q[2],q[3]])[:3]
def combine(a,b):
 v=rotate(a[1],b[0]);return ([a[0][i]+v[i] for i in range(3)],multiply(a[1],b[1]))
def inverse(a):
 q=[-a[1][0],-a[1][1],-a[1][2],a[1][3]];return (rotate(q,[-v for v in a[0]]),q)
def origin(element):
 if element is None:return ([0.,0.,0.],[0.,0.,0.,1.])
 xyz=list(map(float,element.get('xyz','0 0 0').split()));r,p,y=[v/2 for v in map(float,element.get('rpy','0 0 0').split())]
 return xyz,multiply(multiply([0,0,math.sin(y),math.cos(y)],[0,math.sin(p),0,math.cos(p)]),[math.sin(r),0,0,math.cos(r)])

def load_visuals(path):
 root=ET.parse(path).getroot();materials={}
 for m in root.findall('material'):
  c=m.find('color')
  if c is not None:materials[m.get('name')]=list(map(float,c.get('rgba').split()))
 edges={}
 for j in root.findall('joint'):
  if j.get('type')!='fixed':raise ValueError('T2 fleet visual renderer requires fixed joints')
  parent=j.find('parent').get('link');child=j.find('child').get('link');t=origin(j.find('origin'))
  edges.setdefault(parent,[]).append((child,t));edges.setdefault(child,[]).append((parent,inverse(t)))
 frames={'base_link':([0.,0.,0.],[0.,0.,0.,1.])};queue=['base_link']
 while queue:
  parent=queue.pop(0)
  for child,t in edges.get(parent,[]):
   if child not in frames:frames[child]=combine(frames[parent],t);queue.append(child)
 result=[]
 for link in root.findall('link'):
  for visual in link.findall('visual'):
   t=combine(frames[link.get('name')],origin(visual.find('origin')));shape=list(visual.find('geometry'))[0]
   size=list(map(float,shape.get('size','').split())) if shape.tag=='box' else None
   if shape.tag=='cylinder':size=[2*float(shape.get('radius'))]*2+[float(shape.get('length'))]
   if shape.tag=='sphere':size=[2*float(shape.get('radius'))]*3
   if shape.tag=='mesh':size=list(map(float,shape.get('scale','1 1 1').split()))
   if size is None:raise ValueError('Unsupported URDF visual '+shape.tag)
   material=visual.find('material');color=[.6,.6,.6,1.]
   if material is not None:
    color=materials.get(material.get('name'),color);c=material.find('color')
    if c is not None:color=list(map(float,c.get('rgba').split()))
   result.append({'link':link.get('name'),'shape':shape.tag,'size':size,'transform':t,'color':color,'mesh':shape.get('filename','')})
 return result

def fleet_root(robot):
 p=robot['pose'];local=robot.get('local_pose_3d');alignment=robot.get('alignment') or [0,0,0]
 if local:
  q=multiply([0,0,math.sin(alignment[2]/2),math.cos(alignment[2]/2)],local[3:7]);z=local[2]
 else:q=[0,0,math.sin(p[2]/2),math.cos(p[2]/2)];z=0.
 return ([p[0],p[1],z],q)
