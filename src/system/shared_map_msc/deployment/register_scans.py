import json,math,sys
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scipy.signal import fftconvolve
from scipy.ndimage import gaussian_filter
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root=Path(sys.argv[1]);ref3=np.load(root/'alignment113.npz')['points'];src3=np.load(root/'alignment104.npz')['points']
def down(points):
 _,index=np.unique(np.floor(points[:,:2]/.06).astype(int),axis=0,return_index=True)
 return points[index,:2]
ref,src=down(ref3),down(src3);resolution=.1;size=201;center=100
def rotation(theta):return np.array([[math.cos(theta),-math.sin(theta)],[math.sin(theta),math.cos(theta)]])
def raster(points):
 grid=np.zeros((size,size));ij=np.rint(points/resolution).astype(int)+center
 good=(ij>=0).all(axis=1)&(ij<size).all(axis=1);ij=ij[good];grid[ij[:,1],ij[:,0]]=1
 return grid
reference=gaussian_filter(raster(ref),.8);candidates=[]
for degree in range(-180,180,2):
 theta=math.radians(degree);correlation=fftconvolve(reference,raster(src@rotation(theta).T)[::-1,::-1],mode='full')
 yy,xx=np.indices(correlation.shape);distance=np.hypot(xx-200,yy-200)*resolution
 correlation[(distance>8)|(distance<.35)]=-1
 index=np.argmax(correlation);y,x=np.unravel_index(index,correlation.shape)
 candidates.append((float(correlation[y,x]),theta,np.array([(x-200)*resolution,(y-200)*resolution])))
selected=[]
for score,theta,translation in sorted(candidates,key=lambda c:-c[0]):
 if all(abs(math.atan2(math.sin(theta-t),math.cos(theta-t)))>.2 or np.linalg.norm(translation-p)>.6 for _,t,p in selected):
  selected.append((score,theta,translation))
 if len(selected)>=12:break
tree=cKDTree(ref);results=[]
for score,theta,translation in selected:
 matrix=rotation(theta)
 for _ in range(45):
  transformed=src@matrix.T+translation;distance,index=tree.query(transformed);mask=distance<.35
  if mask.sum()<20:break
  mask&=distance<=np.quantile(distance[mask],.85)
  a=transformed[mask];b=ref[index[mask]];ac=a.mean(axis=0);bc=b.mean(axis=0)
  u,_,vt=np.linalg.svd((a-ac).T@(b-bc));delta=vt.T@u.T
  if np.linalg.det(delta)<0:vt[-1]*=-1;delta=vt.T@u.T
  shift=bc-delta@ac;matrix=delta@matrix;translation=delta@translation+shift
  if np.linalg.norm(shift)<1e-4 and abs(math.atan2(delta[1,0],delta[0,0]))<1e-4:break
 transformed=src@matrix.T+translation;distance,_=tree.query(transformed);back,_=cKDTree(transformed).query(ref)
 overlap=float(((distance<.15).mean()+(back<.15).mean())/2);rmse=float(np.sqrt(np.mean(distance[distance<.15]**2)))
 results.append({'translation':translation.tolist(),'yaw':math.atan2(matrix[1,0],matrix[0,0]),'overlap':overlap,'inlier_rmse':rmse,'score':overlap/(1+rmse*5)})
results.sort(key=lambda r:-r['score']);best=results[0]
p113=json.loads((root/'alignment113.json').read_text())['map_pose'];p104=json.loads((root/'alignment104.json').read_text())['map_pose']
def compose(a,b):
 xy=np.array(a[:2])+rotation(a[2])@np.array(b[:2]);return [*xy,math.atan2(math.sin(a[2]+b[2]),math.cos(a[2]+b[2]))]
def inv(a):return [*(-rotation(-a[2])@np.array(a[:2])),-a[2]]
body104=compose(p113,[*best['translation'],best['yaw']]);alignment=compose(body104,inv(p104))
report={'candidate_only':True,'reference_map_pose113':p113,'body104_in_map113':body104,'transform_map104_to_map113':alignment,'candidates':results}
(root/'registration.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
a=ref@rotation(p113[2]).T+np.array(p113[:2]);b=src@rotation(body104[2]).T+np.array(body104[:2])
fig,ax=plt.subplots(figsize=(10,9));ax.scatter(a[:,0],a[:,1],s=2,c='#1565c0',label='113 scan');ax.scatter(b[:,0],b[:,1],s=2,c='#ef6c00',label='104 scan (candidate alignment)')
for name,p,color in [('113',p113,'#1565c0'),('104',body104,'#ef6c00')]:
 ax.scatter([p[0]],[p[1]],s=100,c=color,marker='s');ax.arrow(p[0],p[1],.5*math.cos(p[2]),.5*math.sin(p[2]),width=.05,color=color);ax.text(p[0]+.15,p[1]+.15,name,fontsize=16,weight='bold')
ax.set_aspect('equal');ax.grid(alpha=.25);ax.legend();ax.set(xlabel='113 map X (m)',ylabel='113 map Y (m)',title=f'Candidate only: overlap {best["overlap"]:.1%}, inlier RMSE {best["inlier_rmse"]:.3f} m')
fig.tight_layout();fig.savefig(root/'registration-overlay.png',dpi=150)
