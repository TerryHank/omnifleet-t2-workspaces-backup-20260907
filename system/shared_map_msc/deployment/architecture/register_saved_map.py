"""Offline static scan registration against the requested .mm map; candidate only."""
import argparse,json,math
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scipy.signal import fftconvolve
from scipy.ndimage import gaussian_filter
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

p=argparse.ArgumentParser();p.add_argument('reference');p.add_argument('scan');p.add_argument('output');a=p.parse_args()
ref3=np.loadtxt(a.reference);src3=np.load(a.scan)['points']
def down(points):
 points=points[(points[:,2]>.2)&(points[:,2]<1.8),:2]
 _,idx=np.unique(np.floor(points/.08).astype(int),axis=0,return_index=True)
 return points[idx]
ref=down(ref3);src=down(src3);resolution=.1;size=401;center=200
assert len(ref)>100 and len(src)>100
def rotation(theta):return np.array([[math.cos(theta),-math.sin(theta)],[math.sin(theta),math.cos(theta)]])
def raster(points):
 grid=np.zeros((size,size));ij=np.rint(points/resolution).astype(int)+center;ij=ij[(ij>=0).all(axis=1)&(ij<size).all(axis=1)]
 grid[ij[:,1],ij[:,0]]=1;return grid
reference=gaussian_filter(raster(ref),.8);candidates=[]
for degree in range(-180,180,2):
 theta=math.radians(degree);c=fftconvolve(reference,raster(src@rotation(theta).T)[::-1,::-1],mode='full')
 y,x=np.unravel_index(np.argmax(c),c.shape);candidates.append((float(c[y,x]),theta,np.array([(x-400)*resolution,(y-400)*resolution])))
selected=[]
for score,theta,translation in sorted(candidates,key=lambda x:-x[0]):
 if all(abs(math.atan2(math.sin(theta-t),math.cos(theta-t)))>.2 or np.linalg.norm(translation-p)>.6 for _,t,p in selected):selected.append((score,theta,translation))
 if len(selected)>=12:break
tree=cKDTree(ref);results=[]
for score,theta,translation in selected:
 matrix=rotation(theta)
 for _ in range(50):
  transformed=src@matrix.T+translation;distance,index=tree.query(transformed);mask=distance<.5
  if mask.sum()<30:break
  mask&=distance<=np.quantile(distance[mask],.8);left=transformed[mask];right=ref[index[mask]]
  lc=left.mean(axis=0);rc=right.mean(axis=0);u,_,vt=np.linalg.svd((left-lc).T@(right-rc));delta=vt.T@u.T
  if np.linalg.det(delta)<0:vt[-1]*=-1;delta=vt.T@u.T
  shift=rc-delta@lc;matrix=delta@matrix;translation=delta@translation+shift
  if np.linalg.norm(shift)<1e-4:break
 distance,_=tree.query(src@matrix.T+translation);good=distance<.2
 overlap=float(good.mean());rmse=float(np.sqrt(np.mean(distance[good]**2))) if good.any() else 999.
 results.append({'pose':[float(translation[0]),float(translation[1]),math.atan2(matrix[1,0],matrix[0,0])],
                 'scan_overlap':overlap,'inlier_rmse':rmse,'score':overlap/(1+rmse*5)})
results.sort(key=lambda x:-x['score']);best=results[0];out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
(out/'registration.json').write_text(json.dumps({'candidate_only':True,'reference_points':len(ref),'scan_points':len(src),'candidates':results},indent=2))
pose=best['pose'];transformed=src@rotation(pose[2]).T+pose[:2]
fig,ax=plt.subplots(figsize=(10,7));ax.scatter(ref[:,0],ref[:,1],s=3,c='#777',label='foxglove_map.mm');ax.scatter(transformed[:,0],transformed[:,1],s=2,c='#e46b17',label='current scan')
ax.arrow(pose[0],pose[1],.6*math.cos(pose[2]),.6*math.sin(pose[2]),width=.08,color='#1565c0');ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend()
ax.set_title(f'Saved map registration: overlap {best["scan_overlap"]:.1%}, RMSE {best["inlier_rmse"]:.3f}m');fig.tight_layout();fig.savefig(out/'overlay.png',dpi=140)
print(json.dumps(best))
