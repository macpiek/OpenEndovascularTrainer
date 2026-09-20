import json, numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=json.load(open('/tmp/oet-closed-axis-guard/shapes.json'));C=json.load(open('/tmp/oet-axis-no-early-full/shapes.json'))
cases=[]
for r,c in zip(R,C):
 if r['phase']!='wire-in':continue
 xr=np.array(r['coordinates']);xc=np.array(c['coordinates']);a=np.array(r['positions'])+r['origin'];b=np.array(c['positions'])+c['origin'];ap=np.stack([np.interp(xc,xr,a[:,k]) for k in range(3)],axis=1)
 cases.append((np.max(np.linalg.norm(ap-b,axis=1)),r,c))
worst=max(cases,key=lambda x:x[0]);last=cases[-1]
fig,axs=plt.subplots(2,2,figsize=(10,9))
for row,(err,r,c) in enumerate([worst,last]):
 for col,horizontal in enumerate([0,2]):
  ax=axs[row,col]
  for s,color,label in [(r,'#2675c9','Previous fallback'),(c,'#db6b19','Skip early predictor fallback')]:
   p=np.array(s['positions'])+s['origin'];p=p[np.array(s['coordinates'])>500]
   ax.plot(p[:,horizontal],p[:,1],color=color,label=label,lw=1.6)
   ax.scatter(p[-1,horizontal],p[-1,1],color=color,s=24)
  ax.set_title(f"Wire {c['wire']:.1f} mm | frame max delta {err:.1f} mm")
  ax.set_xlabel(('x' if horizontal==0 else 'z')+' [mm]');ax.set_ylabel('y [mm]');ax.set_aspect('equal');ax.grid(alpha=.2)
axs[0,0].legend(fontsize=8)
fig.suptitle('Same anatomy, stiffness, feed and acceptance tolerances; different buckling trajectory')
fig.tight_layout();fig.savefig('/tmp/oet-fallback-shapes.png',dpi=150)
print('worst',worst[1]['index'],worst[1]['wire'],worst[0])
