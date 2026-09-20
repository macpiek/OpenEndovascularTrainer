import json, numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R={s['index']:s for s in json.load(open('/tmp/oet-axis-no-early-full/shapes.json'))}
C=json.load(open('/tmp/oet-closed-compliance-pdas/shapes.json'))
phases=['wire-in','catheter-in','catheter-out','wire-out']
fig,axs=plt.subplots(4,2,figsize=(13,17))
for row,phase in enumerate(phases):
 cases=[]
 for c in C:
  if c['phase']!=phase:continue
  r=R[c['index']];xr=np.array(r['coordinates']);xc=np.array(c['coordinates']);a=np.array(r['positions'])+r['origin'];b=np.array(c['positions'])+c['origin'];ap=np.stack([np.interp(xc,xr,a[:,k]) for k in range(3)],axis=1)
  cases.append((np.max(np.linalg.norm(ap-b,axis=1)),r,c))
 err,r,c=max(cases,key=lambda x:x[0])
 for col,horizontal in enumerate([0,2]):
  ax=axs[row,col]
  for s,color,label in [(r,'#2675c9','Reference'),(c,'#db6b19','Compliant contact + batch release')]:
   p=np.array(s['positions'])+s['origin'];p=p[np.array(s['coordinates'])>450]
   ax.plot(p[:,horizontal],p[:,1],color=color,label=label,lw=1.6)
   ax.scatter(p[-1,horizontal],p[-1,1],color=color,s=22)
  ax.set_title(f"{phase}: W {c['wire']:.0f} / C {c['catheter']:.0f} mm\nMaximum shape difference {err:.1f} mm",fontsize=10)
  ax.set_xlabel(('x' if horizontal==0 else 'z')+' [mm]');ax.set_ylabel('y [mm]');ax.set_aspect('equal');ax.grid(alpha=.2)
fig.legend(*axs[0,0].get_legend_handles_labels(),loc='lower center',ncol=2,fontsize=10)
fig.suptitle('Worst saved shape difference in each phase; distal segment after arc 450 mm')
fig.tight_layout(rect=(0,.03,1,.97),h_pad=3,w_pad=2);fig.savefig('/tmp/oet-compliance-shapes.png',dpi=135)
