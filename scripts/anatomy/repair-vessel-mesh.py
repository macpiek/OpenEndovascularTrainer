"""Conservative binary STL seam repair (offline dependency: numpy).

Usage: python3 repair-vessel-mesh.py INPUT.stl OUTPUT.stl REPORT.json
Does not cap anatomical branches, Boolean-union overlapping walls, or remesh
whole vessels. A 0.0001 source-unit tolerance only repairs numerical seams.
Ambiguous non-manifold junctions are reported and left for targeted inspection.
"""
import numpy as np, json, itertools, sys, hashlib, pathlib
source, output, report_path = sys.argv[1:]
if len({pathlib.Path(p).resolve() for p in [source,output,report_path]}) != 3:
 raise ValueError('Input, output, and report must be different files')
raw=pathlib.Path(source).read_bytes()
dtype=np.dtype([('n','<f4',3),('v','<f4',(3,3)),('a','<u2')])
if len(raw)<84 or len(raw)!=84+50*int.from_bytes(raw[80:84],'little'):
 raise ValueError('Expected a complete binary STL')
data=np.frombuffer(raw,dtype=dtype,offset=84)
if not np.isfinite(data['v']).all():raise ValueError('Non-finite vertex coordinates')
v,f=np.unique(data['v'].reshape(-1,3),axis=0,return_inverse=True)
v=v.astype(float);f=f.reshape(-1,3);tol=1e-4
iterations=[]


def clean(f):
 cross=np.cross(v[f[:,1]]-v[f[:,0]],v[f[:,2]]-v[f[:,0]])
 good=np.sum(cross*cross,axis=1)>1e-24
 f=f[good];_,ids=np.unique(np.sort(f,axis=1),axis=0,return_index=True)
 return f[np.sort(ids)]
def edges(f):
 e=f[:,[[0,1],[1,2],[2,0]]].reshape(-1,2);ue,first,inv,count=np.unique(np.sort(e,axis=1),axis=0,return_index=True,return_inverse=True,return_counts=True)
 return e,ue,first,inv,count
def audit(v,f):
 e,ue,first,inv,count=edges(f)
 sign=np.where(e[:,0]<e[:,1],1,-1)
 orientation=np.bincount(inv,weights=sign)
 cross=np.cross(v[f[:,1]]-v[f[:,0]],v[f[:,2]]-v[f[:,0]])
 return dict(triangles=len(f),vertices=len(np.unique(f)),
  zeroArea=int((np.sum(cross*cross,axis=1)<=1e-24).sum()),
  duplicates=len(f)-len(np.unique(np.sort(f,axis=1),axis=0)),
  boundaryEdges=int((count==1).sum()),nonManifoldEdges=int((count>2).sum()),
  inconsistentWinding=int(((count==2)&(orientation!=0)).sum()))
before=audit(v,f)
f=clean(f)
clean_topology=audit(v,f)
for iteration in range(3):
 e,ue,first,inv,count=edges(f);boundary=ue[count==1];ids=np.unique(boundary);print('iteration',iteration,'faces',len(f),'boundary',len(boundary),'nonman',int((count>2).sum()),flush=True)
 # Weld only boundary vertices within one tenth of a micron in source units.
 grid={};lookup=np.arange(len(v));weld=0
 for i in ids:
  p=v[i];key=tuple(np.floor(p/tol).astype(int));found=None
  for delta in itertools.product([-1,0,1],repeat=3):
   for j in grid.get(tuple(key[k]+delta[k] for k in range(3)),[]):
    if np.linalg.norm(p-v[j])<=tol:found=j;break
   if found is not None:break
  if found is not None:lookup[i]=found;weld+=1
  else:grid.setdefault(key,[]).append(i)
 # Reject weld groups that introduce a newly non-manifold edge.
 for attempt in range(3):
  candidate=clean(lookup[f]);ce,cu,cf,ci,cc=edges(candidate)
  directions=np.bincount(ci,weights=np.where(ce[:,0]<ce[:,1],1,-1))
  bad=cu[(cc>2)|((cc==2)&(directions!=0))]; targets=np.unique(bad); changed=np.where(lookup!=np.arange(len(v)))[0]
  undo=changed[np.isin(lookup[changed],targets)]
  if not len(undo):break
  lookup[undo]=undo
 weld=int((lookup!=np.arange(len(v))).sum())
 f=clean(lookup[f]);e,ue,first,inv,count=edges(f);boundary=ue[count==1];ids=np.unique(boundary)
 counts={tuple(edge):int(c) for edge,c in zip(ue,count)}
 directions={tuple(edge):int(s) for edge,s in zip(ue,np.bincount(inv,weights=np.where(e[:,0]<e[:,1],1,-1)))}
 print('after weld nonman',int((count>2).sum()),flush=True)
 grid={}
 for i in ids:grid.setdefault(tuple(np.floor(v[i]).astype(int)),[]).append(i)
 splits={};n=0
 for a,b in boundary:
  A=v[a];B=v[b];d=B-A;length=np.linalg.norm(d)
  if length<tol*2:continue
  # Grid traversal with neighboring cells, rather than huge 3D AABBs.
  keys=set()
  for p in np.linspace(A,B,max(2,int(np.ceil(length*2))+1)):
   key=np.floor(p).astype(int)
   for delta in itertools.product([-1,0,1],repeat=3):keys.add(tuple(key+delta))
  candidates=np.array(list({i for key in keys for i in grid.get(key,[]) if i!=a and i!=b}),dtype=int)
  if not len(candidates):continue
  t=((v[candidates]-A)@d)/(length*length);dist=np.linalg.norm(v[candidates]-(A+t[:,None]*d),axis=1)
  good=(t>tol/length)&(t<1-tol/length)&(dist<=tol)
  if good.any():
   order=np.argsort(t[good]);inside=candidates[good][order].tolist();chain=[int(a)]+inside+[int(b)]
   pieces=[tuple(sorted(pair)) for pair in zip(chain,chain[1:])]
   if counts.get((int(a),int(b)),0)!=1 or any(counts.get(k,0)>1 for k in pieces):continue
   direction=directions[(int(a),int(b))]
   signs=[direction*(1 if x<y else -1) for x,y in zip(chain,chain[1:])]
   if any(counts.get(k,0)==1 and directions[k]+sign!=0 for k,sign in zip(pieces,signs)):continue
   if len(set(chain))!=len(chain):continue
   splits[(int(a),int(b))]=inside;n+=good.sum()
   counts[(int(a),int(b))]=0
   for k,sign in zip(pieces,signs):
    counts[k]=counts.get(k,0)+1
    directions[k]=directions.get(k,0)+sign
 new=[];newv=v.tolist()
 for tri in f:
  poly=[];changed=False
  for a,b in zip(tri,np.roll(tri,-1)):
   poly.append(int(a));key=tuple(sorted((int(a),int(b))));inside=splits.get(key)
   if inside:poly.extend(inside if a<b else inside[::-1]);changed=True
  if changed and len(set(poly))==len(poly):
   center=len(newv);newv.append(v[tri].mean(0).tolist())
   new.extend((poly[i],poly[(i+1)%len(poly)],center) for i in range(len(poly)))
  else:new.append(tri.tolist())
 v=np.array(newv,dtype=np.float32).astype(float);f=clean(np.array(new,dtype=int));print('welded',weld,'split edges',len(splits),'points',int(n),flush=True)
 iterations.append(dict(welded=weld,splitEdges=len(splits),insertedEdgePoints=int(n)))
 if not weld and not splits:break

# Orient each orientable manifold patch consistently, preserving its majority
# orientation (an inner vessel surface must not be flipped to face outwards).
from collections import deque
e,ue,first,inv,count=edges(f)
paired=np.where(count==2)[0];a=first[paired]
other=np.bincount(inv,weights=np.arange(len(e))).astype(np.int64)[paired]-a
neighbors=np.full((len(f),3),-1,dtype=np.int32)
parity=np.zeros((len(f),3),dtype=bool)
same=e[a,0]==e[other,0]
neighbors.flat[a]=other//3;neighbors.flat[other]=a//3
parity.flat[a]=same;parity.flat[other]=same
flip=np.full(len(f),-1,dtype=np.int8);flipped=0;conflicted=0;components=0
for seed in range(len(f)):
 if flip[seed]!=-1:continue
 components+=1;flip[seed]=0;queue=deque([seed]);patch=[];conflict=False
 while queue:
  face=queue.popleft();patch.append(face)
  for k in range(3):
   neighbor=neighbors[face,k]
   if neighbor<0:continue
   expected=int(flip[face]) ^ int(parity[face,k])
   if flip[neighbor]==-1:flip[neighbor]=expected;queue.append(neighbor)
   elif flip[neighbor]!=expected:conflict=True
 patch=np.array(patch)
 if conflict:flip[patch]=0;conflicted+=1;continue
 if flip[patch].sum()>len(patch)/2:flip[patch]=1-flip[patch]
 selected=patch[flip[patch]==1];f[selected]=f[selected][:,[0,2,1]];flipped+=len(selected)
after=audit(v,f)
if after['nonManifoldEdges']>clean_topology['nonManifoldEdges'] or after['boundaryEdges']>clean_topology['boundaryEdges']:
 raise ValueError('Repair made topology worse; refusing output')
if after['inconsistentWinding']>clean_topology['inconsistentWinding']:
 raise ValueError('Repair introduced inconsistent face orientation; refusing output')
tri=v[f].astype(np.float32)
cross=np.cross(tri[:,1].astype(float)-tri[:,0],tri[:,2].astype(float)-tri[:,0])
norm=np.linalg.norm(cross,axis=1)
if np.any(norm<=1e-12):raise ValueError('Repair generated degenerate faces')
result=np.zeros(len(f),dtype=dtype);result['v']=tri;result['n']=cross/norm[:,None]
encoded=raw[:80]+len(f).to_bytes(4,'little')+result.tobytes()
pathlib.Path(output).parent.mkdir(parents=True,exist_ok=True)
pathlib.Path(output).write_bytes(encoded)
report=dict(kind='surface-repair',sourcePath=source,outputPath=output,
 sourceSha256=hashlib.sha256(raw).hexdigest(),outputSha256=hashlib.sha256(encoded).hexdigest(),
 toleranceSourceUnits=tol,before=before,after=after,iterations=iterations,
 flippedTriangles=flipped,manifoldPatches=components,nonOrientablePatches=conflicted,
 method='Remove zero-area/duplicate faces; weld compatible boundary vertices; conform T-junctions; orient manifold patches. No anatomical hole capping or Boolean union.')
pathlib.Path(report_path).write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report),flush=True)
