// Probe actual anatomy around the private geometry of a captured app replay.
// Requires diagnostics produced after the rejected-geometry snapshot fix.
import {readFileSync} from 'node:fs';
import {createCompositeAnatomyField} from '../tests/helpers/compositeAnatomyField.js';
import {createCompositeWallSdfBranchesWorkspace,evaluateCompositeWallSdfBranches} from '../src/physics/kirchhoffCompositeWallSdfBranches.js';
import {createContactResult} from '../src/physics/collision/vesselContactField.js';
const report=JSON.parse(readFileSync(process.argv[2],'utf8')),d=report.result.diagnostics,positions=new Map(d.rejectedConfiguration.positions),
 origin=JSON.parse(readFileSync(new URL('../tests/fixtures/compositeNativeAppInitial.json',import.meta.url),'utf8')).context.sheaths[0].start,
 anatomy=createCompositeAnatomyField(),field=anatomy.field;
function query(p,r){const c=field.queryCapsuleCoordinates(...p,...p,r,createContactResult());return {gap:c.signedGap,normal:Array.from(c.inward.values),source:c.source};}
const samples=[];
try{for(const s of d.certificate.wall.samples.filter(s=>s.Fn>1)){
 const f=s.endpointFraction,a=positions.get(s.owner)[s.edge],b=positions.get(s.owner)[s.edge+1],p=a.map((v,k)=>v+f*(b[k]-v)+origin[k]),radius=JSON.parse(s.key)[2],
 grid=p.map((v,k)=>(v-field.sdfOrigin[k])/field.voxelSize),base=query(p,radius),jumps=[];
 for(let k=0;k<3;k++){const plus=p.slice(),minus=p.slice();plus[k]+=1e-7;minus[k]-=1e-7;const u=query(plus,radius),v=query(minus,radius);jumps.push(Math.hypot(...u.normal.map((x,i)=>x-v.normal[i])));}
 const axis=2,face=Math.round(grid[axis]),u=p.slice(),v=p.slice();u[axis]=field.sdfOrigin[axis]+face*field.voxelSize+1e-9;v[axis]=field.sdfOrigin[axis]+face*field.voxelSize-1e-9;const uq=query(u,radius),vq=query(v,radius);
 const domainBox={lower:p.map(x=>x-.02),upper:p.map(x=>x+.02)},chart=Math.abs(p[axis]-(field.sdfOrigin[axis]+face*field.voxelSize))<.02?evaluateCompositeWallSdfBranches({field,face:{axis,gridIndex:face},positions:[p],radius,contact:field.queryCapsuleCoordinates(...p,...p,radius,createContactResult()),domainBox},createCompositeWallSdfBranchesWorkspace(1)):{supported:false,reason:'face-outside-local-box'};
 samples.push({localChart:{supported:chart.supported,reason:chart.reason,classification:chart.classification,domainBox,insideProof:chart.localInsideProof,jumpBounds:[chart.jumpLowerBound,chart.jumpUpperBound]},edge:s.edge,f,Fn:s.Fn,grid,...base,normalJumpAcross2eMinus7Mm:jumps,nearestZFace:{face,plus:uq,minus:vq,forceJump:s.Fn*Math.hypot(...uq.normal.map((x,k)=>x-vq.normal[k]))}});
}}finally{anatomy.dispose();}

console.log(JSON.stringify({scope:'captured-original-anatomy-rejected-point-probe',sampleOffsetMm:1e-7,faceOffsetMm:1e-9,source:process.argv[2],samples},null,2));
