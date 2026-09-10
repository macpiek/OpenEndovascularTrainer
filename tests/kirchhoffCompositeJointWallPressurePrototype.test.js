import assert from 'node:assert/strict';
import test from 'node:test';
import {createCompositeJointWallPressurePrototype as create,JOINT_WALL_PRESSURE_SCHEME} from '../src/physics/kirchhoffCompositeJointWallPressurePrototype.js';
import {createCompositeWallEnvelopeWorkspace,refreshCompositeWallEnvelope} from '../src/physics/kirchhoffCompositeWallEnvelope.js';
import {createContactResult} from '../src/physics/collision/vesselContactField.js';
import {captureCompositeReferenceFrames,compileCompositeMaterial} from '../src/physics/kirchhoffCompositeElement.js';
import {createCompositeChainLayout,createCompositeChainWorkspace,assembleCompositeChain} from '../src/physics/kirchhoffCompositeChain.js';
import {assembleCompositeTranslationalInertia} from '../src/physics/kirchhoffCompositeKinematics.js';
import {assembleCompositeRelativeCluster} from '../src/physics/kirchhoffCompositeRelativeCluster.js';
import {createCompositeToolLengthWorkspace,evaluateCompositeToolLengths} from '../src/physics/kirchhoffCompositeToolLengths.js';
import {createCompositeRelativeDirectionWorkspace,solveCompositeRelativeDirection} from '../src/physics/kirchhoffCompositeRelativeDirection.js';
import {evaluateCompositeJointSurfaceMotion} from '../src/physics/kirchhoffCompositeJointSurfaceMotion.js';
const close=(a,b,t=1e-8)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t,`${a} != ${b}`);
const same=(a,b,t)=>{assert.equal(a.length,b.length);a.forEach((v,i)=>close(v,b[i],t));};
const tol={gap:1e-8,normalEquation:1e-8,work:1e-9,frictionSlip:1e-8,frictionCone:1e-9,frictionWork:1e-9,frictionEquation:1e-8,linearConstraint:1e-10};
const dot=(a,b)=>a.reduce((s,x,k)=>s+x*b[k],0),cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
function fixture({right=true,gauges=[0,0]}={}){
    const coordinates=[0,2,4],positions=coordinates.map(x=>[x,.8,0]),layout=createCompositeChainLayout([['wire'],['wire']]),angles=new Float64Array(2),dt=1/120;
    const reference=captureCompositeReferenceFrames(positions),owners={edges:[0,1].map(edge=>({edge,wall:{owner:'wire',radius:.8}}))};
    const field={calls:0,queryCapsuleCoordinates(ax,ay,az,bx,by,bz,r,out=createContactResult()){
        this.calls++;const t=ay<by?0:1,p=t===0?[ax,ay,az]:[bx,by,bz];out.signedDistance=p[1];out.signedGap=p[1]-r;out.segmentT=t;out.capsuleSampleCount=2;
        out.inward.values.set([0,1,0]);out.closestPoint.values.set([p[0],0,p[2]]);out.source='analytic-plane';out.faceIndex=0;return out;}};
    const edgeTools=[0,1].map(edge=>({id:'wire',edge,edgeId:`wire:${edge}`,coordinates:coordinates.slice(edge,edge+2),positions:structuredClone(positions.slice(edge,edge+2)),previousPositions:structuredClone(positions.slice(edge,edge+2)),
        reference:structuredClone(reference[edge]),previousAngle:0,angle:0,materialMap:{sStart:10+2*edge,dsDx:1,dsDt:0},materialPath:{kind:'linear-affine-maps',previousEdgeId:`wire:${edge}`,previousMap:{sStart:10+2*edge,dsDx:1}}}));
    edgeTools.forEach((t,e)=>{const g=gauges[e],[x,y,z]=t.reference.director;t.reference.director=[x,y*Math.cos(g)-z*Math.sin(g),y*Math.sin(g)+z*Math.cos(g)];t.previousAngle=t.angle=angles[e]=-g;});
    const input={layout,coordinates,contactOwners:owners,sites:[{owner:'wire',node:0,edge:0,trace:'right'},{owner:'wire',node:1,edge:right?1:0,trace:right?'right':'left'},{owner:'wire',node:2,edge:1,trace:'left'}],
        edgeTools,wall:{motion:'stationary-material',source:'analytic-plane',tangentBasis:'projected-own-tangent'},dt,mu:[.1,.1],normalScale:1,frictionScale:50,tolerances:tol};
    const envelope=createCompositeWallEnvelopeWorkspace(layout),plan=create(input),toolPositions=new Map([['wire',positions]]),toolAngles=new Map([['wire',angles]]);
    function refresh(order='full'){
        refreshCompositeWallEnvelope({positions,contactOwners:owners,field},envelope);
        return plan.refresh({envelope,positionsByTool:toolPositions,anglesByTool:toolAngles,field,plane:{normal:[0,1,0],offset:0},order});
    }
    return {input,layout,coordinates,positions,angles,reference,dt,owners,field,envelope,plan,refresh,toolPositions,toolAngles};
}
function physicalResidualJacobian(f){
    const n=f.layout.dofCount,m=f.plan.rows.length,A=Array.from({length:n+m},()=>new Float64Array(n+m));
    f.plan.rows.forEach((row,r)=>{
        const ds=[...row.commonDofs];ds.forEach((d,i)=>{A[n+r][d]=row.jacobian[i];A[d][n+r]=row.forceColumn[i];ds.forEach((e,j)=>A[d][e]+=row.geometricTangent[i*ds.length+j]);});
        A[n+r][n+r]=row.multiplierDerivative;row.multiplierDofs.forEach((j,k)=>A[n+r][n+j]=row.multiplierJacobian[k]);
    });return A;
}
function perturb(f,col,delta){const n=f.layout.dofCount;if(col>=n)f.plan.forces[col-n]+=delta;else{
    let changed=false;for(let node=0;node<3;node++)for(let k=0;k<3;k++)if(f.layout.positions[node]+k===col){f.positions[node][k]+=delta;changed=true;}
    if(!changed)for(let e=0;e<2;e++)if(f.layout.spins.get('wire')[e]===col)f.angles[e]+=delta;
}}
const residual=f=>[...f.plan.commonResidual,...f.plan.rows.map(r=>r.residual)];

test('initial pressure sites have stable node/one-sided identities while original capsule t0→t1 and loaded Fn/Ft never migrate',()=>{
    const f=fixture();f.plan.forces.set([1,.01,.02,2,.02,-.01,3,-.02,.01]);const before=Array.from(f.plan.forces),keys=f.plan.siteKeys;
    f.positions[0][1]-=1e-7;f.refresh();assert.equal(f.envelope.rows[2].t,0);const first=f.plan.rows.map(r=>Array.from(r.commonDofs));
    f.positions[0][1]+=2e-7;f.refresh();assert.equal(f.envelope.rows[2].t,1);assert.deepEqual(Array.from(f.plan.forces),before);assert.equal(f.plan.siteKeys,keys);assert.deepEqual(f.plan.rows.map(r=>Array.from(r.commonDofs)),first);
    assert.equal(f.plan.normalForceSlots,3);assert.equal(f.plan.capsulePressureSlots,0);assert.equal(f.plan.proof.originalInequalities.length,6);
    assert.equal(f.plan.queries,0);assert.equal(f.plan.proof.converged,false); // initial arbitrary tractions are not a certificate
});

test('all fresh normal/tangential mechanics and constitutive derivatives agree with FD, including local dual blocks',()=>{
    const f=fixture();f.positions[0][0]-=.001;f.positions[1][2]+=.002;f.angles.set([.01,-.007]);f.plan.forces.set([1,-.11,.02,2,.02,-.23,3,.34,-.02]);f.refresh();
    const A=physicalResidualJacobian(f),n=f.layout.dofCount,N=A.length;
    for(let j=0;j<N;j++){
        const h=1e-6;perturb(f,j,h);f.refresh();const a=residual(f);perturb(f,j,-2*h);f.refresh();const b=residual(f);perturb(f,j,h);
        for(let i=0;i<N;i++)close((a[i]-b[i])/(2*h),A[i][j],2e-7);
    }
});

function runNonlinear(){
    const f=fixture(),n=f.layout.dofCount,old=structuredClone(f.positions),oldv=[[.02,0,0],[.02,0,0]],mass=.13;
    const material=compileCompositeMaterial({EI1:2,EI2:3,GJ:1}),data={positions:f.positions,coordinates:f.coordinates,reference:f.reference,tools:[{id:'wire',angles:f.angles,dsDx:1,material}]};
    const chain=createCompositeChainWorkspace(f.layout,{elementBackend:'wasm-exact'}),cluster=assembleCompositeRelativeCluster({data,layout:f.layout,modes:[],elementBackend:'wasm-exact'}),lengths=createCompositeToolLengthWorkspace({layout:f.layout}),lambda=new Float64Array(2),rest=new Map([['wire',new Float64Array([2,2])]]);
    // Freeze symbolic rows once. The source pressure slots are the FIRST local duals.
    f.refresh();evaluateCompositeToolLengths({toolPositions:f.toolPositions,restLengths:rest,multipliers:lambda,tolerance:1e-8},lengths);
    const rows=[...f.plan.rows,...lengths.rows],workspace=createCompositeRelativeDirectionWorkspace(f.layout,cluster,rows),fixed=new Uint8Array(n);for(const d of f.layout.spins.get('wire'))fixed[d]=1;
    const load=new Float64Array(n);for(let i=0;i<3;i++){load[f.layout.positions[i]]=.1;load[f.layout.positions[i]+1]=-.4;}
    f.positions[0][1]-=1e-7;f.positions[1][1]+=1e-7; // private initial Newton guess, physical old state remains flat
    f.angles.set([.03*f.dt,-.02*f.dt]); // distinct explicit own spin boundary targets
    const trace=[];let success=false,finalResidual,iterations=0;
    for(;iterations<20;iterations++){
        assembleCompositeChain(data,chain);
        for(let e=0;e<2;e++){
            const local=assembleCompositeTranslationalInertia({coordinates:f.coordinates.slice(e,e+2),positions:f.positions.slice(e,e+2),previousPositions:old.slice(e,e+2),dt:f.dt,
                tools:[{id:'wire',massPerMaterialLength:mass,materialMap:{sStart:10+2*e,dsDx:1,dsDt:0},oldMaterialVelocities:oldv}]}),dofs=[e,e+1].flatMap(node=>[0,1,2].map(k=>f.layout.positions[node]+k));
            dofs.forEach((d,i)=>{chain.gradient[d]+=local.gradient[i];for(let j=0;j<=i;j++)chain.hessian[Math.max(d,dofs[j])*f.layout.band+Math.abs(d-dofs[j])]+=local.hessian[6*i+j];});
        }
        f.refresh();evaluateCompositeToolLengths({toolPositions:f.toolPositions,restLengths:rest,multipliers:lambda,tolerance:1e-8},lengths);
        const R=Float64Array.from(chain.gradient,(v,i)=>v+f.plan.commonResidual[i]+lengths.commonGradient[i]-load[i]);finalResidual=R;
        const force=Math.max(...Array.from(R,(v,i)=>fixed[i]?0:Math.abs(v))),rowError=Math.max(...rows.map(r=>Math.abs(r.residual)));
        trace.push({iteration:iterations,capsuleT:[f.envelope.rows[2].t,f.envelope.rows[5].t],force,rowError,Fn:[0,3,6].map(i=>f.plan.forces[i]),gap:f.plan.proof.minGap});
        if(force<=1e-7&&f.plan.proof.converged&&lengths.converged){success=true;break;}
        const direction=solveCompositeRelativeDirection(workspace,chain,{cluster,commonResidual:R,relativeResidual:new Float64Array(),fixed,rows,tolerances:{force:1e-7,torque:1e-8}});
        assert.ok(direction.converged,JSON.stringify(direction));
        for(let node=0;node<3;node++)for(let k=0;k<3;k++)f.positions[node][k]+=direction.commonIncrement[f.layout.positions[node]+k];
        direction.multiplierIncrement.forEach((v,i)=>{if(i<f.plan.forces.length)f.plan.forces[i]+=v;else lambda[i-f.plan.forces.length]+=v;});
    }
    assert.ok(success,JSON.stringify(trace));return {f,trace,iterations,finalResidual,load,old,mass,oldv,workspace};
}

test('one unshifted shared band solve converges the flat inextensible rod with endpoint normal+Coulomb loads through changing capsule winners',context=>{
    const {f,trace,iterations,finalResidual,old,oldv,mass,load,workspace}=runNonlinear();assert.ok(iterations<10);assert.ok(trace.some(x=>x.capsuleT[0]===0)&&trace.some(x=>x.capsuleT[0]===1),JSON.stringify(trace));
    assert.ok(f.plan.proof.converged);assert.ok(f.plan.proof.sites.every(s=>s.Fn>0&&s.Ft[0]<0&&s.kkt.work<0));for(const p of f.positions)close(p[1],.8,1e-8);
    // Independent total material momentum for constant density, affine v and dsDt=0.
    const momentum=[0,0,0];for(let e=0;e<2;e++)for(let k=0;k<3;k++)momentum[k]+=mass*2*((f.positions[e][k]-old[e][k]+f.positions[e+1][k]-old[e+1][k])/(2*f.dt)-(oldv[0][k]+oldv[1][k])/2)/f.dt;
    const applied=[0,0,0],contact=[0,0,0];for(let node=0;node<3;node++)for(let k=0;k<3;k++){applied[k]+=load[f.layout.positions[node]+k];contact[k]-=f.plan.commonResidual[f.layout.positions[node]+k];}
    same(momentum,applied.map((v,k)=>v+contact[k]),1e-7);
    for(const s of f.plan.proof.sites){
        const e=s.edge,q=f.positions.slice(e,e+2),t=q[1].map((v,k)=>v-q[0][k]),L=Math.hypot(...t),axis=t.map(v=>v/L),F=s.physicalTangentLoad;
        const connection=Array.from({length:6},(_,j)=>dot(axis,[s.omegaMap[j],s.omegaMap[7+j],s.omegaMap[14+j]])),physical=F.slice(0,6).map((v,j)=>v-F[6]*connection[j]);
        const moment=cross(q[0],physical.slice(0,3)).map((v,k)=>v+cross(q[1],physical.slice(3,6))[k]+axis[k]*F[6]),world=s.axes[0].map((v,k)=>v*s.Ft[0]+s.axes[1][k]*s.Ft[1]);same(moment,cross(s.point,world),2e-13);
        const rates=[[.2,.1,.3],[-.1,.4,.2]],spin=.05,tool={...f.input.edgeTools[e],positions:q,angle:f.angles[e],coordinate:f.coordinates[s.node],trace:s.trace,
            positionRates:rates,angleRate:spin,materialPath:{...f.input.edgeTools[e].materialPath,previousTrace:s.trace}};
        const instant=evaluateCompositeJointSurfaceMotion({tools:[tool],dt:f.dt,rateMode:'instantaneous',wall:{velocity:[0,0,0]},contact:{point:s.point,axes:s.axes}});
        close(dot(F,rates.flat().concat(spin)),dot(s.Ft,instant.slipRate),2e-13);
    }
    // Quasi-static prescribed spins have nonzero own reaction torques; their
    // reactions are never renamed as a pressure transfer.
    assert.ok([...f.layout.spins.get('wire')].some(d=>Math.abs(finalResidual[d])>1e-6));
    context.diagnostic(JSON.stringify({iterations,band:workspace.packedLayout.kl,unknowns:workspace.count,trace,Fn:f.plan.proof.sites.map(s=>s.Fn),Ft:f.plan.proof.sites.map(s=>s.Ft),momentum,applied,contact}));
});

test('duplicate shared-node pressures are rank deficient at stick, while left/right own spins produce distinct slip laws',()=>{
    const gauges=[.3,-.7],a=fixture({right:false,gauges}),b=fixture({right:true,gauges});a.plan.forces.fill(0);b.plan.forces.fill(0);a.plan.forces[3]=b.plan.forces[3]=1;a.refresh();b.refresh();
    const global=row=>{const x=new Float64Array(a.layout.dofCount);row.commonDofs.forEach((d,i)=>x[d]=row.forceColumn[i]);return x;};
    same(global(a.plan.rows[3]),global(b.plan.rows[3]),0);
    // A two-site mixed matrix has the exact nonzero dual null vector
    // dFn_left=+1,dFn_right=-1. Geometry/force rows cancel; stick dR/dFn=0.
    assert.equal(a.plan.rows[4].multiplierJacobian[0],0);assert.equal(b.plan.rows[4].multiplierJacobian[0],0);
    assert.equal(a.plan.rows[5].multiplierJacobian[0],0);assert.equal(b.plan.rows[5].multiplierJacobian[0],0);
    a.angles.set([.1-gauges[0],-.2-gauges[1]]);b.angles.set([.1-gauges[0],-.2-gauges[1]]);a.refresh();b.refresh();
    const sa=a.plan.proof.sites[1],sb=b.plan.proof.sites[1];close(sa.slip[1],-.8*.1,2e-14);close(sb.slip[1],.8*.2,2e-14);
    assert.notDeepEqual(sa.slip,sb.slip);assert.notEqual(a.plan.signature,b.plan.signature);
    assert.throws(()=>create({...b.input,history:a.plan.copyCandidateHistory()}),/migration/);
});

test('all original capsule gaps remain acceptance gates, incoming ownership/history is owned, and failed refresh revokes scratch',()=>{
    const f=fixture(),saved=structuredClone(f.input.edgeTools);f.input.edgeTools[0].reference.director.fill(999);f.refresh();assert.ok(f.plan.valid);
    const forces=f.plan.forces.slice(),keys=f.plan.siteKeys;
    // Independent interior inequality: valid endpoint pressure does not certify
    // the full segment against a curved/other selected wall witness.
    f.envelope.rows[2].gap=-.01;const proof=f.plan.refresh({envelope:f.envelope,positionsByTool:f.toolPositions,anglesByTool:f.toolAngles,field:f.field,plane:{normal:[0,1,0],offset:0}}).proof;
    assert.equal(proof.converged,false);assert.equal(proof.minGap,-.01);assert.ok(proof.originalInequalities.some(r=>r.role==='capsule'&&r.gap===-.01));
    f.envelope.rows=f.envelope.rows.filter(r=>r.role!=='capsule');assert.throws(()=>f.plan.refresh({envelope:f.envelope,positionsByTool:f.toolPositions,anglesByTool:f.toolAngles,field:f.field}),/inequalities/);
    assert.equal(f.plan.valid,false);assert.ok(f.plan.commonResidual.every(Number.isNaN));same(f.plan.forces,forces,0);assert.equal(f.plan.siteKeys,keys);
    assert.throws(()=>create({...f.input,edgeTools:saved,history:{scheme:'old-min-sample',forces:[1,.1,.2]}}),/migration/);
    assert.throws(()=>create({...f.input,edgeTools:saved,sites:[...f.input.sites,f.input.sites[1]]}),/One explicit/);
});

test('actual original capsule interior penetration remains a failure even when every pressure endpoint is strictly open',()=>{
    const f=fixture(),p=[[-.625,-.625,.125],[.875,.875,.125],[1.5,1.5,.125]],radius=2.2;
    const valley=([x,y,z])=>2+.5*(x-.125)*(y-.125)+.25*z,gradient=([x,y])=>[.5*(y-.125),.5*(x-.125),.25];
    const field={sdfOrigin:[-1,-1,-1],sdfDimensions:[3,3,3],brickSize:2,voxelSize:.5,sdfQuantization:1/1024,sdfBrickLookup:new Uint16Array(27),sdfDistances:new Uint32Array(216)};
    for(let bz=0;bz<3;bz++)for(let by=0;by<3;by++)for(let bx=0;bx<3;bx++){const brick=bx+3*(by+3*bz);field.sdfBrickLookup[brick]=brick;
        for(let z=0;z<2;z++)for(let y=0;y<2;y++)for(let x=0;x<2;x++)field.sdfDistances[brick*8+x+2*(y+2*z)]=valley([-1+.5*(2*bx+x),-1+.5*(2*by+y),-1+.5*(2*bz+z)])*1024;}
    field.queryCapsuleCoordinates=(ax,ay,az,bx,by,bz,r,out=createContactResult())=>{const a=[ax,ay,az],b=[bx,by,bz];let best=Infinity,t=0;
        for(const s of [0,.5,1]){const d=valley(a.map((v,k)=>v+s*(b[k]-v)));if(d<best){best=d;t=s;}}
        const q=a.map((v,k)=>v+t*(b[k]-v)),g=gradient(q),n=g.map(v=>v/Math.hypot(...g));out.signedDistance=best;out.signedGap=best-r;out.segmentT=t;out.capsuleSampleCount=2;out.source='sparse-sdf';out.faceIndex=-1;
        out.inward.values.set(n);out.closestPoint.values.set(q.map((v,k)=>v-best*n[k]));return out;};
    const input={...f.input,wall:{...f.input.wall,source:'sparse-sdf'},contactOwners:{edges:[{edge:0,wall:{owner:'wire',radius}},{edge:1,wall:null}]},
        sites:[{owner:'wire',node:0,edge:0,trace:'right'},{owner:'wire',node:1,edge:0,trace:'left'}],edgeTools:structuredClone(f.input.edgeTools)};
    input.edgeTools[0].positions=structuredClone(p.slice(0,2));input.edgeTools[0].previousPositions=structuredClone(p.slice(0,2));input.edgeTools[0].reference=captureCompositeReferenceFrames(p.slice(0,2))[0];
    const plan=create(input),envelope=createCompositeWallEnvelopeWorkspace(f.layout);refreshCompositeWallEnvelope({positions:p,contactOwners:input.contactOwners,field},envelope);
    const proof=plan.refresh({envelope,positionsByTool:new Map([['wire',p]]),anglesByTool:f.toolAngles,field}).proof;
    assert.ok(proof.sites.every(s=>s.gap>0&&s.converged));assert.equal(envelope.rows[2].t,.5);close(envelope.rows[2].gap,-.16875,1e-14);assert.equal(proof.converged,false);assert.equal(plan.capsulePressureSlots,0);
});

test('value/full reuse preserves residual and force history bitwise and rejects stale or unaccepted pressure history',()=>{
    const f=fixture();f.plan.forces.set([1,.01,.02,2,.02,-.01,3,-.02,.01]);f.refresh();const common=f.plan.commonResidual.slice(),residuals=f.plan.rows.map(r=>r.residual),forces=f.plan.forces.slice();
    f.refresh('value');same(f.plan.commonResidual,common,0);same(f.plan.rows.map(r=>r.residual),residuals,0);assert.ok(f.plan.rows.every(r=>r.geometricTangentValid===false&&r.geometricTangent.every(Number.isNaN)));same(f.plan.forces,forces,0);
    f.refresh();same(f.plan.commonResidual,common,0);assert.ok(f.plan.rows.every(r=>r.geometricTangentValid));
    const candidate=f.plan.copyCandidateHistory();assert.equal(candidate.accepted,false);assert.throws(()=>create({...f.input,history:candidate}),/caller-accepted/);
    const accepted={...candidate,accepted:true},restored=create({...f.input,history:accepted});same(restored.forces,forces,0);accepted.forces.fill(0);same(restored.forces,forces,0);
});
