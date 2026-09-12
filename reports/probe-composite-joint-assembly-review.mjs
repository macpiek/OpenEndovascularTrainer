import fs from 'node:fs';
import assert from 'node:assert/strict';
import {pathToFileURL} from 'node:url';
const manifest=JSON.parse(fs.readFileSync(new URL('./composite-joint-assembly-review-source.json',import.meta.url)));
const root=process.env.OET_JOINT_REVIEW_ROOT??manifest.runtime;
const module=path=>import(pathToFileURL(`${root}/src/physics/${path}.js`));
const {createCompositeJointAssembly}=await module('kirchhoffCompositeJointAssembly');
const {createCompositeChainLayout}=await module('kirchhoffCompositeChain');
const {compileCompositeMaterial,captureCompositeReferenceFrames}=await module('kirchhoffCompositeElement');
const {createCompositeMaterialInertiaEdge}=await module('kirchhoffCompositeMaterialInertia');
const {createCompositeToolLengthWorkspace,evaluateCompositeToolLengths}=await module('kirchhoffCompositeToolLengths');
const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0);
const identity=[[1,0,0],[0,1,0],[0,0,1]],basis=[[.8,.6,0],[-.6,.8,0],[0,0,1]];
const error=(a,b)=>Math.max(0,...Array.from(a,(v,i)=>Math.abs(v-b[i])));
const assertClose=(a,b,t=1e-9)=>assert.ok(Math.abs(a-b)<=t*(1+Math.max(Math.abs(a),Math.abs(b))),`${a} != ${b}`);
function fixture(){
    const coordinates=[0,1.5,3.3,5.7],positions=[[0,.1,-.2],[1.4,.2,.1],[3.1,-.3,.4],[5,.4,.2]],n=positions.length;
    const layout=createCompositeChainLayout(Array.from({length:n-1},()=>['wire','catheter']));
    const modes=positions.map((_,node)=>({node,basis:basis.map(v=>[...v])}));
    const relative=Float64Array.from({length:3*n},(_,i)=>.13*Math.sin(i+.4));
    const previousPositions=new Map(),tools=[],angles=new Map();
    for(const id of layout.spins.keys()){
        const wire=id==='wire',previous=positions.map((p,i)=>p.map((v,k)=>v+(wire?.018*Math.sin(i+k):-.009*Math.cos(2*i+k))));
        previousPositions.set(id,previous);angles.set(id,Float64Array.from({length:n-1},(_,e)=>(wire?.17:-.11)*(e+.3)));
        tools.push({id,dsDx:wire?1.2:.8,reference:captureCompositeReferenceFrames(previous),referenceTwists:new Float64Array(n-2),
            material:compileCompositeMaterial({EI1:wire?2:7,EI2:wire?3:11,GJ:wire?1.5:5,intrinsic:wire?[.03,-.01,.06]:[-.02,.04,-.03]})});
    }
    const inertia={dt:.07,previousPositions,inertiaEdges:layout.edgeToolIds.map((ids,e)=>({tools:ids.map(id=>{
        const wire=id==='wire',dsDx=wire?1.2:.8;
        return {id,massPerMaterialLength:wire?.13:.24,materialMap:{sStart:(wire?20:70)+dsDx*coordinates[e],dsDx,
            dsDt:[coordinates[e],coordinates[e+1]].map(x=>wire?-.7+.1*x:.5-.08*x)},
            oldMaterialVelocities:[e,e+1].map(i=>wire?[.1+.05*i,-.2+.02*i,.3-.04*i]:[-.3+.07*i,.15-.03*i,-.1+.05*i])};
    })}))};
    return {coordinates,positions,layout,modes,relative,tools,angles,inertia};
}
function reconstructed(f){
    const wire=f.positions.map(p=>[...p]);
    f.modes.forEach((m,i)=>m.basis.forEach((b,j)=>b.forEach((v,k)=>wire[m.node][k]+=v*f.relative[3*i+j])));
    return new Map([['wire',wire],['catheter',f.positions.map(p=>[...p])]]);
}
function packed(out){
    const n=out.chain.gradient.length,r=out.cluster.relative.dofCount,N=n+r,H=new Float64Array(N*N);
    for(let i=0;i<n;i++)for(let j=0;j<n;j++)if(Math.abs(i-j)<out.chain.layout.band)H[N*i+j]=out.chain.hessian[Math.max(i,j)*out.chain.layout.band+Math.abs(i-j)];
    for(let i=0;i<r;i++)for(let j=0;j<r;j++)if(Math.abs(i-j)<out.cluster.relative.band)H[N*(n+i)+n+j]=out.cluster.relative.hessian[Math.max(i,j)*out.cluster.relative.band+Math.abs(i-j)];
    const c=out.cluster.coupling;
    for(let row=0;row<c.commonDofs.length;row++)for(let i=c.rowOffsets[row];i<c.rowOffsets[row+1];i++){
        const a=c.commonDofs[row],b=n+c.columns[i];H[N*a+b]=H[N*b+a]=c.values[i];
    }
    return {energy:out.energy,g:Float64Array.from([...out.chain.gradient,...out.cluster.relative.gradient]),H,N};
}

// Independent endpoint/midpoint Simpson integration: all integrands below
// are quadratic for the declared affine fields. This uses neither the
// production Gauss samples nor Kinematics/MaterialInertia to form E/g/H.
function directInertia(f){
    const positions=reconstructed(f),n=f.layout.dofCount,N=n+f.relative.length,g=new Float64Array(N),H=new Float64Array(N*N);
    const perTool=new Map(),physicalGradient=new Map();let energy=0;
    for(const id of f.layout.spins.keys()){
        perTool.set(id,{mass:0,energy:0,kineticEnergy:0,momentum:[0,0,0],oldMomentum:[0,0,0]});
        physicalGradient.set(id,Array.from({length:f.positions.length},()=>[0,0,0]));
    }
    for(let edge=0;edge<f.layout.nodeCount-1;edge++)for(const record of f.inertia.inertiaEdges[edge].tools){
        const p=positions.get(record.id),old=f.inertia.previousPositions.get(record.id),a=p[edge],b=p[edge+1];
        const L=f.coordinates[edge+1]-f.coordinates[edge],map=record.materialMap,totalMass=record.massPerMaterialLength*map.dsDx*L;
        const info=perTool.get(record.id);info.mass+=totalMass;
        const localG=new Float64Array(6),localH=new Float64Array(36);
        for(const [f1,w] of [[0,1/6],[.5,4/6],[1,1/6]]){
            const f0=1-f1,dt=f.inertia.dt,u=-(f0*map.dsDt[0]+f1*map.dsDt[1])/map.dsDx;
            const J=[f0/dt-u/L,f1/dt+u/L],weight=totalMass*w;
            for(let k=0;k<3;k++){
                const velocity=(f0*(a[k]-old[edge][k])+f1*(b[k]-old[edge+1][k]))/dt+u*(b[k]-a[k])/L;
                const oldVelocity=f0*record.oldMaterialVelocities[0][k]+f1*record.oldMaterialVelocities[1][k],delta=velocity-oldVelocity;
                const term=.5*weight*delta*delta;energy+=term;info.energy+=term;info.kineticEnergy+=.5*weight*velocity*velocity;
                info.momentum[k]+=weight*velocity;info.oldMomentum[k]+=weight*oldVelocity;
                for(let i=0;i<2;i++){
                    localG[3*i+k]+=weight*J[i]*delta;
                    for(let j=0;j<2;j++)localH[6*(3*i+k)+3*j+k]+=weight*J[i]*J[j];
                }
            }
        }
        const columns=Array.from({length:6},(_,i)=>{
            const node=edge+Math.floor(i/3),axis=i%3,list=[[f.layout.positions[node]+axis,1]];
            if(record.id==='wire')f.modes[node].basis.forEach((b,j)=>list.push([n+3*node+j,b[axis]]));
            return list;
        });
        for(let i=0;i<6;i++){
            physicalGradient.get(record.id)[edge+Math.floor(i/3)][i%3]+=localG[i];
            for(const [row,v] of columns[i]){
                g[row]+=v*localG[i];
                for(let j=0;j<6;j++)for(const [col,w] of columns[j])H[N*row+col]+=v*localH[6*i+j]*w;
            }
        }
    }
    return {energy,g,H,N,perTool,physicalGradient};
}

const f=fixture(),assembly=createCompositeJointAssembly(f),out=assembly.evaluate(f),full=packed(out);
const elastic=packed(createCompositeJointAssembly({...f,inertia:null}).evaluate(f)),inertia=directInertia(f);
const gI=full.g.map((v,i)=>v-elastic.g[i]),HI=full.H.map((v,i)=>v-elastic.H[i]);
assertClose(full.energy-elastic.energy,inertia.energy,1e-12);assert.ok(error(gI,inertia.g)<1e-12);assert.ok(error(HI,inertia.H)<1e-12);
const independent={inertiaEnergy:inertia.energy,energyError:Math.abs(full.energy-elastic.energy-inertia.energy),
    gradientError:error(gI,inertia.g),hessianError:error(HI,inertia.H),relativeNorm:Math.hypot(...f.relative),N:full.N,perTool:{}};
for(const [id,actual] of out.perTool){
    const expected=inertia.perTool.get(id),physical=inertia.physicalGradient.get(id),total=[0,0,0];
    for(const p of physical)for(let k=0;k<3;k++)total[k]+=p[k];
    for(let k=0;k<3;k++)assertClose(total[k],(expected.momentum[k]-expected.oldMomentum[k])/f.inertia.dt,1e-12);
    assertClose(actual.inertialEnergy,expected.energy,1e-12);assertClose(actual.mass,expected.mass,1e-12);
    assert.ok(error(actual.momentum,expected.momentum)<1e-12);assert.ok(error(actual.oldMomentum,expected.oldMomentum)<1e-12);
    independent.perTool[id]={mass:expected.mass,momentum:expected.momentum,oldMomentum:expected.oldMomentum,
        physicalResidualResultant:total,expectedResultant:expected.momentum.map((v,i)=>(v-expected.oldMomentum[i])/f.inertia.dt)};
}

// A change of the complete rho basis represents the same finite physical
// geometries. Test the FULL constitutive/inertial operator, including spins.
const rotated={...f,modes:f.modes.map(m=>({node:m.node,basis:identity.map(v=>[...v])})),relative:new Float64Array(f.relative.length)};
f.modes.forEach((m,i)=>m.basis.forEach((b,j)=>b.forEach((v,k)=>rotated.relative[3*i+k]+=v*f.relative[3*i+j])));
const alt=packed(createCompositeJointAssembly(rotated).evaluate(rotated)),n=f.layout.dofCount,columns=[];
for(let i=0;i<full.N;i++)columns.push(i<n?[[i,1]]:basis[(i-n)%3].map((v,k)=>[n+3*Math.floor((i-n)/3)+k,v]));
const mappedG=new Float64Array(full.N),mappedH=new Float64Array(full.N*full.N);
for(let i=0;i<full.N;i++){
    for(const [r,v] of columns[i])mappedG[i]+=v*alt.g[r];
    for(let j=0;j<full.N;j++)for(const [r,v] of columns[i])for(const [c,w] of columns[j])mappedH[full.N*i+j]+=v*alt.H[full.N*r+c]*w;
}
assertClose(full.energy,alt.energy,1e-12);assert.ok(error(full.g,mappedG)<1e-11);assert.ok(error(full.H,mappedH)<1e-10);
const basisCovariance={energyError:Math.abs(full.energy-alt.energy),gradientError:error(full.g,mappedG),hessianError:error(full.H,mappedH)};

// Closed-form straight torsion: two distinct physical straight rods remain
// straight at finite rho. Their elastic material work is counted once each.
const twist={...f,positions:f.coordinates.map(x=>[x,0,0]),relative:new Float64Array(f.relative.length),inertia:null};
const physicalWire=f.coordinates.map(x=>[.1+1.1*x,.2+.12*x,-.1-.08*x]);
twist.modes.forEach((m,i)=>m.basis.forEach((b,j)=>twist.relative[3*i+j]=dot(b,physicalWire[i].map((v,k)=>v-twist.positions[i][k]))));
twist.tools=f.tools.map(t=>({...t,reference:captureCompositeReferenceFrames(t.id==='wire'?physicalWire:twist.positions),
    referenceTwists:new Float64Array(2),material:compileCompositeMaterial({EI1:t.id==='wire'?2:7,GJ:t.id==='wire'?1.5:5})}));
let expectedTwistEnergy=0;
for(const tool of twist.tools)for(let h=1;h<3;h++){
    const length=.5*(f.coordinates[h+1]-f.coordinates[h-1])*tool.dsDx,angles=twist.angles.get(tool.id),GJ=tool.id==='wire'?1.5:5;
    expectedTwistEnergy+=.5*GJ*(angles[h]-angles[h-1])**2/length;
}
const twistOut=createCompositeJointAssembly(twist).evaluate(twist);assertClose(twistOut.energy,expectedTwistEnergy,1e-12);
const straightTorsion={expectedEnergy:expectedTwistEnergy,actualEnergy:twistOut.energy,materialHinges:twistOut.statistics.elementEvaluations};

// Concrete contract gap: the same wire gets ds/dx=1 for elasticity and rest
// length, but ds/dx=2 for inertia. No alternate material-coordinate measure,
// density conversion or prestrain is declared, and every operator accepts it.
const layout=createCompositeChainLayout([['wire','catheter'],['wire','catheter']]),coordinates=[0,1,2],previous=coordinates.map(x=>[x,0,0]);
const data={layout,coordinates,positions:previous.map(p=>[p[0],p[1],.1]),modes:[{node:1,basis:identity}],relative:new Float64Array(3),
    angles:new Map([['wire',new Float64Array([0,.2])],['catheter',new Float64Array(2)]]),
    tools:['wire','catheter'].map(id=>({id,dsDx:1,reference:captureCompositeReferenceFrames(previous),referenceTwists:new Float64Array(1),material:compileCompositeMaterial({EI1:1,GJ:1})})),
    inertia:{dt:.1,previousPositions:new Map(['wire','catheter'].map(id=>[id,previous.map(p=>[...p])])),
        inertiaEdges:[0,1].map(e=>({tools:['wire','catheter'].map(id=>({id,massPerMaterialLength:1,
            materialMap:{sStart:e*(id==='wire'?2:1),dsDx:id==='wire'?2:1,dsDt:0},oldMaterialVelocities:[[0,0,0],[0,0,0]]}))}))}};
const metricOut=createCompositeJointAssembly(data).evaluate(data),lengths=createCompositeToolLengthWorkspace({layout,modes:metricOut.cluster.modes});
evaluateCompositeToolLengths({toolPositions:metricOut.toolPositions,restLengths:new Map([['wire',[1,1]],['catheter',[1,1]]]),multipliers:new Float64Array(4),tolerance:1e-12},lengths);
assert.equal(metricOut.operatorReady,true);assert.equal(lengths.converged,true);
const metricWitness={acceptedOperator:metricOut.operatorReady,allLengthsSatisfied:lengths.converged,
    commonCoordinates:coordinates,elasticDsDx:1,inertiaDsDxWire:2,restLengthsWire:[1,1],massPerReferenceLength:1,
    wireMassReported:metricOut.perTool.get('wire').mass,wireMassForRestArcLength:2,
    wireMomentumReported:Array.from(metricOut.perTool.get('wire').momentum),wireMomentumForRestArcLength:[0,0,2],
    totalZResidual:Array.from(layout.positions).reduce((s,d)=>s+metricOut.chain.gradient[d+2],0),totalZResidualForRestArcLength:40,
    wireElasticEnergy:metricOut.perTool.get('wire').elasticEnergy,
    interpretation:'If s is physical reference arclength, one material is silently assigned incompatible rest/elastic and inertial measures. If s is an arbitrary label, the missing metric/density conversion must instead be declared.'};
assertClose(metricWitness.wireMassReported,4);assertClose(metricWitness.totalZResidual,60);assertClose(metricWitness.wireElasticEnergy,.02);

// A finite bilateral tension has physical force -lambda J, while the KKT
// residual column is +J. Decode common/rho residuals without adding rho forces
// as another world body force. A complete basis makes this check invertible.
const lengthWorkspace=createCompositeToolLengthWorkspace({layout:f.layout,modes:out.cluster.modes});
const physical=reconstructed(f),rest=new Map([...physical].map(([id,p])=>[id,Float64Array.from({length:3},(_,e)=>Math.hypot(...p[e+1].map((v,k)=>v-p[e][k])))]));
const lambda=Float64Array.from([.7,-.2,.3,-.6,.8,-.4]);
evaluateCompositeToolLengths({toolPositions:physical,restLengths:rest,multipliers:lambda,tolerance:1e-12},lengthWorkspace);
const decoded={wire:[],catheter:[]};
for(let node=0;node<4;node++){
    const gw=[0,0,0];basis.forEach((b,j)=>b.forEach((v,k)=>gw[k]+=v*lengthWorkspace.relativeGradient[3*node+j]));
    decoded.wire.push(gw.map(v=>-v));decoded.catheter.push(gw.map((v,k)=>v-lengthWorkspace.commonGradient[f.layout.positions[node]+k]));
}
let maxForceError=0,maxMoment=0;
for(const id of ['wire','catheter']){
    const expected=Array.from({length:4},()=>[0,0,0]),p=physical.get(id);
    for(let e=0;e<3;e++){
        const r=p[e+1].map((v,k)=>v-p[e][k]),L=Math.hypot(...r),force=lambda[2*e+(id==='wire'?0:1)];
        for(let k=0;k<3;k++){expected[e][k]+=force*r[k]/L;expected[e+1][k]-=force*r[k]/L;}
    }
    maxForceError=Math.max(maxForceError,error(decoded[id].flat(),expected.flat()));
    const moment=[0,0,0];for(let node=0;node<4;node++){
        const x=p[node],F=decoded[id][node];moment[0]+=x[1]*F[2]-x[2]*F[1];moment[1]+=x[2]*F[0]-x[0]*F[2];moment[2]+=x[0]*F[1]-x[1]*F[0];
    }
    maxMoment=Math.max(maxMoment,Math.hypot(...moment));
}
assert.ok(maxForceError<1e-12);assert.ok(maxMoment<1e-12);
const reactions={maxPhysicalForceDecodeError:maxForceError,maxPerToolMoment:maxMoment,positiveLambda:'tension',
    physicalLengthForce:'-lambda J',commonResidual:'wire + catheter',relativeResidual:'B^T wire'};
const result={independent,basisCovariance,straightTorsion,reactions,metricWitness};
fs.writeFileSync(new URL('./composite-joint-assembly-review-checks.json',import.meta.url),JSON.stringify(result,null,2)+'\n');
console.log(JSON.stringify(result,null,2));
