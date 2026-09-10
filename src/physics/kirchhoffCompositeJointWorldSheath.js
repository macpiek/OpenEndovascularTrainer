import {createCompositeContactPullbackFactory,pullbackCompositeContact} from './kirchhoffCompositeContactPullback.js';

const preparations=new WeakMap(),dot=(a,b)=>a.reduce((s,v,k)=>s+v*b[k],0);
const fail=message=>{const e=new RangeError(message);e.code='joint-world-sheath-adapter-required';throw e;};
const finite=(v,name)=>{if(!Number.isFinite(v))fail(`${name} must be finite`);return v;};
const nonnegative=(v,name)=>{if(!(finite(v,name)>=0))fail(`${name} must be nonnegative`);return v;};
const positive=(v,name)=>{if(!(finite(v,name)>0))fail(`${name} must be positive`);return v;};
const vector=(v,name)=>{if(v?.length!==3)fail(`${name} requires three components`);return Array.from(v,x=>finite(x,name));};
const signature=value=>JSON.stringify(value);

function sourceSnapshot(world,state,bindings) {
    if(!Array.isArray(world.sheaths)||bindings.length!==world.bodies.length||bindings.length!==state.tools.length)
        fail('Sheath source requires complete actual World bindings');
    const bodies=new Set(),ids=new Set();
    for(const b of bindings) {
        if(!world.bodies.includes(b.body)||bodies.has(b.body)||ids.has(b.toolId)||!state.layout.spins.has(b.toolId))fail('Sheath source bindings must be distinct actual tools');
        bodies.add(b.body);ids.add(b.toolId);
    }
    return world.sheaths.flatMap((s,sheathIndex)=>{
        if(s.enabled===false)return [];
        const start=vector([s.startX,s.startY,s.startZ],'Sheath start'),axis=vector([s.axisX,s.axisY,s.axisZ],'Sheath axis');
        if(Math.abs(dot(axis,axis)-1)>64*Number.EPSILON)fail('The actual sheath axis must be unit length');
        const length=positive(s.length,'Sheath length'),innerRadius=nonnegative(s.innerRadius,'Sheath inner radius'),
            proximalExtension=nonnegative(s.proximalExtension??0,'Sheath proximal extension');
        if(s.bodies!==null&&s.bodies!==undefined&&(!Array.isArray(s.bodies)||s.bodies.some(b=>!world.bodies.includes(b))))fail('Sheath body owners must belong to World');
        const tools=bindings.filter(b=>!s.bodies||s.bodies.includes(b.body)).map(b=>{
            const body=b.body,start=body.activeStart,end=body.activeEnd,materialEnd=body.sheathMaterialEndNode??Infinity;
            if(!Number.isInteger(start)||!Number.isInteger(end)||start<0||end<start||end>=body.count||body.nodeRadius?.length!==body.count||
                !(materialEnd===Infinity||Number.isFinite(materialEnd)))fail('Sheath body range, material end and actual radii are required');
            const nodes=b.nodes.slice().sort((a,c)=>a.node-c.node);
            if(nodes.length!==end-start+1||nodes.some((r,i)=>r.node!==start+i||!Number.isInteger(r.jointNode)||r.jointNode<0||r.jointNode>=state.layout.nodeCount||
                ![r.jointNode-1,r.jointNode].some(e=>state.layout.edgeToolIds[e]?.includes(b.toolId))))fail('Every source sheath node needs its actual own Joint node');
            return {toolId:b.toolId,bodyId:body.id??null,count:body.count,start,end,materialEnd:materialEnd===Infinity?'unbounded':materialEnd,
                nodes:nodes.filter(r=>r.node<=Math.min(end,materialEnd)).map(r=>({node:r.node,jointNode:r.jointNode,radius:nonnegative(body.nodeRadius[r.node],'Actual node radius')}))};
        });
        return [{sheathIndex,id:s.id??null,start,axis,length,innerRadius,proximalExtension,tools}];
    });
}

/** The original World sheath is an open, frictionless, NODE pressure law.
 * Own material range and original node radii are retained. No segment or
 * curved-element envelope is claimed: continuous geometry has identical node
 * positions, so this pressure law can accompany either material geometry.
 * The source's 1e-5 axial slab margin is preserved, with no axial force.
 */
export function prepareCompositeJointWorldSheath({world,state,bindings,forcePerLength=1}={}) {
    positive(forcePerLength,'Sheath numerical forcePerLength');
    const snapshots=sourceSnapshot(world,state,bindings),sites=snapshots.flatMap(s=>s.tools.flatMap(t=>t.nodes.map(n=>({
        id:signature(['world-sheath',s.sheathIndex,t.toolId,t.bodyId,n.node,n.jointNode]),toolId:t.toolId,node:n.jointNode,
        start:s.start.slice(),axis:s.axis.slice(),length:s.length,proximalExtension:s.proximalExtension,
        clearance:Math.max(0,s.innerRadius-n.radius),sourceRadius:n.radius,sourceInnerRadius:s.innerRadius
    })))),sheath={mode:'world-sheath-normal',forcePerLength,sites},proof=Object.freeze({scope:'actual-world-open-nodal-sheath',sites:sites.length});
    preparations.set(proof,{world,state,bindings,snapshots:signature(snapshots),numeric:signature(sheath)});
    return {sheath,proof};
}

export function assertCompositeJointWorldSheath({proof,world,state,bindings,sheath}={}) {
    const p=preparations.get(proof);
    if(!p||p.world!==world||p.state!==state||p.bindings!==bindings||p.snapshots!==signature(sourceSnapshot(world,state,bindings))||p.numeric!==signature(sheath))
        fail('Actual sheath ownership, geometry, radii or prepared pressure law changed');
    return true;
}

function transverse(axis) {
    const k=axis.map(Math.abs).indexOf(Math.min(...axis.map(Math.abs))),seed=[0,0,0];seed[k]=1;
    const a=seed.map((v,j)=>v-axis[k]*axis[j]),norm=Math.hypot(...a),u=a.map(v=>v/norm),
        v=[axis[1]*u[2]-axis[2]*u[1],axis[2]*u[0]-axis[0]*u[2],axis[0]*u[1]-axis[1]*u[0]];
    return [u,v];
}

/** Physical gap g=c-|P(x-start)| and inward load B=-P(x-start)/r.
 * Derivatives include the exact stored-axis projector twice: no assumed
 * axis roundoff identity, cached normal, axial target or post-step projection.
 */
export function evaluateCompositeJointSheathGeometry(site,position,{order='full'}={}) {
    if(!['full','gradient'].includes(order))fail('Sheath differential order must be full or gradient');
    const axis=vector(site.axis,'Sheath axis'),start=vector(site.start,'Sheath start'),p=vector(position,'Sheath position'),
        dx=p.map((v,k)=>v-start[k]),axial=dot(dx,axis),clearance=nonnegative(site.clearance,'Sheath clearance');
    const applicable=axial>=-nonnegative(site.proximalExtension,'Sheath proximal extension')-1e-5&&axial<=positive(site.length,'Sheath length')+1e-5;
    if(!applicable)return {applicable:false,supported:false,gap:null,axial};
    const radial=dx.map((v,k)=>v-axis[k]*axial),r=Math.hypot(...radial),gap=clearance-r;
    if(!(r>0))return {applicable:true,supported:false,gap,axial};
    const n=radial.map(v=>v/r),P=Array.from({length:9},(_,j)=>(Math.floor(j/3)===j%3?1:0)-axis[Math.floor(j/3)]*axis[j%3]),
        G=axis.map((_,k)=>-dot([P[k],P[3+k],P[6+k]],n)),B=n.map(v=>-v),DB=new Float64Array(9);
    // The original source reaction is exactly -n, while its stored projection
    // gives G=-P^T n. Preserve that distinction for roundoff as for any B/G.
    if(order==='full')for(let i=0;i<3;i++)for(let j=0;j<3;j++)for(let k=0;k<3;k++)
        DB[3*i+j]-=((i===k?1:0)-n[i]*n[k])*P[3*k+j]/r;
    return {applicable:true,supported:true,gap,axial,gapJacobian:Float64Array.from(G),normalForceColumn:Float64Array.from(B),normalDerivative:DB,hessianValid:order==='full'};
}

/** Same-dt common/relative rows. Positive clearance uses signed private NCP
 * trials; a zero-clearance source is the exact two-normal bilateral subspace,
 * admitting radial reaction in either direction and arbitrary axial slide.
 * Empty sites are valid only when the actual source adapter proves ownership.
 */
export function createCompositeJointSheathRows({layout,modes,relativeToolId='wire',sheath,history=null,tolerances}) {
    if(sheath?.mode!=='world-sheath-normal'||!Array.isArray(sheath.sites))fail('Explicit World sheath node pressure is required');
    const k=positive(sheath.forcePerLength,'Sheath numerical forcePerLength'),factory=createCompositeContactPullbackFactory({layout,modes,relativeToolId}),seen=new Set();
    const sites=sheath.sites.map(source=>{
        if(typeof source.id!=='string'||!source.id||seen.has(source.id))fail('Sheath sites require distinct source identities');seen.add(source.id);
        const s={...source,start:vector(source.start,'Sheath start'),axis:vector(source.axis,'Sheath axis')};
        if(Math.abs(dot(s.axis,s.axis)-1)>64*Number.EPSILON)fail('Sheath axis must be unit length');
        nonnegative(s.clearance,'Sheath clearance');positive(s.length,'Sheath length');nonnegative(s.proximalExtension,'Sheath proximal extension');
        const pulled=factory([{toolId:s.toolId,node:s.node}]);return {...s,pulled,bases:s.clearance===0?transverse(s.axis):null};
    });
    const chart=signature({sheath,edges:layout.edgeToolIds,positions:Array.from(layout.positions),modes,relativeToolId}),rows=[];
    sites.forEach(s=>(s.bases??[null]).forEach(basis=>{
        const p=s.pulled,size=p.dofCount,index=rows.length;
        rows.push({toolId:'world-sheath',index,site:s,basis,anchorNode:p.anchorNode,commonDofs:p.commonDofs,relativeDofs:p.relativeDofs,
            unit:'mm',jacobian:new Float64Array(size),forceColumn:new Float64Array(size),geometricTangent:new Float64Array(size*size),
            geometricTangentValid:false,residual:NaN,tolerance:tolerances.linearConstraint,multiplierDerivative:0,active:false});
    }));
    if(history!==null&&(history.signature!==chart||history.forces?.length!==rows.length))fail('Sheath source chart changed; transfer reactions explicitly');
    const forces=history===null?new Float64Array(rows.length):Float64Array.from(history.forces,(v,i)=>rows[i].basis?finite(v,'Accepted radial force'):nonnegative(v,'Accepted normal force')),
        nodalForces=new Map([...layout.spins.keys()].map(id=>[id,Array.from({length:layout.nodeCount},()=>[0,0,0])])),
        cg=new Float64Array(layout.dofCount),rg=new Float64Array(modes.reduce((n,m)=>n+m.basis.length,0));
    let committed=false,atPositions=null,atPoints=null,atForces=null;
    function refresh({toolPositions,commonResidual,relativeResidual,order,consumeQuery=()=>{}}) {
        committed=false;atPositions=null;
        if(!['gradient','full'].includes(order)||commonResidual.length!==cg.length||relativeResidual.length!==rg.length||
            !commonResidual.every(Number.isFinite)||!relativeResidual.every(Number.isFinite))fail('Current finite sheath residual arrays are required');
        cg.fill(0);rg.fill(0);for(const p of nodalForces.values())p.forEach(v=>v.fill(0));
        let penetration=0,ncp=0,work=0,minForce=Infinity,merit=0,lineSearchMerit=0;const samples=[];
        try {
            for(const row of rows) {
                consumeQuery();const s=row.site,F=finite(forces[row.index],'Private sheath force'),position=toolPositions.get(s.toolId)?.[s.node],
                    raw=evaluateCompositeJointSheathGeometry(s,position,{order}),applicable=raw.applicable,p=s.pulled;
                let geometry=raw,eq=raw.gap;
                if(row.basis&&applicable) {
                    eq=dot(row.basis,position.map((v,k)=>v-s.start[k]));
                    geometry={supported:true,gap:eq,gapJacobian:row.basis,normalForceColumn:row.basis,normalDerivative:new Float64Array(9),hessianValid:order==='full'};
                }
                const eliminated=!applicable||!geometry.supported&&F===0&&raw.gap>0;
                if(!geometry.supported&&!eliminated)fail('Loaded sheath reaction at an undefined radial direction');
                row.applicable=applicable;row.gap=eq;
                row.active=applicable&&(!!row.basis||F-k*eq>0);row.residual=row.active?eq:F/k;row.multiplierDerivative=row.active?0:1/k;
                row.geometricTangentValid=order==='full';
                if(eliminated){row.jacobian.fill(0);row.forceColumn.fill(0);row.geometricTangent.fill(0);}
                else {
                    pullbackCompositeContact(geometry,p,{order});
                    row.jacobian.forEach((_,i)=>row.jacobian[i]=row.active?p.gapJacobian[i]:0);row.forceColumn.set(p.forceColumn);
                    if(order==='full')row.geometricTangent.forEach((_,i)=>row.geometricTangent[i]=-F*p.normalDerivative[i]);
                    row.commonDofs.forEach((d,i)=>cg[d]+=F*row.forceColumn[i]);row.relativeDofs.forEach((d,i)=>rg[d]+=F*row.forceColumn[row.commonDofs.length+i]);
                    geometry.normalForceColumn.forEach((v,k)=>nodalForces.get(s.toolId)[s.node][k]+=F*v);
                }
                const gapError=applicable?(row.basis?Math.abs(eq):Math.max(0,-eq)):0,negative=row.basis?0:Math.max(0,-F),residual=Math.abs(row.residual),product=applicable?Math.abs(F*eq):0;
                penetration=Math.max(penetration,gapError);ncp=Math.max(ncp,residual);work=Math.max(work,product);if(!row.basis)minForce=Math.min(minForce,F);
                const local=(gapError/tolerances.sheathGap)**2+(negative/tolerances.force)**2+(residual/tolerances.sheathNcp)**2;
                lineSearchMerit+=local;merit+=local+(product/tolerances.sheathWork)**2;
                samples.push({id:s.id,component:row.basis? s.bases.indexOf(row.basis):null,force:F,gap:applicable?eq:null,active:row.active,applicable});
            }
            if(!Number.isFinite(merit)||!cg.every(Number.isFinite)||!rg.every(Number.isFinite))fail('Nonfinite sheath certificate');
            cg.forEach((v,i)=>commonResidual[i]+=v);rg.forEach((v,i)=>relativeResidual[i]+=v);
            const converged=minForce>=0&&penetration<=tolerances.sheathGap&&ncp<=tolerances.sheathNcp&&work<=tolerances.sheathWork;
            committed=converged;atPositions=toolPositions;atPoints=sites.map(s=>Array.from(toolPositions.get(s.toolId)[s.node]));atForces=forces.slice();
            return {converged,penetration,ncp,complementarity:work,minForce,merit,lineSearchMerit,samples,sites:sites.length,rows:rows.length,
                pressureDiscretization:'actual-world-nodes',axialLaw:'free-slide'};
        } catch(error){for(const p of nodalForces.values())p.forEach(v=>v.fill(0));for(const row of rows){row.residual=NaN;row.geometricTangentValid=false;}throw error;}
    }
    function commit() {
        if(!committed||!atPositions||forces.some((v,i)=>v!==atForces[i])||sites.some((s,i)=>atPositions.get(s.toolId)[s.node].some((v,k)=>v!==atPoints[i][k])))
            fail('Sheath commit requires unchanged freshly certified forces and positions');
        return {signature:chart,forces:forces.slice()};
    }
    // Solve the piecewise-linear unilateral model, not just the branch at
    // the incoming point. An entering neighbour can make a newly active
    // site's unconstrained Newton reaction tensile. Pivot that site's dual
    // equation at the SAME geometry and solve the full coupled system again.
    // No physical position or force is changed here; refresh still evaluates
    // the original NCP and all acceptance gates on every trial.
    function pivotLinearizedActiveSet({commonIncrement,relativeIncrement,multiplierIncrement,rowOffset}) {
        const changes=[];
        for(const row of rows) {
            if(row.basis||!row.applicable)continue;
            const p=row.site.pulled,F=forces[row.index],nextForce=F+multiplierIncrement[rowOffset+row.index];
            let nextGap=row.gap;
            row.commonDofs.forEach((d,i)=>nextGap+=p.gapJacobian[i]*commonIncrement[d]);
            row.relativeDofs.forEach((d,i)=>nextGap+=p.gapJacobian[row.commonDofs.length+i]*relativeIncrement[d]);
            // Accepted unilateral reactions have an exact nonnegative sign.
            // Even a tiny tensile target must use the inactive equation
            // F_target=0; a force-tolerance deadband here can stall a fully
            // balanced step forever at an inadmissible negative reaction.
            const active=row.active?nextForce>=0:nextGap < -tolerances.sheathGap;
            if(active===row.active)continue;
            row.active=active;row.residual=active?row.gap:F/k;row.multiplierDerivative=active?0:1/k;
            row.jacobian.forEach((_,i)=>row.jacobian[i]=active?p.gapJacobian[i]:0);
            changes.push({site:row.site.id,active,predictedForce:nextForce,predictedGap:nextGap});
        }
        return changes;
    }
    return {rows,forces,nodalForces,refresh,commit,pivotLinearizedActiveSet};
}
