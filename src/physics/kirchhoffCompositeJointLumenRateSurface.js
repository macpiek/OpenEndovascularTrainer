import {createCompositeJointSurfaceForceMapWorkspace,evaluateCompositeJointSurfaceForceMap} from './kirchhoffCompositeJointSurfaceMotion.js';
import {createCompositeLumenSideGeometryWorkspace,differentiateCompositeLumenSideContact} from './kirchhoffCompositeLumenSideGeometry.js';

import {createCompositeExternalCapsuleGeometryWorkspace,evaluateCompositeExternalCapsuleContact} from './kirchhoffCompositeExternalCapsuleGeometry.js';

const plans=new WeakMap(),N=14,Q=11,pos=j=>j<6?j:j+1;
const dot=(a,b)=>a.reduce((s,v,k)=>s+v*b[k],0),cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]],
    add=(a,b)=>a.map((v,k)=>v+b[k]),scale=(a,s)=>a.map(v=>v*s),sub=(a,b)=>a.map((v,k)=>v-b[k]);
const fail=message=>{const e=new RangeError(message);e.code='joint-lumen-rate-surface-unsupported';throw e;};
const vec=(v,n,name)=>{if(v?.length!==n||!Array.from(v).every(Number.isFinite))fail(`${name} needs ${n} finite entries`);return Array.from(v);};

export function createCompositeJointLumenRateSurfaceWorkspace() {
    const out={increment:new Float64Array(2),forceMap:new Float64Array(2*N),slipJacobian:new Float64Array(2*N),DforceMap:new Float64Array(2*N*N),
        currentQueryJacobian:new Float64Array(Q*N),tools:[],physicalDofCount:N,point:new Float64Array(3),normal:new Float64Array(3),tangent:new Float64Array(3)};
    plans.set(out,{force:createCompositeJointSurfaceForceMapWorkspace(2),normal:createCompositeLumenSideGeometryWorkspace(),external:createCompositeExternalCapsuleGeometryWorkspace()});return out;
}

/** Original native side/fillet/rim pressure geometry supplies its physical
 * unit inner reaction normal. The tangential pair acts at ONE virtual capsule
 * sample point x_inner-radius*n, so forces, bending couples and own spins
 * share exactly one world moment. This is the declared discrete point-contact
 * extension of the original pressure law, not exact intersecting cylinders.
 * Both own endpoint pose paths are linear in time; slip is dt times their
 * endpoint material surface rate, with all moving-foot/normal/point chains.
 */
export function evaluateCompositeJointLumenRateSurface({geometry:g,input,tools,dt,order='full'},out) {
    const plan=plans.get(out);if(!plan)throw new TypeError('Use an owned native lumen rate workspace');
    out.supported=out.incrementValid=out.forceMapValid=out.slipJacobianValid=out.DforceMapValid=out.operatorReady=false;
    for(const key of ['increment','forceMap','slipJacobian','DforceMap','currentQueryJacobian'])out[key].fill(NaN);
    if(!['full','value'].includes(order)||!(dt>0)||!Number.isFinite(dt)||!g?.supported||tools?.length!==2)fail('Fresh original native pressure geometry and two own poses are required');
    const full=order==='full',kind=g.rawContact?.kind;
    if(!['side','distal-fillet','distal-rim','external-capsule'].includes(kind))fail('Original side, fillet or rim geometry is required');
    // The normal manager can omit DB at exactly Fn=0. Tangential surface B
    // still needs its normal/query derivative: differentiate that SAME owned
    // original record, without another detector query or a new witness.
    if(full&&kind==='side'&&!g.normalDerivative.every(Number.isFinite)) {
        g=differentiateCompositeLumenSideContact({input,contact:g.rawContact},plan.normal,{order:'full'});
        if(!g.supported)fail(`Original side differential unavailable: ${g.reason}`);
    }
    if(full&&kind==='external-capsule'&&!g.normalDerivative.every(Number.isFinite)) {
        g=evaluateCompositeExternalCapsuleContact({input,order:'full'},plan.external);
        if(!g.supported||g.openDistalExcluded)fail('Original external capsule differential unavailable');
    }
    const positions=g.positions.map((p,i)=>vec(p,3,`Pressure endpoint ${i}`)),fractions=[g.innerT,g.outerT],Dfractions=[new Float64Array(N),new Float64Array(N)],
        Bn=vec(g.normalForceColumn,12,'Physical normal column'),normal=[0,1,2].map(k=>Bn[k]+Bn[3+k]),Dn=new Float64Array(3*N),
        point=vec(g.innerPoint,3,'Original inner foot'),radius=input?.innerRadius;
    if(!(radius>0)||Math.abs(Math.hypot(...normal)-1)>1e-9||fractions.some(f=>!(f>=0&&f<=1)))fail('Unit physical normal, original feet and positive inner radius are required');
    if(full) {
        const DBn=vec(g.normalDerivative,144,'Physical normal derivative');
        for(let j=0;j<12;j++) {
            const column=pos(j);if(kind==='distal-rim'||kind==='external-capsule')Dfractions[0][column]=g.innerTGradient[j];
            if(kind==='side'||kind==='external-capsule')Dfractions[1][column]=g.outerTGradient[j];
            for(let k=0;k<3;k++)Dn[k*N+column]=DBn[k*12+j]+DBn[(3+k)*12+j];
        }
    }
    const prepared=tools.map((t,i)=>{
        const p=t.positions?.map(v=>vec(v,3,'Own current endpoint')),old=t.previousPositions?.map(v=>vec(v,3,'Own previous endpoint'));
        if(p?.length!==2||old?.length!==2||p.some((v,end)=>v.some((a,k)=>a!==positions[2*i+end][k])))fail('Own current endpoints must match the original pressure geometry');
        const coordinates=vec(t.coordinates,2,'Own coordinates'),L=coordinates[1]-coordinates[0],map=t.materialMap;
        if(!(L>0)||!(map?.dsDx>0)||!Number.isFinite(map.dsDx)||!Number.isFinite(map.sStart)||!Number.isFinite(t.angle)||!Number.isFinite(t.previousAngle))fail('Own affine map and unwrapped pose angles are required');
        const rates=map.dsDtEnds!==undefined?vec(map.dsDtEnds,2,'Own label rates'):typeof map.dsDt==='number'?[map.dsDt,map.dsDt]:vec(map.dsDt,2,'Own label rates');
        if(!rates.every(Number.isFinite))fail('Finite own feed rates are required');
        const f=fractions[i];return {...t,positions:p,previousPositions:old,coordinates,coordinate:coordinates[0]+L*f,trace:f===0?'right':f===1?'left':undefined,
            materialMap:{...map,dsDt:(1-f)*rates[0]+f*rates[1]},rateEnds:rates,L};
    });
    out.currentQueryJacobian.fill(full?0:NaN);
    const rawTangent=sub(positions[3],positions[2]),projected=sub(rawTangent,scale(normal,dot(rawTangent,normal))),Dtangent=new Float64Array(3*N);
    let direction=rawTangent,baseDerivative=new Float64Array(3*N),basis='projected-outer-axis';
    for(let k=0;k<3;k++){baseDerivative[k*N+7+k]=-1;baseDerivative[k*N+10+k]=1;}
    if(Math.hypot(...projected)<1e-10) {
        // At axial incidence the meridional projection vanishes. The own
        // transported material director supplies an explicit local tangent.
        const length=Math.hypot(...rawTangent),t=scale(rawTangent,1/length),ref=prepared[1].reference,a=vec(ref?.tangent,3,'Own old tangent'),d=vec(ref?.director,3,'Own old director'),
            axis=cross(a,t),first=cross(axis,d),second=cross(axis,first),den=1+dot(a,t),theta=prepared[1].angle;
        if(!(den>1e-10))fail('Antiparallel outer reference needs another local chart');
        const carried=add(add(d,first),scale(second,1/den)),perpendicular=cross(t,carried);
        direction=add(scale(carried,Math.cos(theta)),scale(perpendicular,Math.sin(theta)));baseDerivative.fill(0);basis='projected-own-material-director';
        if(full)for(let j=0;j<N;j++) {
            const sign=j>=7&&j<10?-1:j>=10&&j<13?1:0,k=j>=10?j-10:j-7,
                dtan=t.map((v,c)=>sign*((c===k?1:0)-v*(t[k]??0))/length),daxis=cross(a,dtan),dfirst=cross(daxis,d),
                dsecond=add(cross(daxis,first),cross(axis,dfirst)),dden=dot(a,dtan),
                dcarried=add(add(dfirst,scale(dsecond,1/den)),scale(second,-dden/(den*den))),dperp=add(cross(dtan,carried),cross(t,dcarried)),
                value=add(add(scale(dcarried,Math.cos(theta)),scale(dperp,Math.sin(theta))),j===13?add(scale(carried,-Math.sin(theta)),scale(perpendicular,Math.cos(theta))):[0,0,0]);
            for(let c=0;c<3;c++)baseDerivative[c*N+j]=value[c];
        }
    }
    const axial=dot(direction,normal);out.normal.set(normal);out.point.set(sub(point,scale(normal,radius)));out.tangent.set(sub(direction,scale(normal,axial)));
    if(Math.hypot(...out.tangent)<1e-10)fail('Contact tangent chart is degenerate');
    if(full)for(let j=0;j<N;j++) {
        const dn=[0,1,2].map(k=>Dn[k*N+j]),dd=[0,1,2].map(k=>baseDerivative[k*N+j]),daxial=dot(dd,normal)+dot(direction,dn),
            dpoint=[0,1,2].map(k=>(j===k?1-fractions[0]:j===3+k?fractions[0]:0)+(positions[1][k]-positions[0][k])*Dfractions[0][j]-radius*dn[k]);
        for(let i=0;i<2;i++)out.currentQueryJacobian[i*N+j]=prepared[i].L*Dfractions[i][j];
        for(let k=0;k<3;k++){Dtangent[k*N+j]=dd[k]-daxial*normal[k]-axial*dn[k];out.currentQueryJacobian[(2+k)*N+j]=dpoint[k];
            out.currentQueryJacobian[(5+k)*N+j]=dn[k];out.currentQueryJacobian[(8+k)*N+j]=Dtangent[k*N+j];}
    }
    const force=evaluateCompositeJointSurfaceForceMap({tools:prepared,order,forceGeometry:{kind:'explicit-affine-side-query',point:out.point,normal:out.normal,tangent:out.tangent}},plan.force);
    out.forceMap.set(force.forceMap);
    if(full)for(let entry=0;entry<2*N;entry++)for(let j=0;j<N;j++) {
        let value=force.configurationDerivative[entry*N+j];for(let k=0;k<Q;k++)value+=force.queryDerivative[entry*Q+k]*out.currentQueryJacobian[k*N+j];out.DforceMap[entry*N+j]=value;
    }
    const rates=new Float64Array(N),Drates=new Float64Array(N*N);
    prepared.forEach((t,i)=>{
        const start=7*i,feed=t.materialMap.dsDt/t.materialMap.dsDx,qx=sub(t.positions[1],t.positions[0]).map(v=>v/t.L);
        for(let end=0;end<2;end++)for(let k=0;k<3;k++) {
            const row=start+3*end+k;rates[row]=(t.positions[end][k]-t.previousPositions[end][k])/dt-feed*qx[k];Drates[row*N+row]=1/dt;
            Drates[row*N+start+k]+=feed/t.L;Drates[row*N+start+3+k]-=feed/t.L;
            if(full)for(let j=0;j<N;j++)Drates[row*N+j]-=qx[k]*(t.rateEnds[1]-t.rateEnds[0])/t.materialMap.dsDx*Dfractions[i][j];
        }
        rates[start+6]=(t.angle-t.previousAngle)/dt;Drates[(start+6)*N+start+6]=1/dt;
    });
    out.increment.fill(0);if(full)out.slipJacobian.fill(0);
    for(let c=0;c<2;c++)for(let i=0;i<N;i++) {
        out.increment[c]+=dt*out.forceMap[2*i+c]*rates[i];
        if(full)for(let j=0;j<N;j++)out.slipJacobian[c*N+j]+=dt*(out.DforceMap[(2*i+c)*N+j]*rates[i]+out.forceMap[2*i+c]*Drates[i*N+j]);
    }
    for(const k of full?['increment','forceMap','slipJacobian','DforceMap']:['increment','forceMap'])if(!out[k].every(Number.isFinite))fail('Nonfinite full native pair surface operator');
    out.tools=prepared.map(t=>({id:t.id,edgeId:t.edgeId,edge:t.edge}));out.physicalForce=force;
    out.identity={feature:kind,witness:'common-inner-capsule-point-along-physical-normal',basis};
    out.motion={rateMode:'backward-euler-grid',slipModel:'implicit-backward-euler-surface-rate',finiteStepSlipKnown:false,includesHingeTransport:false,rates};
    out.supported=out.incrementValid=out.forceMapValid=true;out.operatorReady=out.slipJacobianValid=out.DforceMapValid=full;return out;
}
