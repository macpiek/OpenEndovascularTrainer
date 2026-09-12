import { assembleKirchhoffDirect } from './kirchhoffDirectSolver.js';
import { rotateVectorByQuaternion, inverseRotateVectorByQuaternion, quaternionLog, conjugateQuaternion, multiplyQuaternions, normalizeQuaternion } from './discreteKirchhoffRod.js';
import { createContactResult } from './collision/vesselContactField.js';
import { certifyKirchhoffWallFrictionModes, commitKirchhoffWallFrictionHistory } from './kirchhoffWallFrictionMode.js';

const XYZ=['X','Y','Z'];
const materialKeys=['adaptationLambdaX','adaptationLambdaY','adaptationLambdaZ','bendTwistLambda1','bendTwistLambda2','bendTwistLambda3'];
const bodyBankKeys=[...materialKeys,'controlLambda','orientationControlLambda','wallLambda','wallFrictionLambda','wallProjectionX','wallProjectionY','wallProjectionZ','toolProjectionX','toolProjectionY','toolProjectionZ'];
const jointBankKeys=['_coupledBoundaries','_coupledFoldRows','_coupledOrientationRows','_coupledExternalFriction'];
const contactKeys=['normalLambda','twistLambda','innerTwistImpulse','outerTwistImpulse'];
const contactVectors=['tangentLambda','normal','tangentU','tangentV'];
const frame=(body,node)=>({x:body.orientationX[node],y:body.orientationY[node],z:body.orientationZ[node],w:body.orientationW[node]});
const clone=v=>v.slice();
const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0);
const minus=(a,b)=>a.map((v,i)=>v-b[i]);
/** Prototype-only per-joint selection; physical material data never changes. */
export function configureKirchhoffSplitBias(joint,{materialMode='physical-compliance'}={}) {
    if(!['physical-compliance','preserve-strain','coupled-compliance'].includes(materialMode))throw new RangeError('Unknown split bias material mode');
    if(joint._splitMotion&&joint._splitMotion.phase!=='complete')throw new Error('Configure bias between physical timesteps');
    joint._splitBiasMaterialMode=materialMode;
}
/** Numeric witness for the EXISTING open cylindrical sheath, including its
 * existing axial inclusion tolerance. No end cap or swept feature is added. */
export function captureKirchhoffSplitSheathGeometry(sheath,bodies) {
    return {sheath,origin:[sheath.startX,sheath.startY,sheath.startZ],axis:[sheath.axisX,sheath.axisY,sheath.axisZ],
        minimumAxial:-sheath.proximalExtension-1e-5,maximumAxial:sheath.length+1e-5,innerRadius:sheath.innerRadius,
        included:bodies.map(b=>!sheath.bodies||sheath.bodies.includes(b)),
        start:bodies.map(b=>b.activeStart),end:bodies.map(b=>Math.min(b.activeEnd,b.sheathMaterialEndNode))};
}
function sheathCoordinates(g,p) {
    const offset=minus(p,g.origin),axial=dot(offset,g.axis);
    return {axial,radius:Math.hypot(...offset.map((v,i)=>v-axial*g.axis[i]))};
}
function sheathAxialFeature(g,axial) {return axial<g.minimumAxial?'proximal-exterior':axial>g.maximumAxial?'distal-exterior':'interior';}
function unverifiedSheath(s,reason,detail) {
    if(!s.diagnostics.unverifiedHistoryKinds.includes(reason))s.diagnostics.unverifiedHistoryKinds.push(reason);
    const issues=s.diagnostics.sheathHistoryIssues??=[];
    if(!issues.some(i=>i.reason===reason&&i.sheath===detail.sheath&&i.side===detail.side&&i.node===detail.node))issues.push({reason,...detail});
}
function sameSheathGeometry(a,b) {
    return ['origin','axis','included','start','end'].every(k=>a[k].every((v,i)=>v===b[k][i]))&&
        ['minimumAxial','maximumAxial','innerRadius'].every(k=>a[k]===b[k]);
}
function noteSheathReaction(h,phase,side,node,...values) {
    const maximum=h.reactionHistory?.[phase]?.[side];
    if(!maximum||node>=maximum.length||!values.every(Number.isFinite))return false;
    maximum[node]=Math.max(maximum[node],...values.map(Math.abs));
    return Number.isFinite(maximum[node]);
}
function observeSheathReactionBanks(joint,h,side,node) {
    const s=joint._splitMotion,b=side?joint.outerBody:joint.innerBody;
    const read=state=>state?.sheaths.get(h.sheath)?.[side].lambda[node]??0;
    const currentPhase=s.phase==='bias'?'bias':'physical';
    const results=[noteSheathReaction(h,currentPhase,side,node,read(joint._coupledBoundaries)),
        noteSheathReaction(h,'physical',side,node,read(s.bank?.joints._coupledBoundaries)),
        noteSheathReaction(h,'bias',side,node,read(s.biasBank?.joints._coupledBoundaries)),
        noteSheathReaction(h,'legacy',side,node,h.sheath.lambdas?.get(b)?.[node]??0)];
    return results.every(Boolean);
}
/** With a fixed cylinder, the radial norm along the straight start/end chord
 * cannot exceed its endpoint maximum. Strictly positive endpoint clearance
 * plus an identically zero reaction history certifies a free transition in
 * this discrete model. Touching/loaded transitions and missing history do not. */
function unloadedSheathTransition(joint,h,g,side,node,before,now) {
    const b=side?joint.outerBody:joint.innerBody;
    if(!sameSheathGeometry(h,g)||b.nodeRadius[node]!==h.nodeRadii[side][node]||
        b.materialCoordinate[node]!==h.materialCoordinates[side][node]||
        !observeSheathReactionBanks(joint,h,side,node))return false;
    const clearance=Math.max(0,h.innerRadius-h.nodeRadii[side][node]);
    return clearance-before.radius>0&&clearance-now.radius>0&&
        ['physical','bias','legacy'].every(phase=>h.reactionHistory[phase][side][node]===0);
}
function recordSheathAppliedReactions(joint,result) {
    const s=joint._splitMotion;
    for(const [index,row]of (joint._coupledBoundaries?.rows??[]).entries()) {
        if(row.kind!=='sheath')continue;
        const h=s.sheathHistory.get(row.sheathWitness?.geometry.sheath);
        const delta=result.scale*result.additionalIncrement[index];
        // This hook runs before pose application and normal-bank commit, so
        // a later collector cannot erase evidence by dropping the loaded row.
        if(!h||!noteSheathReaction(h,s.phase,row.side,row.node,row.lambda,delta,Math.max(0,row.lambda+delta)))
            unverifiedSheath(s,'sheath-reaction-history-missing',{side:row.side,node:row.node});
    }
}
/** Observe all supported material nodes, including nodes whose CURRENT row
 * vanished outside the axial slab. Dropping a row is not a proof of release. */
function auditKirchhoffSplitSheathHistory(joint) {
    const s=joint._splitMotion,current=joint._coupledBoundaries?.sheathGeometry??[];
    if(current.length!==s.sheathHistory.size)unverifiedSheath(s,'sheath-membership-changed',{});
    for(const [index,g]of current.entries()) {
        const h=s.sheathHistory.get(g.sheath);
        if(!h){unverifiedSheath(s,'sheath-membership-changed',{sheath:index});continue;}
        if(['origin','axis'].some(k=>g[k].some((v,i)=>v!==h[k][i]))||
            ['minimumAxial','maximumAxial','innerRadius'].some(k=>g[k]!==h[k]))
            unverifiedSheath(s,'sheath-geometry-changed',{sheath:index});
        if(['included','start','end'].some(k=>g[k].some((v,i)=>v!==h[k][i])))
            unverifiedSheath(s,'sheath-material-support-changed',{sheath:index});
        for(const [side,b]of [joint.innerBody,joint.outerBody].entries()) {
            if(!h.included[side])continue;
            for(let node=h.start[side];node<=h.end[side];node++) {
                const historyPresent=observeSheathReactionBanks(joint,h,side,node);
                const priorProof=s.diagnostics.sheathUnloadedTransitions?.find(t=>t.sheath===index&&t.side===side&&t.node===node);
                if(priorProof&&(!historyPresent||['physical','bias','legacy'].some(phase=>h.reactionHistory[phase][side][node]!==0))) {
                    priorProof.reactionHistoryValid=false;
                    unverifiedSheath(s,historyPresent?'sheath-axial-feature-changed':'sheath-reaction-history-missing',
                        {sheath:index,side,node,detail:'earlier-free-transition-has-reaction-history'});
                }
                if(b.nodeRadius[node]!==h.nodeRadii[side][node]||b.materialCoordinate[node]!==h.materialCoordinates[side][node])
                    unverifiedSheath(s,'sheath-material-support-changed',{sheath:index,side,node});
                const before=sheathCoordinates(h,XYZ.map(a=>s.start[side][a][node]));
                const now=sheathCoordinates(h,XYZ.map(a=>b[a.toLowerCase()][node]));
                const clearance=Math.max(0,h.innerRadius-h.nodeRadii[side][node]);
                const from=sheathAxialFeature(h,before.axial),to=sheathAxialFeature(h,now.axial);
                if(from!==to) {
                    if(historyPresent&&unloadedSheathTransition(joint,h,g,side,node,before,now)) {
                        const transitions=s.diagnostics.sheathUnloadedTransitions??=[];
                        const entry=transitions.find(t=>t.sheath===index&&t.side===side&&t.node===node&&t.from===from&&t.to===to);
                        const detail={sheath:index,side,node,from,to,startAxial:before.axial,currentAxial:now.axial,
                            startRadialGap:clearance-before.radius,currentRadialGap:clearance-now.radius,reactionHistoryValid:true};
                        if(entry)Object.assign(entry,detail);else transitions.push(detail);
                    } else unverifiedSheath(s,historyPresent?'sheath-axial-feature-changed':'sheath-reaction-history-missing',
                        {sheath:index,side,node,from,to,startAxial:before.axial,currentAxial:now.axial});
                }
            }
        }
    }
}
function sample(body,start,record,side) {
    const prefix=side?'_outer':'_inner', nodes=record[prefix+'NodeIndices'];
    const count=nodes?record[prefix+'NodeCount']:2, segment=record[prefix+'SegmentIndex'];
    const weights=nodes?record[prefix+'NodeWeights']:record[side?'outerWeights':'innerWeights'];
    const out=[0,0,0];
    for(let k=0;k<count;k++) for(let axis=0;axis<3;axis++) out[axis]+=weights[k]*start[XYZ[axis]][nodes?nodes[k]:segment+k];
    return out;
}
function energy(joint,dt) {
    let elastic=0, hardMaximum=0;
    for(const body of [joint.innerBody,joint.outerBody]) {
        const s=assembleKirchhoffDirect(body,dt);
        for(let r=0;r<s.rowCount;r++) {
            const c=s.strain[r], compliance=s.alpha[r]*dt*dt;
            if(compliance>0) elastic+=.5*c*c/compliance;
            else hardMaximum=Math.max(hardMaximum,Math.abs(c));
        }
    }
    return {elastic,hardMaximum};
}
/** Owned state for ONE dt. Prediction is captured after force/damping
 * integration, before any geometric repair. Angular velocities are world. */
export function beginKirchhoffSplitMotion(joint,world) {
    const dt=world.fixedDt;
    const biasMaterialMode=joint._splitBiasMaterialMode??'physical-compliance';
    const state=joint._splitMotion={phase:'physical',dt,step:world.stepCount,bodies:[],start:[],sweeps:[],pointWalls:[new Map(),new Map()],
        physicalNormalRows:[],materialStrainOffsets:null,biasMaterialMode,diagnostics:{mode:'split-physical-bias',physicalDt:dt,biasMaterialMode,
            physicalPasses:0,biasPasses:0,historyCommits:0,rejectedTrials:0,rollbackCount:0,physicalAccepted:false,biasAccepted:false,
            sweptWitnesses:0,unverifiedHistoryKinds:[],biasElasticEnergyDelta:0,certified:false}};
    for(const body of [joint.innerBody,joint.outerBody]) {
        const motion={},start={};
        for(const axis of XYZ) {
            start[axis]=clone(body['previous'+axis]);
            motion['velocity'+axis]=Float64Array.from(body[axis.toLowerCase()],(v,i)=>body.inverseMass[i]>0?(v-start[axis][i])/dt:0);
            motion['angularVelocity'+axis]=clone(body['angularVelocity'+axis]);
        }
        for(const axis of [...XYZ,'W']) start['orientation'+axis]=clone(body['previousOrientation'+axis]);
        state.start.push(start);state.bodies.push(motion);
        body._splitPhysicalMotion=motion;
    }
    state.contactActivation=world.contactActivation;
    const bodies=[joint.innerBody,joint.outerBody];
    state.sheathHistory=new Map(world.sheaths.map(sheath=>{
        const geometry=captureKirchhoffSplitSheathGeometry(sheath,bodies);
        geometry.nodeRadii=bodies.map(b=>b.nodeRadius.slice());
        geometry.materialCoordinates=bodies.map(b=>b.materialCoordinate.slice());
        geometry.reactionHistory=Object.fromEntries(['physical','bias','legacy'].map(phase=>
            [phase,bodies.map(b=>new Float64Array(b.count))]));
        return [sheath,geometry];
    }));
    joint.surfaceMotion=state;
    return state;
}
export function prescribeKirchhoffSplitOrientation(joint,body,segment) {
    const s=joint._splitMotion;if(!s||s.phase!=='physical')return;
    const side=body===joint.innerBody?0:1,start=s.start[side];
    const previous={x:start.orientationX[segment],y:start.orientationY[segment],z:start.orientationZ[segment],w:start.orientationW[segment]};
    const target={x:body.orientationControlX,y:body.orientationControlY,z:body.orientationControlZ,w:body.orientationControlW};
    const relative=normalizeQuaternion(multiplyQuaternions(target,conjugateQuaternion(previous,{}),{}),{});
    const rotation=quaternionLog(relative,{});
    for(const axis of XYZ)s.bodies[side]['angularVelocity'+axis][segment]=rotation[axis.toLowerCase()]/s.dt;
    // A prescribed frame is physical operator motion. Both channel poses
    // share this exact boundary; otherwise the bias material strain would
    // mistake the commanded rotation for geometric error at the support.
    const physicalPose=s.twoChannel?.physicalPose[side];
    if(physicalPose)for(const axis of [...XYZ,'W'])physicalPose['orientation'+axis][segment]=target[axis.toLowerCase()];
}
export function captureKirchhoffSplitSweep(joint,body,node,contact) {
    const s=joint._splitMotion,side=body===joint.innerBody?0:1,start=s.start[side];
    const toi=contact.timeOfImpact,n=[contact.inward.x,contact.inward.y,contact.inward.z];
    const point=XYZ.map(axis=>start[axis][node]+toi*(body[axis.toLowerCase()][node]-start[axis][node]));
    s.sweeps.push({side,node,point,n,lambda:0,kind:'split-sweep',lower:0,upper:Infinity,alpha:0,
        wallFrictionWitness:{branchId:contact.branchId,faceIndex:contact.faceIndex,planeOffset:dot(n,point)},
        gradients:n.map((value,axis)=>({side,dof:node*6+axis,value})),strain:0});
    s.diagnostics.sweptWitnesses++;
}
export function appendKirchhoffSplitSweeps(joint,rows) {
    const s=joint._splitMotion;if(!s)return;
    for(const row of s.sweeps) {
        const body=row.side?joint.outerBody:joint.innerBody;
        row.strain=dot(XYZ.map(a=>body[a.toLowerCase()][row.node]-row.point[XYZ.indexOf(a)]),row.n);
        rows.push(row);
    }
}
/** Endpoint sphere witnesses supplement, never replace, capsule-interior
 * witnesses. They keep a planar initial overlap from changing which endpoint
 * owns an old capsule multiplier as successive feet become the deepest. */
export function appendKirchhoffSplitPointWalls(joint,world,rows) {
    const s=joint._splitMotion;if(!s||!world.contactField)return;
    const result=s.wallQuery??=createContactResult(),point=s.wallPoint??={x:0,y:0,z:0};
    for(const [side,body]of [joint.innerBody,joint.outerBody].entries()) {
        const start=Math.max(body.activeStart,body.collisionStartSegment);
        const lastSegment=Math.min(body.activeEnd-1,body.collisionEndSegment);
        // Endpoints supplement active vessel capsules. An empty segment
        // interval has no endpoints; it commonly denotes sheath-owned material.
        if(lastSegment<start)continue;
        const end=lastSegment+1;
        for(let node=start;node<=end;node++) {
            point.x=body.x[node];point.y=body.y[node];point.z=body.z[node];
            world.contactField.querySphere(point,body.nodeRadius[node],result);
            let row=s.pointWalls[side].get(node);
            if(result.signedGap>world.contactActivation&&!(row?.lambda>0))continue;
            if(!row){row={kind:'split-point-wall',side,node,owner:body,lower:0,upper:Infinity,lambda:0,gradients:[],normal:[0,0,0]};s.pointWalls[side].set(node,row);}
            row.strain=result.signedGap;row.alpha=body.wallCompliance/(s.dt*s.dt);
            row.normal[0]=result.inward.x;row.normal[1]=result.inward.y;row.normal[2]=result.inward.z;
            // Contact identity owns the collider plane, not the moving rod
            // surface point. Copy query values before the result is reused.
            row.wallFrictionWitness={branchId:result.branchId,faceIndex:result.faceIndex,
                planeOffset:dot(row.normal,XYZ.map(axis=>result.closestPoint[axis.toLowerCase()]))};
            row.gradients=row.normal.map((value,axis)=>({side,dof:node*6+axis,value}));
            rows.push(row);
        }
    }
}
/** Pull a row into displacement-equivalent PHYSICAL velocity coordinates. */
export function kirchhoffSplitRowMotion(joint,gradients) {
    const state=joint._splitMotion;let value=0;
    for(const g of gradients) {
        const b=g.side?joint.outerBody:joint.innerBody,m=state.bodies[g.side],node=Math.floor(g.dof/6),axis=g.dof%6;
        if(axis<3)value+=g.value*m['velocity'+XYZ[axis]][node];
        else {
            const w=inverseRotateVectorByQuaternion(frame(b,node),{x:m.angularVelocityX[node],y:m.angularVelocityY[node],z:m.angularVelocityZ[node]});
            value+=g.value*w[XYZ[axis-3].toLowerCase()];
        }
    }
    return state.dt*value;
}
function lumenGradients(record) {
    if(record.normalGradients)return record.normalGradients;
    const out=[];
    for(let side=0;side<2;side++) {
        const prefix=side?'_outer':'_inner',nodes=record[prefix+'NodeIndices'];
        const count=nodes?record[prefix+'NodeCount']:2,weights=nodes?record[prefix+'NodeWeights']:record[side?'outerWeights':'innerWeights'];
        for(let k=0;k<count;k++)for(let axis=0;axis<3;axis++)out.push({side,dof:(nodes?nodes[k]:record[prefix+'SegmentIndex']+k)*6+axis,value:(side?1:-1)*weights[k]*record.normal[axis]});
    }
    return out;
}
function lumenStartGap(joint,record) {
    const s=joint._splitMotion;
    const pi=sample(joint.innerBody,s.start[0],record,0),po=sample(joint.outerBody,s.start[1],record,1),offset=minus(pi,po);
    if(record.kind==='sliding-rim')return record.clearance-Math.hypot(...offset);
    const seg=record._outerSegmentIndex,start=s.start[1];
    const axis=XYZ.map(a=>start[a][seg+1]-start[a][seg]),h=Math.hypot(...axis);
    if(!(h>0))throw new RangeError('Split motion requires a nondegenerate historical witness');
    for(let k=0;k<3;k++)axis[k]/=h;
    const axial=dot(offset,axis),rho=Math.hypot(...offset.map((v,k)=>v-axial*axis[k]));
    if(record.kind==='distal-fillet') {
        const f=joint.portalFilletRadius,clearance=Math.max(0,joint.innerRadius-Math.max(joint.innerBody.nodeRadius[record._innerSegmentIndex],joint.innerBody.nodeRadius[record._innerSegmentIndex+1]));
        return Math.hypot(axial+f,rho-(clearance+f))-f;
    }
    if(!['side','material-side','distal-rim'].includes(record.kind)&&!s.diagnostics.unverifiedHistoryKinds.includes(record.kind))s.diagnostics.unverifiedHistoryKinds.push(record.kind);
    return record.clearance-rho;
}
/** Actual geometry remains available to witnesses/diagnostics. The physical
 * row sees initial positive clearance plus physical normal motion; the bias
 * row sees the unmodified geometric penetration. Histories are evaluated from
 * immutable time-start positions on the existing material witness. */
export function prepareKirchhoffSplitLumenRows(joint) {
    const s=joint._splitMotion;if(!s)return;
    for(const r of joint.kirchhoffContacts) {
        r._splitActualGap=r.gap;
        if(s.phase==='bias')continue;
        const gradients=lumenGradients(r),startGap=lumenStartGap(joint,r);
        r.gap=Math.max(0,startGap)+kirchhoffSplitRowMotion(joint,gradients);
        r._splitStartGap=startGap;r._splitMotionGradients=gradients;
    }
}
export function prepareKirchhoffSplitBoundaryRows(joint,rows) {
    const s=joint._splitMotion;if(!s)return;
    auditKirchhoffSplitSheathHistory(joint);
    for(const row of rows) {
        row._splitActualStrain=row.strain;
        if(s.phase==='bias') {
            if(row.kind==='control') {
                const ref=s.controlStrain?.[row.side]?.[row.node*3+row.component];
                if(ref!==undefined)row.strain-=ref;
            }
            continue;
        }
        if(!['wall','tool','sheath','split-sweep','split-point-wall'].includes(row.kind))continue;
        let startGap=row.strain;
        if(row.kind==='sheath') {
            const witness=row.sheathWitness,h=s.sheathHistory.get(witness?.geometry.sheath);
            if(h) {
                const p=sheathCoordinates(h,XYZ.map(a=>s.start[row.side][a][row.node]));
                startGap=Math.max(0,h.innerRadius-h.nodeRadii[row.side][row.node])-p.radius;
                row._splitSheathStartAxial=p.axial;
                const now={axial:witness.axial,radius:witness.radius};
                if(sheathAxialFeature(h,p.axial)!=='interior'&&
                    !unloadedSheathTransition(joint,h,witness.geometry,row.side,row.node,p,now))
                    unverifiedSheath(s,'sheath-axial-feature-changed',{sheath:witness.geometry.sheath.id,side:row.side,node:row.node});
                if(witness.radius<=1e-12)
                    unverifiedSheath(s,'sheath-degenerate-radial-witness',{sheath:witness.geometry.sheath.id,side:row.side,node:row.node});
            } else unverifiedSheath(s,'sheath-missing-historical-witness',{side:row.side,node:row.node});
        } else if(row.kind==='tool') {
            const ia=row.bodyA===joint.innerBody?0:1,ib=1-ia;
            const a=XYZ.map(k=>(1-row.tA)*s.start[ia][k][row.segmentA]+row.tA*s.start[ia][k][row.segmentA+1]);
            const b=XYZ.map(k=>(1-row.tB)*s.start[ib][k][row.segmentB]+row.tB*s.start[ib][k][row.segmentB+1]);
            startGap=Math.hypot(...minus(a,b))-(row.distance-row.strain);
        } else {
            for(const g of row.gradients) {
                const axis=g.dof%6,node=Math.floor(g.dof/6),body=g.side?joint.outerBody:joint.innerBody;
                if(axis<3)startGap+=g.value*(s.start[g.side][XYZ[axis]][node]-body[XYZ[axis].toLowerCase()][node]);
            }
        }
        row._splitStartGap=startGap;
        row.strain=Math.max(0,startGap)+kirchhoffSplitRowMotion(joint,row.gradients);
    }
}
/** Call BEFORE the pose apply: local/right angular impulse uses that frame.
 * All channels are owned by the joint and therefore roll back with its trial. */
export function applyKirchhoffSplitPhysicalIncrement(joint,result) {
    const s=joint._splitMotion;if(!s)return;
    recordSheathAppliedReactions(joint,result);
    if(s.phase!=='physical')return;
    for(const [side,body]of [joint.innerBody,joint.outerBody].entries()) {
        const m=s.bodies[side],c=(side?result.outer:result.inner).correction,f=result.scale/s.dt;
        for(let node=body.activeStart;node<=body.activeEnd;node++) {
            for(let axis=0;axis<3;axis++)m['velocity'+XYZ[axis]][node]+=f*c[node*6+axis];
            if(node===body.activeEnd)continue;
            const w=rotateVectorByQuaternion(frame(body,node),{x:c[node*6+3],y:c[node*6+4],z:c[node*6+5]});
            for(const axis of XYZ)m['angularVelocity'+axis][node]+=f*w[axis.toLowerCase()];
        }
    }
}
export function beginKirchhoffSplitBias(joint,world,accepted) {
    const s=joint._splitMotion;s.diagnostics.physicalAccepted=accepted;
    s.physicalMaterialResidual={...measureKirchhoffSplitMaterial(joint,s.dt,{})};
    s.physicalEnergy=energy(joint,s.dt);
    s.materialStrainOffsets=[joint.innerBody,joint.outerBody].map(b=>assembleKirchhoffDirect(b,s.dt).strain.slice());
    s.controlStrain=[joint.innerBody,joint.outerBody].map(b=>Float64Array.from({length:b.count*3},(_,i)=>{
        const node=Math.floor(i/3),axis=XYZ[i%3];return b[axis.toLowerCase()][node]-b['control'+axis][node];
    }));
    s.bank={bodies:[],joints:{},contacts:[],tools:[],sweeps:s.sweeps,pointWalls:s.pointWalls};
    for(const b of [joint.innerBody,joint.outerBody]) {
        const bank={};for(const key of bodyBankKeys){bank[key]=b[key];b[key]=new b[key].constructor(b[key].length);}
        bank.orientationTarget=[...XYZ,'W'].map(axis=>b['orientationControl'+axis]);
        const controlled=b.orientationControlSegment;
        if(b.orientationControlCompliance>0&&controlled>=b.activeStart&&controlled<b.activeEnd)
            for(const axis of [...XYZ,'W'])b['orientationControl'+axis]=b['orientation'+axis][controlled];
        s.bank.bodies.push(bank);
    }
    for(const key of jointBankKeys){s.bank.joints[key]=joint[key];delete joint[key];}
    for(const c of joint.manifold.contacts()) {
        const bank={contact:c,id:c.id,innerMaterialSegmentId:c.innerMaterialSegmentId,
            outerMaterialSegmentId:c.outerMaterialSegmentId,feature:c.feature};
        for(const key of contactKeys){bank[key]=c[key];c[key]=0;}
        for(const key of contactVectors)bank[key]=c[key].slice();c.tangentLambda.fill(0);s.bank.contacts.push(bank);
    }
    for(const tool of world.toolContacts) {
        s.bank.tools.push({tool,lambdas:tool.lambdas,reactions:tool._jointReactions});
        tool.lambdas=new tool.lambdas.constructor(tool.lambdas.length);tool._jointReactions=new Map();
    }
    s.sweeps=s.sweeps.map(r=>({...r,lambda:0}));s.pointWalls=[new Map(),new Map()];s.phase='bias';
}
export function finishKirchhoffSplitBias(joint,accepted) {
    const s=joint._splitMotion;s.diagnostics.biasAccepted=accepted;
    const after=energy(joint,s.dt);s.diagnostics.biasElasticEnergyDelta=after.elastic-s.physicalEnergy.elastic;
    s.diagnostics.biasHardStrainMaximum=after.hardMaximum;
    s.biasBank={bodies:[],joints:{},contacts:[],tools:[],pointWalls:s.pointWalls};
    for(const [side,b]of [joint.innerBody,joint.outerBody].entries()) {
        const bank={};for(const key of bodyBankKeys){bank[key]=b[key];b[key]=s.bank.bodies[side][key];}s.biasBank.bodies.push(bank);
        [...XYZ,'W'].forEach((axis,i)=>{b['orientationControl'+axis]=s.bank.bodies[side].orientationTarget[i];});
    }
    for(const key of jointBankKeys){s.biasBank.joints[key]=joint[key];joint[key]=s.bank.joints[key];}
    for(const c of joint.manifold.contacts()) {
        const bank={contact:c};for(const key of contactKeys){bank[key]=c[key];c[key]=0;}
        for(const key of contactVectors)bank[key]=c[key].slice();c.tangentLambda.fill(0);s.biasBank.contacts.push(bank);
    }
    for(const bank of s.bank.contacts) {
        const c=bank.contact;
        if(c._manifold!==joint.manifold||c.id!==bank.id||c.innerMaterialSegmentId!==bank.innerMaterialSegmentId||
            c.outerMaterialSegmentId!==bank.outerMaterialSegmentId||c.feature!==bank.feature) {
            if(!s.diagnostics.unverifiedHistoryKinds.includes('contact-identity-changed-during-bias'))
                s.diagnostics.unverifiedHistoryKinds.push('contact-identity-changed-during-bias');
            // The phase-final reset above already cleared all live reactions.
            // A recycled object must never receive another material contact's bank.
            continue;
        }
        for(const key of contactKeys)bank.contact[key]=bank[key];
        for(const key of contactVectors)for(let i=0;i<bank[key].length;i++)bank.contact[key][i]=bank[key][i];
    }
    for(const bank of s.bank.tools) {
        s.biasBank.tools.push({tool:bank.tool,lambdas:bank.tool.lambdas,reactions:bank.tool._jointReactions});
        bank.tool.lambdas=bank.lambdas;bank.tool._jointReactions=bank.reactions;
    }
    s.sweeps=s.bank.sweeps;s.pointWalls=s.bank.pointWalls;s.phase='complete';s.materialStrainOffsets=null;
}
export function measureKirchhoffSplitMaterial(joint,dt,out={}) {
    const s=joint._splitMotion;out.adaptationMm=out.bendTwistRad=0;
    // A successful physical phase does not certify the later geometric pose.
    // Re-evaluate native material equations after bias, with physical lambda.
    // If bias changed elastic strain too much, this bounded prototype rejects
    // the step; the saved phase residual remains available for diagnostics.
    for(const [side,b]of [joint.innerBody,joint.outerBody].entries()) {
        const m=assembleKirchhoffDirect(b,dt),offset=s.phase==='bias'?s.materialStrainOffsets[side]:null;
        const preserveStrain=s.phase==='bias'&&s.biasMaterialMode==='preserve-strain';
        for(let row=0;row<m.rowCount;row+=6) {
            const residual=k=>m.strain[row+k]-(offset?.[row+k]??0)+(preserveStrain?0:m.alpha[row+k]*m.lambda[row+k]);
            out.adaptationMm=Math.max(out.adaptationMm,Math.hypot(residual(3),residual(4),residual(5)));
            out.bendTwistRad=Math.max(out.bendTwistRad,Math.hypot(residual(0),residual(1),residual(2)));
        }
    }
    return out;
}
export function copyKirchhoffSplitVelocity(body) {
    const m=body._splitPhysicalMotion;if(!m)return false;
    for(const axis of XYZ){body['velocity'+axis].set(m['velocity'+axis]);body['angularVelocity'+axis].set(m['angularVelocity'+axis]);}
    return true;
}

export function syncKirchhoffSplitVelocity(joint) {
    const s=joint._splitMotion;let maximum=0;
    for(const [side,b]of [joint.innerBody,joint.outerBody].entries())for(const prefix of ['velocity','angularVelocity'])for(const axis of XYZ) {
        const physical=s.bodies[side][prefix+axis],actual=b[prefix+axis];
        for(let i=0;i<physical.length;i++)maximum=Math.max(maximum,Math.abs(physical[i]-actual[i]));
        physical.set(actual);
    }
    s.diagnostics.finishVelocityMaximumChange=maximum;
}
/** Certify the discrete normal law actually solved over this entire dt.
 * Published physical velocity is not a terminal TOI/restitution velocity.
 * Current geometry must have been collected before this measurement; unlike
 * a cached row strain, J*v is recomputed from the current physical channel. */
export function measureKirchhoffSplitNormalCertificate(joint,world,out={}) {
    const s=joint._splitMotion;
    if(s?.phase!=='complete')throw new Error('Normal certification requires completed physical and bias phases');
    out.contactCount=out.maximumHistoryResidualMm=out.maximumRawPenetrationMm=out.maximumGeometryViolationMm=0;
    out.finite=true;
    const evaluate=(startGap,actualGap,gradients,alpha,physicalLambda,biasLambda,loadedThreshold)=>{
        const motion=kirchhoffSplitRowMotion(joint,gradients);
        if(![startGap,actualGap,motion,alpha,physicalLambda,biasLambda].every(Number.isFinite)||
            alpha<0||physicalLambda<0||biasLambda<0) {
            out.finite=false;out.maximumHistoryResidualMm=out.maximumGeometryViolationMm=Infinity;return;
        }
        const historyGap=Math.max(0,startGap)+motion;
        const residual=historyGap+alpha*physicalLambda;
        out.maximumHistoryResidualMm=Math.max(out.maximumHistoryResidualMm,
            physicalLambda>loadedThreshold?Math.abs(residual):Math.max(0,-residual));
        // Bias solved actual geometric compression with its OWN normal bank.
        // Preserve that allowance; a hard contact still requires raw g>=-tol.
        out.maximumRawPenetrationMm=Math.max(out.maximumRawPenetrationMm,-actualGap);
        out.maximumGeometryViolationMm=Math.max(out.maximumGeometryViolationMm,-actualGap-alpha*biasLambda);
        out.contactCount++;
    };
    const biasContacts=new Map(s.biasBank.contacts.map(b=>[b.contact,b]));
    for(const r of joint.kirchhoffContacts)if(r.manifoldContact) {
        evaluate(r._splitStartGap,r._splitActualGap,lumenGradients(r),r._normalAlpha??0,
            r.manifoldContact.normalLambda,biasContacts.get(r.manifoldContact)?.normalLambda??0,1e-8);
    }
    for(const r of joint._coupledBoundaries?.rows??[]) {
        let bias=0;
        if(r.kind==='wall')bias=s.biasBank.bodies[r.side].wallLambda[r.node];
        else if(r.kind==='split-point-wall')bias=s.biasBank.pointWalls[r.side].get(r.node)?.lambda??0;
        else if(r.kind==='tool')bias=s.biasBank.tools.find(b=>b.tool===r.owner)?.lambdas[r.node]??0;
        else if(r.kind==='sheath')bias=s.biasBank.joints._coupledBoundaries?.sheaths.get(r.sheathWitness?.geometry.sheath)?.[r.side].lambda[r.node]??0;
        else if(r.kind==='split-sweep'&&s.twoChannel)bias=s.biasBank.sweeps[r.side].get(r.node)?.lambda??0;
        else if(r.kind!=='split-sweep')continue;
        evaluate(r._splitStartGap,r._splitActualStrain,r.gradients,r.alpha,r.lambda,bias,1e-10);
    }
    out.settled=out.finite&&out.maximumHistoryResidualMm<=world.coupledContainmentTolerance&&
        out.maximumGeometryViolationMm<=world.coupledContainmentTolerance;
    return out;
}
export function commitKirchhoffSplitHistory(joint,world,measurement) {
    const s=joint._splitMotion,d=s.diagnostics;
    d.rejectedTrials=d.rollbackCount=world.lastJointBacktracks;
    d.physicalKKTResidualMm=Math.max(joint.kirchhoffSolverResidual,joint._jointBoundaryResidual,
        measurement.frictionResidual.maximumDisplacementResidualMm,measurement.externalFrictionResidual.maximumDisplacementResidualMm,
        measurement.wallPhysicalFriction?.maximumDisplacementResidualMm??0);
    d.physicalConeViolation=Math.max(measurement.frictionResidual.maximumConeViolation,
        measurement.externalFrictionResidual.maximumConeViolation,measurement.wallPhysicalFriction?.maximumConeViolation??0);
    d.finalPhysicalResidualSettled=measurement.settled;
    d.physicalPhaseMaterialResidual={...s.physicalMaterialResidual};
    const material=measurement.materialResidual;
    d.finalMaterialResidual=s.twoChannel?{adaptationMm:material.adaptationMm,bendTwistRad:material.bendTwistRad,
        physicalResidual:{...material.physicalResidual},biasResidual:{...material.biasResidual},finite:material.finite}:{...material};
    if(s.twoChannel) {
        // Generalized release vectors are solver scratch. Retain the scalar
        // certificate and owned release list without copying both full rods
        // into the public diagnostics of every physical timestep.
        const {releaseCorrection,...channels}=measurement.channelResidual;
        d.finalChannelResidual=structuredClone(channels);
    }
    d.maximumOutwardContactVelocity=0;
    for(const r of joint.kirchhoffContacts)if(r.manifoldContact&&r._splitActualGap<=world.coupledContainmentTolerance)
        d.maximumOutwardContactVelocity=Math.max(d.maximumOutwardContactVelocity,-kirchhoffSplitRowMotion(joint,lumenGradients(r))/s.dt);
    for(const r of joint._coupledBoundaries?.rows??[])if(['wall','tool','sheath','split-sweep','split-point-wall'].includes(r.kind)&&r._splitActualStrain<=world.coupledContainmentTolerance)
        d.maximumOutwardContactVelocity=Math.max(d.maximumOutwardContactVelocity,-kirchhoffSplitRowMotion(joint,r.gradients)/s.dt);
    d.normalCertificate=measureKirchhoffSplitNormalCertificate(joint,world,d.normalCertificate??{});
    if(s.wallFrictionModes)d.wallFrictionCertificate=certifyKirchhoffWallFrictionModes(joint,
        measurement.wallPhysicalFriction?._batch,{converged:d.physicalAccepted&&measurement.settled});
    d.certified=d.physicalAccepted&&d.biasAccepted&&measurement.settled&&d.unverifiedHistoryKinds.length===0&&
        d.normalCertificate.settled;
    if(d.certified) {
        if(d.historyCommits)throw new Error('A physical timestep cannot commit its history twice');
        // This snapshot is the committed physical motion consumed by the next
        // prediction via the published body velocities; pseudo poses never
        // become an implicit velocity-history channel.
        joint._acceptedPhysicalMotion={step:s.step,dt:s.dt,bodies:s.bodies.map(m=>Object.fromEntries(Object.entries(m).map(([k,v])=>[k,v.slice()]))) };
        if(s.wallFrictionModes)commitKirchhoffWallFrictionHistory(joint,d.wallFrictionCertificate);
        d.historyCommits++;
    }
    return d.certified;
}
export function getKirchhoffSplitMotionStats(world) {
    const joint=world.containments.find(c=>c._splitMotion?.phase==='complete');
    if(!joint)return null;
    const s=joint._splitMotion,d=s.diagnostics,contacts=[];
    const biasByContact=new Map(s.biasBank.contacts.map(c=>[c.contact,c]));
    const friction=new Map((joint._jointFrictionResidual?._batch?.entries??[]).map(e=>[e.contact,e]));
    for(const c of joint.manifold.contacts()) {
        const b=biasByContact.get(c),e=friction.get(c);
        const tangent=e?e.surface.rows.map(r=>r.lambda):[...c.tangentLambda];
        if(c.normalLambda||b?.normalLambda||tangent.some(v=>v))contacts.push({id:c.id,kind:'lumen',feature:e?.record.kind??c.feature,
            normalPhysical:c.normalLambda,normalBias:b?.normalLambda??0,tangentPhysical:tangent,
            mu:e?[...e.surface.group.mu]:[joint.axialFriction,joint.torsionalFriction]});
    }
    const wallFriction=new Map((joint._jointSplitWallFrictionResidual?._batch?.entries??[]).map(e=>[e.contact.key,e]));
    for(const [side,body]of [joint.innerBody,joint.outerBody].entries()) {
        for(let node=body.activeStart;node<body.activeEnd;node++) {
            const physical=body.wallLambda[node],bias=s.biasBank.bodies[side].wallLambda[node],e=wallFriction.get(`wall:${side}:${node}`);
            if(physical||bias||e?.contact.tangentLambda.some(v=>v))contacts.push({id:`wall:${side}:${node}`,kind:'wall',normalPhysical:physical,normalBias:bias,
                tangentPhysical:e?e.surface.rows.map(r=>r.lambda):[0,0],mu:e?[...e.surface.group.mu]:[body.wallStaticFriction,body.wallStaticFriction]});
        }
        const nodes=new Set([...s.pointWalls[side].keys(),...s.biasBank.pointWalls[side].keys()]);
        for(const node of nodes) {
            const p=s.pointWalls[side].get(node),b=s.biasBank.pointWalls[side].get(node),e=wallFriction.get(`split-point-wall:${side}:${node}`);
            if(p?.lambda||b?.lambda||e?.contact.tangentLambda.some(v=>v))contacts.push({id:`wall-point:${side}:${node}`,kind:'wall',normalPhysical:p?.lambda??0,normalBias:b?.lambda??0,
                tangentPhysical:e?e.surface.rows.map(r=>r.lambda):[0,0],mu:e?[...e.surface.group.mu]:[body.wallStaticFriction,body.wallStaticFriction]});
        }
    }
    for(const bank of s.bank.tools)for(let index=0;index<bank.tool.lambdas.length;index++) {
        const p=bank.tool.lambdas[index],b=s.biasBank.tools.find(t=>t.tool===bank.tool)?.lambdas[index]??0;
        const e=joint._jointExternalFrictionResidual?._batch?.entries.find(e=>e.owner===bank.tool&&e.index===index);
        if(p||b||e?.contact.tangentLambda.some(v=>v))contacts.push({id:`tool:${bank.tool.id??'contact'}:${index}`,kind:'tool',normalPhysical:p,normalBias:b,
            tangentPhysical:e?e.surface.rows.map(r=>r.lambda):[0,0],mu:[bank.tool.friction,bank.tool.friction]});
    }
    for(const [index,[sheath,h]]of [...s.sheathHistory].entries())for(let side=0;side<2;side++) {
        const p=joint._coupledBoundaries?.sheaths.get(sheath)?.[side];
        const b=s.biasBank.joints._coupledBoundaries?.sheaths.get(sheath)?.[side];
        for(let node=h.start[side];node<=h.end[side];node++)if(p?.lambda[node]||b?.lambda[node])
            contacts.push({id:`sheath:${index}:${side}:${node}`,kind:'sheath',feature:'radial-interior',
                normalPhysical:p?.lambda[node]??0,normalBias:b?.lambda[node]??0,tangentPhysical:[0,0],mu:[0,0]});
    }
    return {...d,reactionUnits:'xpbd-multiplier',impulseScale:1/s.dt,limitations:[...d.unverifiedHistoryKinds],contacts};
}
