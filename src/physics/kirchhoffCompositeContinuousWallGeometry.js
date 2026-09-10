import {Vector3} from 'three';
import {createContactResult} from './collision/vesselContactField.js';
import {compositeContinuousBezierControls,evaluateCompositeContinuousGeometry,sampleCompositeContinuousBasis} from './kirchhoffCompositeContinuousGeometry.js';
import {createCompositeWallGeometryWorkspace,differentiateCompositeWallContact} from './kirchhoffCompositeWallGeometry.js';
import {createCompositeWallBvhGeometryWorkspace,differentiateCompositeWallBvhContact} from './kirchhoffCompositeWallBvhGeometry.js';
import {compositeJointWallSourceSignature} from './kirchhoffCompositeJointWallRows.js';

const plans=new WeakMap(),finite=(v,name)=>{if(!Number.isFinite(v))throw new RangeError(`${name} must be finite`);return v;};
const vector=(v,n,name)=>{if(v?.length!==n||!Array.from(v).every(Number.isFinite))throw new RangeError(`${name} needs ${n} finite entries`);return Array.from(v);};
const dot=(a,b)=>a.reduce((s,v,k)=>s+v*b[k],0);
const invalidate=out=>{out.supported=out.hessianValid=false;out.reason='not-evaluated';out.gap=out.signedDistance=NaN;for(const key of ['gapJacobian','normalForceColumn','forceColumn','normalDerivative','physicalForceColumn','pointGapGradient','pointNormalDerivative'])out[key].fill(NaN);};

/** A fixed material pressure location on the SAME compiled C2 curve as the
 * rod model. Wider interpolation support is retained in full; no chord,
 * endpoint scatter, frozen normal or hidden spin coupling is substituted.
 * This is an operator for a declared sample, not a full contact certificate.
 */
export function createCompositeContinuousWallPoint({layout,modes,geometry,owner,relativeToolId='wire',fraction,radius,field}) {
    if(typeof field?.querySphere!=='function')throw new TypeError('An original wall point provider is required');
    if(!(finite(radius,'Wall radius')>=0))throw new RangeError('Wall radius must be nonnegative');
    const basis=sampleCompositeContinuousBasis(geometry,fraction),nodes=geometry.nodeIndices.slice(),edge=geometry.edge;
    if(!layout.edgeToolIds[edge]?.includes(owner)||nodes.some(node=>![node-1,node].some(e=>layout.edgeToolIds[e]?.includes(owner)))||
        !(layout.positionSupports??[]).some(s=>s.length===nodes.length&&s.every((node,i)=>node===nodes[i])))
        throw new RangeError('The C2 wall point requires its actual complete declared physical support');
    const byNode=new Map(modes.map(m=>[m.node,m])),commonDofs=Int32Array.from(nodes.flatMap(node=>[0,1,2].map(k=>layout.positions[node]+k))),
        relativeDofs=Int32Array.from(owner===relativeToolId?modes.filter(m=>nodes.includes(m.node)).flatMap(m=>m.relativeDofs):[]),
        relativeIndex=new Map(Array.from(relativeDofs,(d,i)=>[d,commonDofs.length+i])),columns=Array.from({length:commonDofs.length+relativeDofs.length},()=>[0,0,0]);
    for(const [i,node] of nodes.entries()) {
        for(let k=0;k<3;k++)columns[3*i+k][k]=basis.weights[i];
        if(owner===relativeToolId){
            const m=byNode.get(node);
            if(m){
                if(m.basis?.length!==3||m.relativeDofs?.length!==3)throw new RangeError('Continuous wall requires all three relative coordinates');
                m.basis.forEach((b,a)=>{vector(b,3,'Relative basis');for(let j=0;j<3;j++)if(Math.abs(dot(b,m.basis[j])-(a===j?1:0))>1e-10)throw new RangeError('Relative basis must be orthonormal');});
                m.basis.forEach((b,a)=>b.forEach((v,k)=>columns[relativeIndex.get(m.relativeDofs[a])][k]+=basis.weights[i]*v));
            }
        }
    }
    const size=columns.length,out={owner,edge,fraction,radius,nodeIndices:nodes.slice(),positionWeights:Float64Array.from(basis.weights),anchorNode:edge,commonDofs,relativeDofs,
        constraintSupport:{kind:'continuous-wall-point',toolId:owner,edge,nodeIndices:nodes.slice()},
        gapJacobian:new Float64Array(size),normalForceColumn:new Float64Array(size),forceColumn:new Float64Array(size),normalDerivative:new Float64Array(size*size),
        physicalForceColumn:new Float64Array(3*nodes.length),position:new Float64Array(3),normal:new Float64Array(3),closestPoint:new Float64Array(3),
        pointGapGradient:new Float64Array(3),pointNormalDerivative:new Float64Array(9),signedDistance:NaN,gap:NaN,supported:false,hessianValid:false,
        pressureDiscretization:'fixed-C2-material-point',wholeCurveCertified:false,reason:'not-evaluated',source:null};
    plans.set(out,{layout,modes,geometry,owner,fraction,radius,field,source:compositeJointWallSourceSignature(field),query:field.querySphere,nodes,columns,basis,
        sdf:createCompositeWallGeometryWorkspace(1),bvh:createCompositeWallBvhGeometryWorkspace(1),contact:createContactResult()});
    return out;
}

/** Exactly one original point query. G=T^T dg/dx, B=T^T n,
 * DB=T^T Dn T. T includes every curve shape coefficient and the complete
 * relative basis. Mechanical residual is Fn*(-B), tangent -Fn*DB.
 */
export function queryCompositeContinuousWallPoint(out,{toolPositions,order='full',consumeQuery=()=>{},localFaceIndices=[]}={}) {
    const p=plans.get(out);if(!p)throw new TypeError('Use a compiled continuous wall point');invalidate(out);
    if(!['full','gradient'].includes(order))throw new RangeError('Wall point order must be full or gradient');
    if(p.query!==p.field.querySphere||p.source!==compositeJointWallSourceSignature(p.field))throw new RangeError('Original wall source changed');
    const own=toolPositions.get(p.owner),positions=p.nodes.map(node=>own[node]),curve=evaluateCompositeContinuousGeometry(p.geometry,{positions,fraction:p.fraction});
    consumeQuery();const raw=p.field.querySphere(curve.position,p.radius,p.contact);out.position.set(curve.position);out.gap=finite(raw.signedGap,'Original wall gap');out.signedDistance=finite(raw.signedDistance,'Original signed wall distance');out.closestPoint.set(raw.closestPoint.values);out.source=raw.source;
    let g;
    if(raw.source==='sparse-sdf')g=differentiateCompositeWallContact({field:p.field,contact:raw,positions:[curve.position],radius:p.radius},p.sdf);
    else if(raw.source==='sparse-sdf-bvh')g=differentiateCompositeWallBvhContact({field:p.field,contact:raw,positions:[curve.position],radius:p.radius,localFaceIndices},p.bvh);
    else {out.reason=`unsupported-source:${raw.source}`;return out;}
    if(!g.supported){out.reason=g.reason;return out;}
    out.normal.set(g.normal);out.pointGapGradient.set(g.gapGradient);if(order==='full')out.pointNormalDerivative.set(g.normalForceJacobian);const n=p.columns.length;
    for(let i=0;i<n;i++) {
        out.gapJacobian[i]=dot(p.columns[i],g.gapGradient);out.normalForceColumn[i]=dot(p.columns[i],g.normalForceColumn);out.forceColumn[i]=-out.normalForceColumn[i];
        if(order==='full')for(let j=0;j<n;j++) {
            let value=0;for(let a=0;a<3;a++)for(let b=0;b<3;b++)value+=p.columns[i][a]*g.normalForceJacobian[3*a+b]*p.columns[j][b];
            out.normalDerivative[n*i+j]=value;
        }
    }
    p.nodes.forEach((_,i)=>g.normalForceColumn.forEach((v,k)=>out.physicalForceColumn[3*i+k]=p.basis.weights[i]*v));
    if([out.gapJacobian,out.normalForceColumn,out.physicalForceColumn,...(order==='full'?[out.normalDerivative]:[])].some(a=>!a.every(Number.isFinite))){invalidate(out);out.reason='nonfinite-C2-pullback';return out;}
    out.reason=null;out.supported=true;out.hessianValid=order==='full';return out;
}

function halves(controls) {
    const rows=[controls.map(p=>p.slice())];
    while(rows.at(-1).length>1){const old=rows.at(-1);rows.push(old.slice(1).map((p,i)=>p.map((v,k)=>.5*old[i][k]+.5*v)));}
    return [rows.map(r=>r[0]),rows.map(r=>r.at(-1)).reverse()];
}

/** Whole-curve UNSIGNED clearance against every triangle in the actual
 * source BVH, independently of nodal/pressure sampling. A quintic's convex
 * hull lies in the ball covering its six controls. Exact closest-triangle
 * distance is 1-Lipschitz, giving lowerBound=d(center)-ballRadius-radius.
 * Subdivision refines only inconclusive intervals; budget exhaustion cannot
 * produce a certificate. This proves mesh separation, NOT which side of the
 * wall the curve occupies or the provider's SDF/sign/cache switching law.
 * Thus `signedProviderCertified` is always false and cannot authorize a dt.
 */
export function measureCompositeContinuousWallMeshClearance({field,geometry,positions,radius,tolerance=1e-8,maxDepth=16,maxQueries=4096,consumeQuery=()=>{}}) {
    if(!(finite(radius,'Wall radius')>=0)||!(finite(tolerance,'Clearance tolerance')>0)||!Number.isInteger(maxDepth)||maxDepth<0||!Number.isInteger(maxQueries)||maxQueries<1)
        throw new RangeError('Finite geometry radius, tolerance and explicit coverage budgets are required');
    const mesh=field?.fallbackGeometry,tree=mesh?.boundsTree;
    if(!tree||tree.geometry!==mesh||typeof tree.closestPointToPoint!=='function')throw new RangeError('The actual source triangle BVH is required');
    const query=new Vector3(),target={point:new Vector3(),distance:Infinity,faceIndex:-1},stack=[{controls:compositeContinuousBezierControls(geometry,positions),a:0,b:1,depth:0}],intervals=[];
    let queries=0,lowerBound=Infinity,minimumSampleGap=Infinity,witness=null;
    const result=(meshCertified,status)=>({meshCertified,signedProviderCertified:false,status,queries,lowerBound,minimumSampleGap,witness,intervals,
        scope:'whole-C2-curve-unsigned-source-triangle-clearance'});
    while(stack.length) {
        if(queries>=maxQueries)return result(false,'coverage-query-budget');
        const current=stack.pop(),[left,right]=halves(current.controls),center=left.at(-1),ball=Math.max(...current.controls.map(p=>Math.hypot(...p.map((v,k)=>v-center[k])))),
            round=256*Number.EPSILON*Math.max(1,radius,ball,...current.controls.flat().map(Math.abs));
        consumeQuery();queries++;target.distance=Infinity;const hit=tree.closestPointToPoint(query.fromArray(center),target);
        if(!hit||!Number.isFinite(hit.distance))return result(false,'undefined-source-BVH-distance');
        const gap=hit.distance-radius,bound=gap-ball-round;
        minimumSampleGap=Math.min(minimumSampleGap,gap);if(gap<0&&(!witness||gap<witness.gap))witness={fraction:.5*(current.a+current.b),gap,position:center.slice(),faceIndex:hit.faceIndex};
        if(gap < -tolerance)return result(false,'mesh-penetration');
        if(bound>=-tolerance){lowerBound=Math.min(lowerBound,bound);intervals.push({fractions:[current.a,current.b],lowerBound:bound});continue;}
        if(current.depth>=maxDepth)return result(false,'coverage-depth-budget');
        const middle=.5*(current.a+current.b);stack.push({controls:right,a:middle,b:current.b,depth:current.depth+1},{controls:left,a:current.a,b:middle,depth:current.depth+1});
    }
    return result(true,'mesh-clearance-certified');
}
