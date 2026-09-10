import {createCompositeContinuousWallPoint,queryCompositeContinuousWallPoint} from './kirchhoffCompositeContinuousWallGeometry.js';
import {compositeJointWallSourceSignature} from './kirchhoffCompositeJointWallRows.js';

const fail=message=>{const e=new RangeError(message);e.code='unsupported-continuous-wall-contact';throw e;};
const positive=(v,name)=>{if(!Number.isFinite(v)||v<=0)fail(`${name} must be finite and positive`);return v;};

/** Independent pressure unknowns at original-resolution material samples on
 * the C2 curve. Every sample has its current original sphere g/G/B/DB. The
 * certificate covers this EXPLICIT discretization, never continuum clearance.
 * Source producer owns inherited sample coverage and radii; no old capsule
 * gradient, remote wire post-pass or force clipping is used in this manager.
 */
export function createCompositeJointContinuousWallRows({layout,coordinates,modes,relativeToolId='wire',wall,geometryByTool,history=null,tolerances}) {
    if(wall?.mode!=='wall-normal'||wall.friction!=='none'||wall.contactMode!=='continuous-samples'||wall.pressureDiscretization!=='original-resolution-C2-material-samples')
        fail('Explicit normal-only C2 material sample pressure is required');
    if(!(geometryByTool instanceof Map)||!Array.isArray(wall.pressureSites)||!wall.pressureSites.length)fail('C2 sampled pressure needs actual geometry and declared sites');
    const k=positive(wall.forcePerLength,'Wall forcePerLength'),source=compositeJointWallSourceSignature(wall.field),seen=new Set(),
        chart=JSON.stringify({source,chartId:wall.chartId,edges:layout.edgeToolIds,coordinates:Array.from(coordinates),relativeToolId,modes});
    const samples=wall.pressureSites.map((site,index)=>{
        if(typeof site.id!=='string'||!site.id||seen.has(site.id)||!Number.isInteger(site.edge)||!layout.edgeToolIds[site.edge]?.includes(site.owner))fail('Distinct original physical wall sample identities are required');seen.add(site.id);
        const geometry=geometryByTool.get(site.owner)?.edges?.[site.edge],point=createCompositeContinuousWallPoint({layout,modes,geometry,owner:site.owner,relativeToolId,
            field:wall.field,fraction:site.fraction,radius:site.radius}),size=point.commonDofs.length+point.relativeDofs.length,
            row={toolId:site.owner,edge:site.edge,index,forceIndex:index,anchorNode:point.anchorNode,commonDofs:point.commonDofs,relativeDofs:point.relativeDofs,
                constraintSupport:point.constraintSupport,unit:'mm',residual:NaN,jacobian:new Float64Array(size),forceColumn:new Float64Array(size),
                geometricTangent:new Float64Array(size*size),geometricTangentValid:false,multiplierDerivative:0,tolerance:tolerances.linearConstraint};
        return {site:{...site},point,row,index};
    }),rows=samples.map(s=>s.row),normalForces=new Float64Array(samples.length),
        nodalForces=new Map([...layout.spins.keys()].map(id=>[id,Array.from({length:layout.nodeCount},()=>[0,0,0])])),
        cg=new Float64Array(layout.dofCount),rg=new Float64Array(modes.reduce((n,m)=>n+m.basis.length,0));
    if(history!==null) {
        if(history.signature!==chart||history.contactMode!=='continuous-samples'||history.friction!=='none'||!Array.isArray(history.records))fail('Accepted C2 pressure belongs to another field/chart');
        const incoming=new Map();
        for(const r of history.records){if(incoming.has(r.site?.id)||!Number.isFinite(r.force)||r.force<0)fail('Accepted pressure needs distinct finite nonnegative forces');incoming.set(r.site.id,r);}
        for(const s of samples) {
            const r=incoming.get(s.site.id);if(!r)continue;
            if(JSON.stringify(r.site)!==JSON.stringify(s.site)&&r.force!==0)fail('Loaded C2 sample changed physical support or radius');
            normalForces[s.index]=JSON.stringify(r.site)===JSON.stringify(s.site)?r.force:0;incoming.delete(s.site.id);
        }
        if([...incoming.values()].some(r=>r.force!==0))fail('Loaded pressure site disappeared without force transfer');
    }
    const checkpoints=new WeakMap();let commitReady=false,lastPositions=null,lastPoints=null,lastForces=null;
    function invalidate(){commitReady=false;lastPositions=null;for(const s of samples){s.row.residual=NaN;s.row.geometricTangentValid=false;}for(const p of nodalForces.values())p.forEach(v=>v.fill(0));}
    function refresh({toolPositions,commonResidual,relativeResidual,order,consumeQuery=()=>{}}) {
        invalidate();cg.fill(0);rg.fill(0);
        if(!['full','gradient'].includes(order)||commonResidual.length!==cg.length||relativeResidual.length!==rg.length||!commonResidual.every(Number.isFinite)||!relativeResidual.every(Number.isFinite))fail('Current finite common and relative residuals are required');
        let minGap=Infinity,minForce=Infinity,penetration=0,ncp=0,complementarity=0,merit=0,lineSearchMerit=0;const proofs=[];
        try {
            for(const s of samples) {
                const {row,index,point}=s,Fn=normalForces[index];if(!Number.isFinite(Fn))fail('Nonfinite private wall pressure');
                queryCompositeContinuousWallPoint(point,{toolPositions,order,consumeQuery,localFaceIndices:wall.localFaceIndices??[]});
                const gap=point.gap,eliminated=!point.supported&&Fn===0&&gap>0,active=Fn-k*gap>0;
                if(!point.supported&&!eliminated)fail(`Unsupported active/loaded original C2 point (${point.source}; ${point.reason})`);
                row.residual=active?gap:Fn/k;row.multiplierDerivative=active?0:1/k;row.geometricTangentValid=order==='full';
                if(eliminated){row.jacobian.fill(0);row.forceColumn.fill(0);row.geometricTangent.fill(0);}
                else {
                    row.jacobian.forEach((_,i)=>row.jacobian[i]=active?point.gapJacobian[i]:0);row.forceColumn.set(point.forceColumn);
                    if(order==='full')row.geometricTangent.forEach((_,i)=>row.geometricTangent[i]=Fn===0?0:-Fn*point.normalDerivative[i]);
                    row.commonDofs.forEach((d,i)=>cg[d]+=Fn*row.forceColumn[i]);row.relativeDofs.forEach((d,i)=>rg[d]+=Fn*row.forceColumn[row.commonDofs.length+i]);
                    point.nodeIndices.forEach((node,j)=>[0,1,2].forEach(c=>nodalForces.get(s.site.owner)[node][c]+=Fn*point.physicalForceColumn[3*j+c]));
                }
                const penetrationHere=Math.max(0,-gap),negative=Math.max(0,-Fn),residual=Math.abs(row.residual),work=Math.abs(Fn*gap),
                    local=(penetrationHere/tolerances.wallGap)**2+(negative/tolerances.force)**2+(residual/tolerances.wallNcp)**2;
                minGap=Math.min(minGap,gap);minForce=Math.min(minForce,Fn);penetration=Math.max(penetration,penetrationHere);ncp=Math.max(ncp,residual);complementarity=Math.max(complementarity,work);
                lineSearchMerit+=local;merit+=local+(work/tolerances.wallWork)**2;
                proofs.push({sampleId:s.site.id,owner:s.site.owner,edge:s.site.edge,fraction:s.site.fraction,gap,Fn,ncp:residual,complementarity:work,active,eliminated,source:point.source});
            }
            if(!Number.isFinite(merit)||!cg.every(Number.isFinite)||!rg.every(Number.isFinite))fail('Nonfinite sampled C2 wall certificate');
            cg.forEach((v,i)=>commonResidual[i]+=v);rg.forEach((v,i)=>relativeResidual[i]+=v);
            commitReady=minForce>=0&&penetration<=tolerances.wallGap&&ncp<=tolerances.wallNcp&&complementarity<=tolerances.wallWork;
            lastPositions=toolPositions;lastPoints=samples.map(s=>s.point.nodeIndices.map(node=>toolPositions.get(s.site.owner)[node].slice()));lastForces=normalForces.slice();
            return {converged:commitReady,minGap,minForce,penetration,ncp,complementarity,merit,lineSearchMerit,samples:proofs,
                sampleCount:samples.length,coverage:'declared-original-resolution-samples',continuumClearanceCertified:false,pressureDiscretization:wall.pressureDiscretization};
        } catch(error){invalidate();throw error;}
    }
    function checkpoint(){const token=Object.freeze({});checkpoints.set(token,normalForces.slice());return token;}
    function restore(token){const data=checkpoints.get(token);if(!data)fail('Use the current C2 wall checkpoint');normalForces.set(data);invalidate();}
    function commit() {
        if(!commitReady||!lastPositions||normalForces.some((v,i)=>v!==lastForces[i])||samples.some((s,i)=>s.point.nodeIndices.some((node,j)=>lastPositions.get(s.site.owner)[node].some((v,k)=>v!==lastPoints[i][j][k]))))fail('C2 wall commit requires unchanged freshly certified state');
        return {signature:chart,contactMode:'continuous-samples',friction:'none',normalForces:normalForces.slice(),records:samples.map(s=>({site:{...s.site},force:normalForces[s.index]}))};
    }
    function assertCurrentSurface({toolPositions}) {
        if(!lastPositions||lastPositions!==toolPositions||normalForces.some((v,i)=>v!==lastForces[i])||samples.some((s,i)=>s.point.nodeIndices.some((node,j)=>toolPositions.get(s.site.owner)[node].some((v,k)=>v!==lastPoints[i][j][k]))))fail('Current normal point queries are required before surface friction');
        return true;
    }
    return {signature:chart,samples,rows,rowForceIndices:rows.map((_,i)=>i),normalForces,nodalForces,surfaceRecords:[],refresh,checkpoint,restore,commit,assertCurrentSurface,
        discoverCharts(){return {rowStructureChanged:false};}};
}
