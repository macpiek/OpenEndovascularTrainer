import {createCompositeJointWallSurfaceWorkspace,evaluateCompositeJointWallSurface} from './kirchhoffCompositeJointWallSurface.js';
import {createCompositeJointSurfacePullback,pullbackCompositeJointSurface,evaluateCompositeJointSurfaceLoads} from './kirchhoffCompositeJointSurfacePullback.js';
import {createCompositeFrictionEquationWorkspace,evaluateCompositeFrictionEquation,measureCompositeFriction} from './kirchhoffCompositeFriction.js';

export const JOINT_WALL_PRESSURE_SCHEME='nodal-endpoints-one-sided-surface';
const finite=(v,n)=>{if(!Number.isFinite(v))throw new RangeError(`${n} must be finite`);return v;};
const positive=(v,n)=>{if(!(finite(v,n)>0))throw new RangeError(`${n} must be positive`);return v;};
const vec=(a,n,name)=>{if(a?.length!==n||!Array.from(a).every(Number.isFinite))throw new RangeError(`${name} requires ${n} finite entries`);return Array.from(a);};
const key=(id,node)=>JSON.stringify([id,node]);
const copy=x=>structuredClone(x);
const trace=f=>f===0?'right':f===1?'left':undefined;

/** EXPLICIT physical pressure discretization, selected before loading. A shared
 * material node has ONE force site and a caller-selected one-sided surface
 * frame/spin. There is no capsule/duplicate Fn or Ft to transfer. Every original
 * endpoint AND capsule gap remains an independent feasibility measurement.
 * This prototype owns prepared own histories and provides local band rows;
 * it does not own nonlinear stepping, mechanics, CCD or accepted state.
 */
export function createCompositeJointWallPressurePrototype({layout,modes=[],relativeToolId='wire',coordinates,contactOwners,sites,edgeTools,wall,dt,mu,normalScale,frictionScale,tolerances,history}) {
    positive(dt,'dt');positive(normalScale,'normal equation scale');positive(frictionScale,'friction equation scale');
    wall=copy(wall);
    const coefficients=vec(mu,2,'mu');if(coefficients.some(x=>x<0))throw new RangeError('mu must be nonnegative');
    const tol=Object.fromEntries(['gap','normalEquation','work','frictionSlip','frictionCone','frictionWork','frictionEquation','linearConstraint'].map(n=>[n,positive(tolerances?.[n],n)]));
    if(contactOwners?.edges?.length!==layout.nodeCount-1||coordinates?.length!==layout.nodeCount)throw new RangeError('Frozen chart and owners required');
    const owners=copy(contactOwners),expected=new Map();
    for(const [e,item] of owners.edges.entries()){
        if(item.edge!==e)throw new RangeError('Owners must follow physical edges');
        if(!item.wall)continue;const {owner,radius}=item.wall;positive(radius,'wall radius');
        if(!layout.edgeToolIds[e].includes(owner))throw new RangeError('Present own wall material required');
        for(const node of [e,e+1]){const k=key(owner,node),prior=expected.get(k);if(prior&&prior.radius!==radius)throw new RangeError('Radius discontinuity needs a separate declared physical surface model');expected.set(k,{owner,node,radius});}
    }
    if(!Array.isArray(sites)||sites.length!==expected.size)throw new RangeError('One explicit site must cover each owned material endpoint');
    const incoming=new Map((edgeTools??[]).map(t=>[key(t.id,t.edge),copy(t)])),seen=new Set();
    if(incoming.size!==edgeTools?.length)throw new RangeError('Each own edge history must be supplied exactly once');
    const frozen=sites.map((site,index)=>{
        const {owner,node,edge}=site,k=key(owner,node),end=node-edge,required=expected.get(k),tool=incoming.get(key(owner,edge));
        if(!required||seen.has(k)||![0,1].includes(end)||owners.edges[edge]?.wall?.owner!==owner||!tool||typeof tool.edgeId!=='string'||!tool.edgeId)
            throw new RangeError('Each unique endpoint needs one explicit incident own-edge trace');
        seen.add(k);if(site.trace!==trace(end))throw new RangeError('Explicit one-sided material trace must match the selected incident edge');
        if(tool.coordinates?.length!==2||tool.coordinates[0]!==coordinates[edge]||tool.coordinates[1]!==coordinates[edge+1])throw new RangeError('Own trace must use its actual coordinate interval');
        return {...required,edge,end,trace:site.trace,index,tool,key:JSON.stringify([owner,node,edge,site.trace,tool.edgeId,required.radius])};
    });
    const signature=JSON.stringify({scheme:JOINT_WALL_PRESSURE_SCHEME,coordinates:Array.from(coordinates),edgeToolIds:layout.edgeToolIds,sites:frozen.map(s=>s.key),mu:coefficients,wall});
    if(history&&(history.scheme!==JOINT_WALL_PRESSURE_SCHEME||history.signature!==signature))throw new RangeError('Pressure law/chart/one-sided ownership changed: no implicit Fn/Ft history migration');
    if(history&&history.accepted!==true)throw new RangeError('Only explicitly caller-accepted pressure history can seed a new prepared step');
    const forces=history?Float64Array.from(vec(history.forces,3*frozen.length,'Own endpoint pressure history')):new Float64Array(3*frozen.length);
    const surfaceScratch=createCompositeJointWallSurfaceWorkspace(),equationScratch=createCompositeFrictionEquationWorkspace(),rows=[];
    const normalPhysical={forceMap:new Float64Array(14),slipJacobian:new Float64Array(14),DforceMap:new Float64Array(98),forceMapValid:true,slipJacobianValid:true,DforceMapValid:true};
    for(const site of frozen){
        const args={layout,modes,relativeToolId,tools:[site.tool]};site.surface=createCompositeJointSurfacePullback(args);site.normal=createCompositeJointSurfacePullback(args);
        const mapping=site.surface,n=mapping.dofCount;
        site.rows=[0,1,2].map(component=>{
            const row={anchorNode:mapping.anchorNode,commonDofs:mapping.commonDofs,relativeDofs:mapping.relativeDofs,unit:'mm',
                multiplierDofs:component===0?new Int32Array():Int32Array.of(3*site.index,3*site.index+(component===1?2:1)),
                multiplierJacobian:new Float64Array(component===0?0:2),jacobian:new Float64Array(n),forceColumn:new Float64Array(n),geometricTangent:new Float64Array(n*n),
                tolerance:tol.linearConstraint,geometricTangentValid:false};rows.push(row);return row;
        });
    }
    const out={scheme:JOINT_WALL_PRESSURE_SCHEME,signature,forces,rows,siteKeys:Object.freeze(frozen.map(s=>s.key)),
        commonResidual:new Float64Array(layout.dofCount),relativeResidual:new Float64Array(3*modes.length),valid:false,certified:false,
        scope:'explicit-nodal-pressure-local-equations-not-a-timestep',normalForceSlots:frozen.length,frictionForceSlots:2*frozen.length,
        capsulePressureSlots:0,queries:0,proof:null};
    function invalidate(){out.valid=false;out.proof=null;out.commonResidual.fill(NaN);out.relativeResidual.fill(NaN);for(const r of rows){r.geometricTangentValid=false;r.residual=NaN;r.jacobian.fill(NaN);r.forceColumn.fill(NaN);r.geometricTangent.fill(NaN);}}
    function scatter(load,mapping,factor=1){mapping.commonDofs.forEach((d,i)=>out.commonResidual[d]+=factor*load.common[i]);mapping.relativeDofs.forEach((d,i)=>out.relativeResidual[d]+=factor*load.relative[i]);}
    out.refresh=({envelope,positionsByTool,anglesByTool,field,plane,localFaceIndices=[],order='full'})=>{
        invalidate();if(order!=='full'&&order!=='value')throw new RangeError('Explicit full/value pressure order required');
        const full=order==='full';
        try{
            vec(forces,3*frozen.length,'Candidate endpoint forces');out.commonResidual.fill(0);out.relativeResidual.fill(0);
            const original=[];let minGap=Infinity,converged=true;
            for(const [edge,owner] of owners.edges.entries())if(owner.wall){
                const group=envelope.rows.filter(r=>r.edge===edge);
                if(group.length!==3||new Set(group.map(r=>r.role)).size!==3)throw new RangeError('All original endpoint and capsule inequalities required');
                for(const role of ['proximal','distal','capsule']){
                    const r=group.find(r=>r.role===role);if(!r?.included||r.owner!==owner.wall.owner||r.radius!==owner.wall.radius)throw new RangeError('Original envelope owner/radius changed');
                    finite(r.gap,'Original wall gap');minGap=Math.min(minGap,r.gap);converged=converged&&r.gap>=-tol.gap;
                    original.push({owner:r.owner,edge,role,gap:r.gap,t:r.t,pressureDof:role==='capsule'?false:'single-node-owner'});
                }
            }
            const proofs=[];
            for(const s of frozen){
                const raw=envelope.rows.find(r=>r.edge===s.edge&&r.role===(s.end===0?'proximal':'distal')),p=positionsByTool.get(s.owner),angle=anglesByTool.get(s.owner)?.[s.edge];
                const positions=[p[s.edge],p[s.edge+1]],tool={...s.tool,positions,angle,radius:s.radius,trace:s.trace};
                const map=tool.materialMap,old=tool.materialPath.previousMap,dx=tool.coordinates[1]-tool.coordinates[0];
                const oldFraction=(map.sStart+map.dsDx*dx*s.end-old.sStart)/(old.dsDx*dx);
                tool.materialPath={...tool.materialPath,previousTrace:trace(oldFraction)};
                const surface=evaluateCompositeJointWallSurface({current:{field,row:raw,positions,plane,localFaceIndices},tool,dt,wall,order},surfaceScratch);
                const Fn=forces[3*s.index],Ft=Array.from(forces.slice(3*s.index+1,3*s.index+3)),g=raw.gap,active=Fn-normalScale*g>0;
                const tangential=pullbackCompositeJointSurface(full?surface:{tools:surface.tools,forceMap:surface.forceMap,forceMapValid:true},s.surface);
                const slip=Array.from(surface.increment),eq=evaluateCompositeFrictionEquation({traction:Ft,slip,normalForce:Fn,mu:coefficients,penalty:frictionScale},equationScratch),size=tangential.dofCount;
                for(const [c,r] of s.rows.slice(1).entries()){
                    r.residual=eq.residual[c];r.multiplierDerivative=eq.tractionJacobian[2*c+c];r.multiplierJacobian.set([eq.normalDerivative[c],eq.tractionJacobian[2*c+1-c]]);
                    r.forceColumn.set(tangential.rows[c].forceColumn);r.geometricTangentValid=full;
                    if(full){for(let j=0;j<size;j++)r.jacobian[j]=eq.slipJacobian[2*c]*tangential.slipJacobian[j]+eq.slipJacobian[2*c+1]*tangential.slipJacobian[size+j];
                        for(let j=0;j<size*size;j++)r.geometricTangent[j]=-Ft[c]*tangential.rows[c].forceDerivative[j];}
                }
                const tangentLoad=evaluateCompositeJointSurfaceLoads(Ft,tangential);scatter(tangentLoad,tangential,-1);
                normalPhysical.tools=surface.tools;normalPhysical.forceMap.fill(0);normalPhysical.slipJacobian.fill(0);normalPhysical.DforceMap.fill(0);
                for(let i=0;i<6;i++){normalPhysical.forceMap[2*i]=-raw.forceColumn[i];normalPhysical.slipJacobian[i]=raw.gapJacobian[i];for(let j=0;j<6;j++)normalPhysical.DforceMap[(2*i)*7+j]=raw.normalDerivative[i*6+j];}
                const normal=pullbackCompositeJointSurface(normalPhysical,s.normal),nr=s.rows[0];
                nr.residual=active?g:Fn/normalScale;nr.multiplierDerivative=active?0:1/normalScale;nr.forceColumn.set(normal.rows[0].forceColumn);nr.geometricTangentValid=full;
                if(full){for(let j=0;j<size;j++)nr.jacobian[j]=active?normal.slipJacobian[j]:0;for(let j=0;j<size*size;j++)nr.geometricTangent[j]=-Fn*normal.rows[0].forceDerivative[j];}
                scatter(evaluateCompositeJointSurfaceLoads([Fn,0],normal),normal,-1);
                const kkt=Fn>=0?measureCompositeFriction({traction:Ft,slip,normalForce:Fn,mu:coefficients,slipTolerance:tol.frictionSlip,coneTolerance:tol.frictionCone,workTolerance:tol.frictionWork}):{converged:false};
                const ok=Fn>=0&&g>=-tol.gap&&Math.abs(Fn*g)<=tol.work&&Math.abs(nr.residual)<=tol.normalEquation&&kkt.converged&&eq.residual.every(x=>Math.abs(x)<=tol.frictionEquation);
                converged=converged&&ok;proofs.push({key:s.key,owner:s.owner,node:s.node,edge:s.edge,trace:s.trace,Fn,Ft,slip,gap:g,normalActive:active,ncp:nr.residual,kkt:copy(kkt),converged:ok,
                    physicalTangentLoad:Array.from(tangentLoad.physical),point:Array.from(surface.point),axes:copy(surface.physicalForce.axes),omegaMap:copy(surface.physicalForce.tools[0].omegaMap)});
            }
            out.valid=true;out.proof={converged,minGap,originalInequalities:original,sites:proofs,pressureDiscretization:JOINT_WALL_PRESSURE_SCHEME};return out;
        }catch(e){invalidate();throw e;}
    };
    out.copyCandidateHistory=()=>({scheme:JOINT_WALL_PRESSURE_SCHEME,signature,forces:forces.slice(),accepted:false});
    invalidate();return out;
}
