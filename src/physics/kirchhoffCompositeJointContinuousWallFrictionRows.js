import {createCompositeContinuousFrame} from './kirchhoffCompositeContinuousFrame.js';
import {evaluateCompositeContinuousGeometry} from './kirchhoffCompositeContinuousGeometry.js';
import {createCompositeContinuousWallSurface,evaluateCompositeContinuousWallSurface} from './kirchhoffCompositeContinuousWallSurface.js';
import {sampleCompositeContinuousRateHistory} from './kirchhoffCompositeContinuousRateHistory.js';
import {createCompositeFrictionEquationWorkspace,evaluateCompositeFrictionEquation,measureCompositeFriction} from './kirchhoffCompositeFriction.js';
import {assessCompositeStaticKineticFriction} from './kirchhoffCompositeStaticKineticFriction.js';

const fail=message=>{const e=new RangeError(message);e.code='unsupported-continuous-wall-friction';throw e;};
const vec=(v,n,name)=>{if(v?.length!==n||!Array.from(v).every(Number.isFinite))fail(`${name} needs ${n} finite entries`);return Array.from(v);};
const dot=(a,b)=>a.reduce((sum,v,k)=>sum+v*b[k],0),cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];

function mapping(frame,layout,modes,relativeToolId) {
    const columns=frame.configurationColumns,commonDofs=Int32Array.from([...new Set(columns.map(c=>c.kind==='angle'?layout.spins.get(c.toolId)[c.edge]:layout.positions[c.node]+c.component))].sort((a,b)=>a-b)),
        relativeDofs=Int32Array.from(frame.toolId===relativeToolId?modes.filter(m=>frame.positionNodeIndices.includes(m.node)).flatMap(m=>m.relativeDofs):[]),
        commonIndex=new Map(Array.from(commonDofs,(d,i)=>[d,i])),relativeIndex=new Map(Array.from(relativeDofs,(d,i)=>[d,commonDofs.length+i])),byNode=new Map(modes.map(m=>[m.node,m])),
        terms=Array.from({length:commonDofs.length+relativeDofs.length},()=>[]);
    columns.forEach((c,p)=>{
        const d=c.kind==='angle'?layout.spins.get(c.toolId)[c.edge]:layout.positions[c.node]+c.component;terms[commonIndex.get(d)].push([p,1]);
        if(c.kind==='position'&&c.toolId===relativeToolId)byNode.get(c.node)?.basis.forEach((b,a)=>{if(b[c.component]!==0)terms[relativeIndex.get(byNode.get(c.node).relativeDofs[a])].push([p,b[c.component]]);});
    });
    return {commonDofs,relativeDofs,terms,size:terms.length,N:columns.length,columns};
}

/** Static/kinetic isotropic Coulomb at the normal manager's original C2
 * material samples. Slip is dt * the implicit surface rate, with complete
 * moving-normal/witness/axis and own director/spin derivatives. It is NOT
 * identified with an exact finite same-material surface displacement.
 */
export function createCompositeJointContinuousWallFrictionRows({state,candidate,prepared,normal,wall,dt,tolerances,normalRowOffset,frictionRowOffset}) {
    const policy=wall?.friction,samples=normal?.samples;
    if(wall?.mode!=='wall-coulomb'||wall.contactMode!=='continuous-samples'||policy?.law!=='coulomb-static-kinetic'||policy.rateMode!=='backward-euler-grid'||
        policy.slipModel!=='implicit-backward-euler-surface-rate'||policy.finiteStepSlipKnown!==false||!Array.isArray(samples)||!samples.length||!(dt>0))fail('Explicit C2 implicit-rate static/kinetic wall law is required');
    if(state.elasticityGeometry!=='continuous-material-frame')fail('Wall surface forces require the complete declared continuous material frame support');
    const penalty=policy.forcePerLength;if(!(penalty>0)||!Number.isFinite(penalty))fail('Positive friction forcePerLength is required');
    const coefficients=new Map(),owners=new Set(samples.map(s=>s.site.owner));
    for(const c of policy.muByOwner??[]) {
        if(!owners.has(c.owner)||coefficients.has(c.owner))fail('Coefficients must cover distinct actual exposed materials');
        const muStatic=vec(c.muStatic,2,'Static coefficients'),muKinetic=vec(c.muKinetic,2,'Kinetic coefficients');
        if(muStatic.some((v,i)=>v<0||v<muKinetic[i])||muKinetic.some(v=>v<0)||muStatic[0]!==muStatic[1]||muKinetic[0]!==muKinetic[1])fail('Actual scalar source friction requires isotropic static and kinetic cones');
        coefficients.set(c.owner,{muStatic,muKinetic});
    }
    if(coefficients.size!==owners.size)fail('Every actual exposed owner needs its coefficients');
    const signature=JSON.stringify({normal:normal.signature,coefficients:[...coefficients],law:policy.law,rateMode:policy.rateMode}),history=state.wallFrictionState,
        old=new Map(),incoming=new Map((policy.incomingSurfaceMotion??[]).map(r=>[r.sampleId,r]));
    if(history){if(history.signature!==signature||history.slipModel!==policy.slipModel||!Array.isArray(history.records))fail('Accepted C2 friction belongs to another source/chart/law');
        for(const r of history.records){if(old.has(r.id))fail('Duplicate accepted friction sample');old.set(r.id,r);}}
    const {layout}=state,tractions=new Float64Array(2*samples.length),rows=[],records=samples.map((sample,index)=>{
        const site=sample.site,tool=state.tools.find(t=>t.id===site.owner),geometry=candidate.inertiaGeometryByTool.get(site.owner),
            frame=createCompositeContinuousFrame({geometry,edge:site.edge,toolId:site.owner,previousPositions:state.toolPositions.get(site.owner),previousAngles:state.angles.get(site.owner),reference:tool.reference,referenceTwists:tool.referenceTwists}),
            map=prepared.inertiaEdges[site.edge].tools.find(t=>t.id===site.owner)?.materialMap,mapped=mapping(frame,layout,candidate.modes,state.relativeToolId),previous=old.get(site.id),ownRows=[];
        if(!map)fail('Every surface needs its own prepared material feed map');
        if(previous){tractions.set(vec(previous.traction,2,'Accepted traction'),2*index);old.delete(site.id);}
        for(let c=0;c<2;c++) {
            const row={toolId:site.owner,edge:site.edge,anchorNode:site.edge,index:2*index+c,unit:'mm',commonDofs:mapped.commonDofs,relativeDofs:mapped.relativeDofs,
                constraintSupport:{kind:'continuous-wall-friction',toolId:site.owner,edge:site.edge,positionNodes:frame.positionNodeIndices.slice(),angleEdges:frame.angleEdgeIndices.slice()},
                multiplierDofs:Int32Array.of(normalRowOffset+index,frictionRowOffset+2*index+1-c),multiplierJacobian:new Float64Array(2),multiplierDerivative:NaN,
                residual:NaN,jacobian:new Float64Array(mapped.size),forceColumn:new Float64Array(mapped.size),geometricTangent:new Float64Array(mapped.size*mapped.size),geometricTangentValid:false,tolerance:tolerances.linearConstraint};
            ownRows.push(row);rows.push(row);
        }
        return {sample,index,site,geometry,frame,map,mapped,rows:ownRows,previous,surface:null,seed:previous?.seedAxis??null,mode:null,initialMode:null,demotions:0,latest:null,decision:null,coefficients:coefficients.get(site.owner)};
    });
    if([...old.values()].some(r=>r.traction.some(v=>v!==0)))fail('Loaded friction sample disappeared without force transfer');
    const nodalForces=new Map([...layout.spins.keys()].map(id=>[id,Array.from({length:layout.nodeCount},()=>[0,0,0])])),spinTorques=new Map([...layout.spins].map(([id,a])=>[id,new Float64Array(a.length)])),
        cg=new Float64Array(layout.dofCount),rg=new Float64Array(candidate.relative.length),equation=createCompositeFrictionEquationWorkspace();
    let preparedReady=false,commitReady=false,last=null;
    const currentValues=toolPositions=>records.flatMap(r=>r.frame.configurationColumns.map(c=>c.kind==='position'?toolPositions.get(c.toolId)[c.node][c.component]:candidate.angles.get(c.toolId)[c.edge]));
    const same=(a,b)=>a.length===b.length&&a.every((v,i)=>v===b[i]);
    function incomingRate(rec,label) {
        if(state.continuousRateHistory)return sampleCompositeContinuousRateHistory({history:state.continuousRateHistory,geometryByTool:state.inertiaGeometryByTool,toolId:rec.site.owner,label,trace:rec.site.fraction===1?'left':'right'});
        const entry=incoming.get(rec.site.id);if(!entry||entry.owner!==rec.site.owner||entry.interpretation!=='actual-uniform-initial-material-rate')fail('New material contact needs actual initial or accepted angular-rate history');
        const L=state.coordinates[rec.site.edge+1]-state.coordinates[rec.site.edge],m=rec.map,rates=typeof m.dsDt==='number'?[m.dsDt,m.dsDt]:m.dsDt,
            oldStart=m.sStart-dt*rates[0],oldMetric=m.dsDx-dt*(rates[1]-rates[0])/L,fraction=(label-oldStart)/(oldMetric*L);
        if(!(oldMetric>0)||fraction<0||fraction>1)fail('Initial material angular-rate transport outside its own interval needs an explicit reservoir');
        const g=rec.geometry.edges[rec.site.edge],position=evaluateCompositeContinuousGeometry(g,{positions:g.nodeIndices.map(n=>state.toolPositions.get(rec.site.owner)[n]),fraction}).position;
        return {position,velocity:vec(entry.velocity,3,'Actual initial material velocity'),angularVelocity:vec(entry.angularVelocity,3,'Actual initial angular velocity')};
    }
    function surface(rec,toolPositions,order) {
        const p=rec.sample.point;
        if(!rec.surface){if(!rec.seed){let k=0;for(let j=1;j<3;j++)if(Math.abs(p.normal[j])<Math.abs(p.normal[k]))k=j;rec.seed=[0,0,0];rec.seed[k]=1;}
            rec.surface=createCompositeContinuousWallSurface({frame:rec.frame,site:p,seedAxis:rec.seed});}
        const out=evaluateCompositeContinuousWallSurface({positions:rec.frame.positionNodeIndices.map(n=>toolPositions.get(rec.site.owner)[n]),
            angles:rec.frame.angleEdgeIndices.map(e=>candidate.angles.get(rec.site.owner)[e]),materialMap:rec.map,dt,order},rec.surface);
        if(rec.mode===null) {
            const prior=rec.previous,c=rec.coefficients;
            if(c.muStatic[0]===c.muKinetic[0])rec.mode='static';
            else if(prior&&prior.label===out.materialLabel&&['static','kinetic'].includes(prior.nextMode))rec.mode=prior.nextMode;
            else {const incoming=incomingRate(rec,out.materialLabel),lever=out.contactPoint.map((v,k)=>v-incoming.position[k]),rotation=cross(incoming.angularVelocity,lever),
                v=incoming.velocity.map((x,k)=>x+rotation[k]),tangent=out.axes.map(a=>dot(a,v));rec.mode=tangent.every(v=>v===0)?'static':'kinetic';}
            rec.initialMode=rec.mode;
        }
        return out;
    }
    function prepare({toolPositions=state.toolPositions}={}){if(preparedReady)fail('Continuous wall friction is already prepared');normal.assertCurrentSurface({toolPositions});preparedReady=true;return {queries:0,samples:records.length};}
    function refresh({toolPositions,commonResidual,relativeResidual,order}) {
        if(!preparedReady)fail('Current normal sample preparation is required');normal.assertCurrentSurface({toolPositions});commitReady=false;last=null;cg.fill(0);rg.fill(0);
        for(const p of nodalForces.values())p.forEach(v=>v.fill(0));for(const t of spinTorques.values())t.fill(0);
        let converged=true,merit=0,lineSearchMerit=0;const proofs=[];
        for(const rec of records) {
            const Fn=normal.normalForces[rec.index],Ft=Array.from(tractions.slice(2*rec.index,2*rec.index+2)),p=rec.sample.point;
            if(!p.supported&&p.gap>0&&Fn===0) {
                // Strictly open original normal law fixes dFn=0. At its zero
                // cone, Ft=0 regardless of unknown surface motion; this exact
                // unloaded elimination introduces no normal or slip estimate.
                rec.rows.forEach((row,c)=>{row.residual=Ft[c]/penalty;row.multiplierDerivative=1/penalty;row.multiplierJacobian.fill(0);row.jacobian.fill(0);row.forceColumn.fill(0);row.geometricTangent.fill(0);row.geometricTangentValid=order==='full';});
                const eq=Math.max(...Ft.map(v=>Math.abs(v/penalty))),ok=Ft.every(v=>v===0);converged&&=ok;lineSearchMerit+=(eq/tolerances.frictionEquation)**2;merit+=(eq/tolerances.frictionEquation)**2;
                rec.latest={unloaded:true,traction:Ft,label:rec.map.sStart+rec.map.dsDx*(rec.site.coordinate-state.coordinates[rec.site.edge])};
                proofs.push({sampleId:rec.site.id,Fn,traction:Ft,slip:null,unloaded:true,converged:ok});continue;
            }
            if(!p.supported)fail('Loaded friction needs the original current supported normal branch');
            const s=surface(rec,toolPositions,order),slip=Array.from(s.slipIncrement),mu=rec.coefficients[rec.mode==='static'?'muStatic':'muKinetic'],
                eq=evaluateCompositeFrictionEquation({traction:Ft,slip,normalForce:Fn,mu,penalty},equation),physical=Fn>=0?measureCompositeFriction({traction:Ft,slip,normalForce:Fn,mu,
                    slipTolerance:tolerances.frictionSlip,coneTolerance:tolerances.frictionCone,workTolerance:tolerances.frictionWork}):{converged:false,workGap:Infinity,slipResidual:Infinity,coneViolation:Infinity},
                eqError=Math.max(...eq.residual.map(Math.abs)),ok=Fn>=0&&physical.converged&&eqError<=tolerances.frictionEquation,
                {terms,size,N,columns}=rec.mapped,B=new Float64Array(2*size),D=order==='full'?new Float64Array(2*size*size):null,J=order==='full'?new Float64Array(2*size):null;
            for(let j=0;j<size;j++)for(let c=0;c<2;c++) {
                for(const [a,w] of terms[j])B[2*j+c]+=w*s.forceMap[2*a+c];
                if(order==='full') {
                    for(const [a,w] of terms[j])J[c*size+j]+=w*s.slipDerivative[c*N+a];
                    for(let l=0;l<size;l++)for(const [a,wa] of terms[j])for(const [b,wb] of terms[l])D[(2*j+c)*size+l]+=wa*s.configurationDerivative[(2*a+c)*N+b]*wb;
                }
            }
            rec.rows.forEach((row,c)=>{
                row.residual=eq.residual[c];row.multiplierDerivative=eq.tractionJacobian[2*c+c];row.multiplierJacobian[0]=eq.normalDerivative[c];row.multiplierJacobian[1]=eq.tractionJacobian[2*c+1-c];
                for(let j=0;j<size;j++){row.forceColumn[j]=-B[2*j+c];row.jacobian[j]=order==='full'?eq.slipJacobian[2*c]*J[j]+eq.slipJacobian[2*c+1]*J[size+j]:NaN;
                    if(order==='full')for(let l=0;l<size;l++)row.geometricTangent[j*size+l]=-Ft[c]*D[(2*j+c)*size+l];}
                row.geometricTangentValid=order==='full';
            });
            rec.mapped.commonDofs.forEach((d,j)=>cg[d]-=B[2*j]*Ft[0]+B[2*j+1]*Ft[1]);rec.mapped.relativeDofs.forEach((d,j)=>{const i=rec.mapped.commonDofs.length+j;rg[d]-=B[2*i]*Ft[0]+B[2*i+1]*Ft[1];});
            columns.forEach((c,j)=>{const force=s.forceMap[2*j]*Ft[0]+s.forceMap[2*j+1]*Ft[1];if(c.kind==='position')nodalForces.get(c.toolId)[c.node][c.component]+=force;else spinTorques.get(c.toolId)[c.edge]+=force;});
            const eqMerit=Array.from(eq.residual).reduce((sum,v)=>sum+(v/tolerances.frictionEquation)**2,0);lineSearchMerit+=eqMerit;
            merit+=Fn>=0?eqMerit+(physical.workGap/tolerances.frictionWork)**2+(physical.slipResidual/tolerances.frictionSlip)**2+(physical.coneViolation/tolerances.frictionCone)**2:Infinity;converged&&=ok;
            rec.latest={traction:Ft,slip,normalForce:Fn,label:s.materialLabel};rec.decision=null;
            proofs.push({sampleId:rec.site.id,owner:rec.site.owner,edge:rec.site.edge,fraction:rec.site.fraction,Fn,traction:Ft,slip,mode:rec.mode,mu:Array.from(mu),
                muStatic:rec.coefficients.muStatic.slice(),muKinetic:rec.coefficients.muKinetic.slice(),equationResidual:Array.from(eq.residual),...physical,converged:ok,finiteStepSlipKnown:false,slipModel:policy.slipModel});
        }
        if(!Number.isFinite(lineSearchMerit)||!cg.every(Number.isFinite)||!rg.every(Number.isFinite))fail('Nonfinite C2 surface friction residual');
        cg.forEach((v,i)=>commonResidual[i]+=v);rg.forEach((v,i)=>relativeResidual[i]+=v);commitReady=converged;last={toolPositions,values:currentValues(toolPositions),tractions:tractions.slice(),normal:normal.normalForces.slice()};
        return {converged,merit,lineSearchMerit,samples:proofs,sampleCount:records.length,scope:'declared-C2-sample-implicit-rate-Coulomb',finiteStepSlipKnown:false,slipModel:policy.slipModel};
    }
    function fresh(){return commitReady&&last&&same(currentValues(last.toolPositions),last.values)&&same(Array.from(tractions),Array.from(last.tractions))&&same(Array.from(normal.normalForces),Array.from(last.normal));}
    function resolveModes({wholeStepConverged}) {
        if(!wholeStepConverged||!fresh())return {accepted:false,changed:false,status:'unconverged',samples:[]};
        let changed=false;const decisions=records.map(rec=>{
            if(rec.latest.unloaded){rec.decision={accepted:true,nextMode:null,stopCertificate:null};return {sampleId:rec.site.id,accepted:true,status:'unloaded'};}
            let decision=assessCompositeStaticKineticFriction({...rec.latest,...rec.coefficients,mode:rec.mode,wholeStepConverged,
                slipTolerance:tolerances.frictionSlip,coneTolerance:tolerances.frictionCone,workTolerance:tolerances.frictionWork});
            // A previously static sample remains an admissible static solution
            // on the cone boundary. Use the declared numerical slip tolerance
            // there too; requiring bitwise zero would reject a fully converged
            // implicit step solely because redundant pressures choose a boundary
            // traction. This does not infer a stop of a kinetic contact.
            if(decision.status==='ambiguous'&&rec.mode==='static'&&decision.physical.converged&&
                Math.hypot(...rec.latest.slip.filter((_,i)=>rec.coefficients.muStatic[i]>0))<=tolerances.frictionSlip)
                decision={...decision,status:'static-within-residual-tolerance',accepted:true,nextMode:'static',stopCertificate:'static-cone-and-declared-slip-residual'};
            rec.decision=decision;if(decision.change){if(rec.demotions||rec.mode!=='static')fail('One static-to-kinetic transition per prepared dt');rec.mode='kinetic';rec.demotions++;rec.decision=null;changed=true;}
            return {sampleId:rec.site.id,initialMode:rec.initialMode,...decision};
        });
        if(changed)commitReady=false;return {accepted:!changed&&decisions.every(d=>d.accepted),changed,status:changed?'breakaway':decisions.every(d=>d.accepted)?'accepted':'ambiguous',samples:decisions};
    }
    function commit() {
        if(!fresh()||records.some(r=>!r.decision?.accepted))fail('C2 friction commit needs a fresh common-step and mode certificate');normal.assertCurrentSurface({toolPositions:last.toolPositions});
        return {signature,law:policy.law,slipModel:policy.slipModel,rateMode:policy.rateMode,finiteStepSlipKnown:false,tractions:tractions.slice(),
            records:records.map(r=>({id:r.site.id,traction:Array.from(tractions.slice(2*r.index,2*r.index+2)),label:r.latest.label,seedAxis:r.seed?.slice()??null,nextMode:r.decision.nextMode,stopCertificate:r.decision.stopCertificate}))};
    }
    return {rows,tractions,nodalForces,spinTorques,prepare,refresh,resolveModes,commit};
}
