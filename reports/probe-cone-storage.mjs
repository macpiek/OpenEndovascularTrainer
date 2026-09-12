import fs from 'node:fs';
import {createHash} from 'node:crypto';
import { pathToFileURL } from 'node:url';
const config = JSON.parse(fs.readFileSync('reports/cone-storage-source.json', 'utf8'));
const source = path => pathToFileURL(config.runtime + '/' + path);
const { createCoupledRuntimeFixture } = await import(source('tests/helpers/coupledRuntimeFixture.js'));
const { createCoupledSolverSelection } = await import(source('src/physics/coupledSolverSelection.js'));
const { configureKirchhoffSplitBias } = await import(source('src/physics/kirchhoffSplitMotion.js'));
const { solveKirchhoffCoupledSystem, applyKirchhoffCoupledCorrection } = await import(source('src/physics/kirchhoffCoupledSystem.js'));
const { evaluateKirchhoffSurfaceFrictionKKT } = await import(source('src/physics/kirchhoffSurfaceFriction.js'));
const selection = createCoupledSolverSelection('joint-active-coulomb', { solve: solveKirchhoffCoupledSystem, apply: applyKirchhoffCoupledCorrection });
const f = createCoupledRuntimeFixture({ coupledSystem: selection.coupledSystem, jointMotionMode: 'split-physical-bias' });
configureKirchhoffSplitBias(f.containment, { materialMode: 'coupled-compliance' });
const dot = (a,b) => a.reduce((s,v,i)=>s+v*b[i],0);
const cone = (lambda, normal, mu) => evaluateKirchhoffSurfaceFrictionKKT(lambda,[0,0],normal,mu).coneViolation;
const bank = (kind,e) => kind==='wall' && e.normalRow.kind==='wall' ? e.normalRow.owner.wallLambda : kind==='external' ? e.owner.lambdas : null;
const key = (kind,e) => kind==='wall'?e.contact.key:kind==='external'?`external:${e.index}:${e.segmentA}:${e.segmentB}`:`lumen:${e.contact.id}:${e.record.kind}`;
const trials = []; let active;
const requestedTolerance = process.env.OET_CONE_LINEAR_TOLERANCE ? Number(process.env.OET_CONE_LINEAR_TOLERANCE) : null;
const label = process.env.OET_CONE_OUTPUT_LABEL ?? 'cone-storage';
let seed = null;
function values(body) { return Object.fromEntries(Object.entries(body).filter(([,v])=>ArrayBuffer.isView(v)).map(([k,v])=>[k,[...v]])); }
f.world.debugConeSolve=(joint,options,pass)=>{
    if (f.world.stepCount !== 49 || pass < 5) return;
    if (!seed) {
        const data={step:f.world.stepCount,pass,bodies:[joint.innerBody,joint.outerBody].map(values),motion:joint._splitMotion.bodies.map(values),
            physicalPose:joint._splitMotion.twoChannel.physicalPose.map(values),
            contacts:joint.kirchhoffContacts.map(r=>({id:r.manifoldContact?.id,normal:[...r.normal],normalLambda:r.manifoldContact?.normalLambda,
                tangentLambda:r.manifoldContact?[...r.manifoldContact.tangentLambda]:null,inner:[...r.innerWeights],outer:[...r.outerWeights],
                gap:r.gap,startGap:r._splitStartGap,actualGap:r._splitActualGap,normalGradients:r.normalGradients})),
            rows:options.additionalRows.map(r=>({kind:r.kind,strain:r.strain,alpha:r.alpha,lambda:r.lambda,lower:r.lower,upper:r.upper,gradients:r.gradients}))};
        const serialized=JSON.stringify(data);seed={sha256:createHash('sha256').update(serialized).digest('hex'),step:49,pass:5};
        fs.writeFileSync('reports/'+label+'-seed.json',serialized+'\n');
    }
    if(requestedTolerance)options.tolerance=requestedTolerance;
    if(process.env.OET_CONE_CAPTURE_SYSTEM)options.includeSystem=true;
};
f.world.debugConeLinear=(joint,result,options,pass)=>{
    if(!result.system || f.world.stepCount!==49 || pass!==5)return;
    const system=result.system,native=system.native,zeros=[];
    for(let original=0;original<native.count;original++){
        const sorted=native.order.indexOf(original),physicalRow=system.physicalRows[sorted];
        let norm=0,terms=0;
        for(const columns of native.columns)for(const column of columns)for(let k=0;k<column.length;k+=2)if(column[k]===sorted){norm+=Math.abs(column[k+1]);terms++;}
        if(norm===0 && system.alpha[physicalRow]===0 && system.rhs[physicalRow]!==0)
            zeros.push({original,sorted,physicalRow,kind:native.rows[original].kind,side:native.rows[original].side,node:native.rows[original].node,
                alpha:system.alpha[physicalRow],rhs:system.rhs[physicalRow],gradientTerms:terms,gradientOneNorm:norm,
                lower:system.lower[physicalRow],upper:system.upper[physicalRow]});
    }
    fs.writeFileSync('reports/'+label+'-zero-rows.json',JSON.stringify({seed,tolerance:options.tolerance,diagnostics:result.diagnostics,zeros},null,2)+'\n');
};
function capture(kind, e) {
    const world = [0,1,2].map(i=>e.contact.tangentLambda[0]*e.contact.tangentU[i]+e.contact.tangentLambda[1]*e.contact.tangentV[i]);
    return { key:key(kind,e), kind, normalStorage:bank(kind,e)?.constructor.name ?? 'Number', tangentStorage:e.contact.tangentLambda.constructor.name,
        storedNormal:e.contact.normalLambda, storedTangent:[...e.contact.tangentLambda], axes:e.surface.axes.map(a=>[...a]),
        surfaceLambda:e.surface.axes.map(axis=>dot(axis,world)), mu:[...e.surface.group.mu], normalKind:e.normalRow?.kind,
        body:e.normalRow?.owner?.id ?? e.owner?.id, node:e.normalRow?.node ?? e.index,
        normal:[...e.record.normal], point:[...e.surface.point], weights:[...e.record.innerWeights] };
}
f.world.debugConeFrozen=(joint,batches,result,pass,trial)=>{
    active={step:f.world.stepCount,pass,trial,linearTolerance:joint._jointOptions.tolerance,scale:result.scale,linear:structuredClone(result.diagnostics),contacts:[]};
    for(const [kind,batch] of Object.entries(batches)) for(const e of batch?.entries??[]) {
        const item=capture(kind,e);
        const index=kind==='lumen'?joint.kirchhoffContacts.indexOf(e.record):joint._coupledBoundaries.rows.indexOf(e.normalRow);
        const dn=kind==='lumen'?result.contactIncrement[index]:result.additionalIncrement[index];
        item.baseNormal=e.contact.normalLambda;item.normalRowLambda=e.normalRow?.lambda??item.baseNormal;item.normalIndex=index;
        item.normalIncrement=result.scale*dn;
        item.frozenNormal=Math.max(0,item.normalRowLambda+item.normalIncrement);
        item.frozenTangent=e.surface.rows.map((row,a)=>row.lambda+result.scale*result.additionalIncrement[batch.rowOffset+e.rowStart+a]);
        item.frozenCone=cone(item.frozenTangent,item.frozenNormal,item.mu);
        item.float32Normal=Math.fround(item.frozenNormal);
        item.float32OnlyCone=cone(item.frozenTangent,item.float32Normal,item.mu);
        active.contacts.push(item);
    }
    trials.push(active);
};
f.world.debugConeCommitted=(joint,batches)=>{
    for(const [kind,batch] of Object.entries(batches)) for(const e of batch?.entries??[]) {
        const item=active.contacts.find(c=>c.key===key(kind,e)), current=capture(kind,e);
        if(!item)throw Error('missing frozen record');
        item.committed=current;
        item.committedCone=cone(current.surfaceLambda,current.storedNormal,current.mu);
    }
};
f.world.debugJointTrial=(joint,state)=>{
    active.fresh={settled:state.settled,merit:state.merit};
    const batches={lumen:state.frictionResidual,external:state.externalFrictionResidual,wall:state.wallPhysicalFriction};
    for(const [kind,residual]of Object.entries(batches)) {
        active.fresh[kind]=residual?.maximumConeViolation??0;
        for(const e of residual?._batch?.entries??[]){
            const item=active.contacts.find(c=>c.key===key(kind,e)),current=capture(kind,e);
            current.cone=cone(current.surfaceLambda,current.storedNormal,current.mu);
            if(item)item.fresh=current;
            else(active.newFresh??=[]).push(current);
        }
    }
};
let result,error=null,accepted=0;
try{
 for(let i=0;i<33;i++)f.step({guidewireAdvance:1});
 for(let i=0;i<40;i++){result=f.step({catheterAdvance:1});if(result?.accepted===false)break;accepted++;if(requestedTolerance&&f.world.stepCount>49)break;}
}catch(e){error={message:e.message,stack:e.stack};result=f.world.lastStepResult;}
const output={state:f.snapshot(),accepted,result,error,seed,requestedTolerance,finalGates:{containmentMm:f.world.coupledContainmentTolerance,angularRad:f.world.coupledAngularToleranceRad,length:f.world.coupledLengthTolerance},diagnostics:f.world.getStats().jointMotion,trials};
fs.writeFileSync('reports/'+label+'-replay.json',JSON.stringify(output,null,2));
const worst=trials.flatMap(t=>t.contacts.map(c=>({step:t.step,pass:t.pass,trial:t.trial,scale:t.scale,...c})))
 .filter(c=>c.fresh).sort((a,b)=>b.fresh.cone-a.fresh.cone);
console.log(JSON.stringify({state:output.state,accepted,error,seed,requestedTolerance,trialCount:trials.length,worst:worst.slice(0,4)},null,2));
f.dispose();
