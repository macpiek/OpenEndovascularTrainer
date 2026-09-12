import {compositeJointFrictionAdditionCompatible} from './kirchhoffCompositeJointContactHistory.js';
import {evaluateKirchhoffLumenSegmentContact} from './kirchhoffLumenContact.js';
import {createCompositeLumenSideGeometryWorkspace,differentiateCompositeLumenSideContact} from './kirchhoffCompositeLumenSideGeometry.js';
import {createCompositeJointLumenSurfaceWorkspace,evaluateCompositeJointLumenSurface} from './kirchhoffCompositeJointLumenSurface.js';
import {createCompositeJointLumenRateSurfaceWorkspace,evaluateCompositeJointLumenRateSurface} from './kirchhoffCompositeJointLumenRateSurface.js';
import {createCompositeJointSurfacePullbackFactory,pullbackCompositeJointSurface,evaluateCompositeJointSurfaceLoads} from './kirchhoffCompositeJointSurfacePullback.js';
import {createCompositeFrictionEquationWorkspace,evaluateCompositeFrictionEquation,measureCompositeFriction} from './kirchhoffCompositeFriction.js';

const workspaces=new WeakMap();
const failure=message=>{const e=new RangeError(message);e.code='unsupported-joint-lumen-friction';return e;};
const finite=(v,name)=>{if(!Number.isFinite(v))throw failure(`${name} must be finite`);return v;};
const positive=(v,name)=>{if(!(finite(v,name)>0))throw failure(`${name} must be positive`);return v;};
const vector=(v,n,name)=>{if(v?.length!==n||!Array.from(v).every(Number.isFinite))throw failure(`${name} needs ${n} finite entries`);return Array.from(v);};
const same=(a,b)=>a?.length===b.length&&b.every((v,i)=>v===a[i]);
const copy=v=>structuredClone(v);
const mapKey=t=>JSON.stringify([t.id,t.edge,t.edgeId]);
const edgeId=v=>{
    if(typeof v==='string')return `string:${v.length}:${v}`;
    if(typeof v==='number'&&Number.isFinite(v))return `number:${Object.is(v,-0)?0:v}`;
    if(typeof v==='bigint')return `bigint:${v}`;
    throw failure('A semantic material segment ID must be a string, finite number or bigint');
};
const trace=f=>f===0?'right':f===1?'left':undefined;
const roundoffEqual=(a,b,scale)=>Math.abs(a-b)<=64*Number.EPSILON*(Math.abs(a)+Math.abs(b)+scale);
const dot3=(a,b)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
const chordUnit=p=>{
    const d=p[1].map((v,k)=>v-p[0][k]),length=Math.hypot(...d);
    return {length,tangent:d.map(v=>v/length)};
};

// Sufficient eligibility bounds, NOT physical tolerances. Outside this
// regular same-edge chart the unchanged full surface provider decides whether
// the candidate is supported. These bounds stay far from its PT/phase/radial
// singularities and numerical overflow, without reconstructing any slip or B.
function regularOpenHistory(rec,currentFrames,dt) {
    if(!(dt>=1e-6&&dt<=1)||!rec.previousRegular)return false;
    const g=rec.sample.geometry,old=rec.previousGeometry;
    if(!(rec.baseInput.innerRadius>0&&rec.baseInput.innerRadius<=1e4&&rec.baseInput.lumenRadius>rec.baseInput.innerRadius&&rec.baseInput.lumenRadius<=1e4))return false;
    if(g.rawContact.id!==rec.previous.contact.id||g.rawContact.feature!==rec.previous.contact.feature||
        !(g.innerT>0&&g.innerT<1&&g.outerT>0&&g.outerT<1)||dot3(g.normal,old.normal)<.99)return false;
    for(let i=0;i<2;i++) {
        const t=rec.tools[i],frame=currentFrames.get(t);
        if(!frame.regular)return false;
        if(i===0&&!(1-dot3(g.normal,frame.tangent)**2>.01))return false;
        const dx=t.coordinates[1]-t.coordinates[0],fraction=i===0?g.innerT:g.outerT;
        if(!(dx>=1e-4&&dx<=1e4)||![t.currentMap.dsDx,t.previousMap.dsDx].every(v=>v>=1e-4&&v<=1e4)||
            Math.abs(t.currentMap.sStart)>1e4||Math.abs(t.previousMap.sStart)>1e4||t.rates.some(v=>Math.abs(v)>1e6))return false;
        // Use the same coordinate->fraction arithmetic as the provider; an
        // endpoint/hinge trace or a cancellation-sensitive rate goes to it.
        const coordinate=t.coordinates[0]+dx*fraction,f=(coordinate-t.coordinates[0])/dx,
            label=t.currentMap.sStart+t.currentMap.dsDx*dx*f,
            previousFraction=(label-t.previousMap.sStart)/(t.previousMap.dsDx*dx),
            rate=(1-fraction)*t.rates[0]+fraction*t.rates[1],
            implied=((t.currentMap.sStart-t.previousMap.sStart)+(t.currentMap.dsDx-t.previousMap.dsDx)*dx*f)/dt,
            bound=64*Number.EPSILON*(Math.abs(t.currentMap.sStart)+Math.abs(t.previousMap.sStart)+
                (Math.abs(t.currentMap.dsDx)+Math.abs(t.previousMap.dsDx))*dx*Math.abs(f))/dt;
        if(!(f>0&&f<1&&previousFraction>0&&previousFraction<1)||!Number.isFinite(implied)||Math.abs(rate-implied)>bound)return false;
    }
    return true;
}

/** ONE heavy provider arena and one equation scratch for sequential contacts
 * and dt calls. This handle owns no Fn/Ft/history or certificate authority.
 * A newer manager lease invalidates an older manager using this workspace.
 */
export function createCompositeJointLumenFrictionWorkspace() {
    const scratch={surface:createCompositeJointLumenSurfaceWorkspace(),equation:createCompositeFrictionEquationWorkspace(),
        previousGeometry:createCompositeLumenSideGeometryWorkspace(),
        generation:0,busy:false,refreshes:0,surfaceEvaluations:0,zeroConeEliminations:0,chartBuilds:0,chartHits:0,createPullback:null};
    const handle=Object.freeze({scope:'sequential-joint-lumen-friction-scratch',get diagnostics(){return {
        surfaceWorkspaceBuilds:1,equationWorkspaceBuilds:1,previousGeometryWorkspaceBuilds:1,managers:scratch.generation,refreshes:scratch.refreshes,
        surfaceEvaluations:scratch.surfaceEvaluations,zeroConeEliminations:scratch.zeroConeEliminations,chartBuilds:scratch.chartBuilds,chartHits:scratch.chartHits,
        support:scratch.createPullback?.diagnostics??null};}});
    workspaces.set(handle,scratch);return handle;
}

/** Strict-side finite Coulomb rows without an interior normal-force gauge.
 * Current detector data is borrowed ONLY immediately after normal.refresh;
 * prepare makes one original previous-state query per declared sample.
 * Numerical friction k may change; physical mu/chart/path cannot silently
 * reinterpret accepted history. Ft and signed private Fn are never clipped.
 * The source state owns old geometry/frames/spins; candidate supplies live
 * current angles. This manager neither integrates nor commits either body.
 */
export function createCompositeJointLumenFrictionRows({state,candidate,prepared,normal,contacts,dt,tolerances,
    normalRowOffset,frictionRowOffset,workspace=createCompositeJointLumenFrictionWorkspace()}) {
    const scratch=workspaces.get(workspace);if(!scratch||scratch.busy)throw failure('A free prepared friction workspace is required');
    const generation=++scratch.generation;
    positive(dt,'dt');if(prepared?.dt!==undefined&&prepared.dt!==dt)throw failure('Prepared friction dt differs from the requested step');
    const policy=contacts?.friction;
    if(contacts?.mode!=='lumen-coulomb'||policy?.law!=='coulomb'||policy.materialPath!=='linear-affine-maps')
        throw failure('Explicit lumen-coulomb, Coulomb law and linear-affine-maps path are required');
    const implicitRate=policy.rateMode==='backward-euler-grid';
    if(policy.rateMode!==undefined&&(!implicitRate||policy.slipModel!=='implicit-backward-euler-surface-rate'||policy.finiteStepSlipKnown!==false))
        throw failure('An explicit consistent native pair surface-rate law is required');
    if(implicitRate&&!scratch.rateSurface)scratch.rateSurface=createCompositeJointLumenRateSurfaceWorkspace();
    const mu=vector(policy.mu,2,'Coulomb coefficients');if(mu.some(v=>v<0))throw failure('Coulomb coefficients must be nonnegative');
    const muByPair=policy.muByPair===undefined?null:copy(policy.muByPair);
    if(muByPair&&(typeof muByPair!=='object'||Array.isArray(muByPair)||Object.entries(muByPair).some(([id,v])=>!id||vector(v,2,'Pair Coulomb coefficients').some(x=>x<0))))throw failure('Pair Coulomb coefficients need an explicit nonnegative pair map');
    const penalty=positive(policy.forcePerLength,'Friction forcePerLength'),normalSignature=normal?.signature;
    if(typeof normalSignature!=='string'||!Array.isArray(normal.samples)||!normal.samples.length||!Array.isArray(normal.rowSampleIndices))
        throw failure('The actual prepared normal sample owner is required');
    if(!implicitRate&&normal.samples.some(s=>s.featurePolicy))throw failure('Native side-fillet transitions require the explicit implicit surface-rate law');
    const {layout}=state,n=layout.nodeCount,r=3*state.modes.length;
    if(candidate.relative?.length!==r)throw failure('Candidate relative coordinates must match the full fixed chart');
    if(!(state.toolPositions instanceof Map)||!(candidate.angles instanceof Map)||prepared?.inertiaEdges?.length!==n-1||!(prepared.previousPositions instanceof Map))
        throw failure('Own incoming geometry, live angles and prepared physical inertia are required');
    const tol=Object.fromEntries(['frictionSlip','frictionCone','frictionWork','frictionEquation','linearConstraint'].map(k=>[k,positive(tolerances?.[k],k)]));
    const retained=normal.rowSampleIndices.slice(),sampleIds=normal.samples.map(s=>s.sampleId),retainedIndex=new Map(retained.map((s,i)=>[s,i]));
    // Exact transfer of NORMAL force/wrench does not preserve local Coulomb
    // power or tangential wrench when slip changes along the physical edge.
    // Keep every declared sample load, never silently reduce its pressure.
    if(normal.gauge?.redundantSamples>0||normal.gauge?.transfers?.length>0||retained.length!==normal.samples.length)
        throw failure('Coulomb friction does not support the interior normal-force endpoint gauge');
    if(new Set(retained).size!==retained.length||retained.some(i=>!Number.isInteger(i)||i<0||i>=normal.samples.length)||
        normal.rows.length!==retained.length||normal.samples.some((s,i)=>s.index!==i||!(implicitRate?['side','distal-fillet','distal-rim','external-capsule']:['side']).includes(s.feature)||s.redundant===retainedIndex.has(i)))
        throw failure('Strict side samples and unreduced normal reactions are required; tip features cannot be omitted');
    for(const [name,value] of Object.entries({normalRowOffset,frictionRowOffset}))if(!Number.isInteger(value)||value<0)throw failure(`${name} must be a nonnegative global row index`);
    if(frictionRowOffset<normalRowOffset+retained.length&&frictionRowOffset+2*retained.length>normalRowOffset)throw failure('Normal and friction global row intervals overlap');
    const signature=JSON.stringify({normal:normalSignature,mu,muByPair,law:'coulomb',materialPath:policy.materialPath,
        rateMode:policy.rateMode,slipModel:policy.slipModel,
        surface:implicitRate?'native-common-inner-capsule-point':'strict-side-virtual-midpoint-witness',gauge:'no-interior-normal-reduction'});
    const history=state.lumenFrictionState===undefined?null:copy(state.lumenFrictionState);
    const historyAddition=implicitRate&&contacts.historyUpdate==='preserve-and-append'&&history&&normal.historyAddition&&
        same(history.sampleIds,normal.historyAddition.previousSampleIds)&&history.tractions?.length===2*history.sampleIds.length&&
        compositeJointFrictionAdditionCompatible({previousSignature:history.signature,currentSignature:signature,normalAddition:normal.historyAddition})?normal.historyAddition:null;
    if(history&&!historyAddition&&(history.signature!==signature||!same(history.sampleIds,sampleIds)||history.tractions?.length!==2*sampleIds.length))
        throw failure('Friction chart/coefficient/path/sample history changed without traction transfer');
    const tractions=historyAddition?Float64Array.from(historyAddition.sourceIndices.flatMap(i=>i<0?[0,0]:vector(history.tractions.slice(2*i,2*i+2),2,'Accepted tangential history'))):history?Float64Array.from(vector(history.tractions,2*sampleIds.length,'Accepted tangential history')):new Float64Array(2*sampleIds.length);
    const oldPositions=new Map([...layout.spins.keys()].map(id=>{
        const p=state.toolPositions.get(id);if(p?.length!==n)throw failure('Missing own incoming physical geometry');
        const owned=p.map(v=>vector(v,3,'Incoming physical position')),previous=prepared.previousPositions.get(id);
        if(previous?.length!==n||owned.some((v,i)=>!same(previous[i],v)))throw failure('Prepared previous geometry must equal the incoming physical state');
        return [id,owned];
    }));
    const byTool=new Map(state.tools.map(t=>[t.id,t])),edges=new Map();
    const retainedHistoryEdges=new Set(historyAddition?normal.samples.flatMap((sample,i)=>historyAddition.sourceIndices[i]<0?[]:
        [sample.points[0],sample.points[2]].map(p=>JSON.stringify([p.toolId,p.node]))):[]);
    const oldMaps=new Map();
    if(history) {
        if(!Array.isArray(history.currentMaps))throw failure('Accepted friction history needs its own current material maps');
        for(const m of history.currentMaps){const key=mapKey(m);if(oldMaps.has(key))throw failure('Duplicate accepted material map');oldMaps.set(key,m.currentMap);}
    }
    function ownEdge(id,edge,semanticId) {
        const idString=edgeId(semanticId),key=JSON.stringify([id,edge]);
        if(edges.has(key)){const t=edges.get(key);if(t.edgeId!==idString)throw failure('One actual material edge has inconsistent semantic identities');return t;}
        if(!layout.edgeToolIds[edge]?.includes(id)||!(layout.spins.get(id)?.[edge]>=0))throw failure('A friction surface needs its actual edge and own spin');
        const matches=prepared.inertiaEdges[edge]?.tools?.filter(t=>t.id===id);if(matches?.length!==1)throw failure('Each surface needs exactly one prepared material map');
        const map=matches[0].materialMap,dx=positive(state.coordinates[edge+1]-state.coordinates[edge],'Own coordinate span');
        const sStart=finite(map?.sStart,'Current material start'),dsDx=positive(map?.dsDx,'Current dsDx');
        const rates=typeof map.dsDt==='number'?[finite(map.dsDt,'dsDt'),map.dsDt]:vector(map.dsDt,2,'Material endpoint rates');
        const startChange=finite(dt*rates[0],'Material start increment'),slopeChange=finite(dt*(rates[1]-rates[0])/dx,'Material slope increment');
        const previousMap={sStart:finite(sStart-startChange,'Old material start'),dsDx:positive(dsDx-slopeChange,'Old material slope')};
        const tool=byTool.get(id),frame=tool?.reference?.[edge],oldAngle=state.angles.get(id)?.[edge];
        const result={id,edge,edgeId:idString,materialSegmentId:semanticId,coordinates:[state.coordinates[edge],state.coordinates[edge+1]],rates,currentMap:{sStart,dsDx},previousMap,
            previousPositions:[oldPositions.get(id)[edge],oldPositions.get(id)[edge+1]],
            reference:{tangent:vector(frame?.tangent,3,'Own old tangent'),director:vector(frame?.director,3,'Own old director')},previousAngle:finite(oldAngle,'Own old angle')};
        const prior=oldMaps.get(mapKey(result));
        if(history&&!(historyAddition&&!prior&&!retainedHistoryEdges.has(key))&&(!prior||!Number.isFinite(prior.sStart)||!Number.isFinite(prior.dsDx)||
            !roundoffEqual(previousMap.sStart,prior.sStart,Math.abs(sStart)+Math.abs(startChange))||
            !roundoffEqual(previousMap.dsDx,prior.dsDx,Math.abs(dsDx)+Math.abs(slopeChange))))throw failure('Derived old material map disagrees with accepted friction history');
        edges.set(key,result);return result;
    }
    const chart={layout,modes:state.modes,relativeToolId:state.relativeToolId,supportPolicy:normal.samples.some(s=>s.feature==='external-capsule')?'declared-tool-pair':'local-two-edge'};
    if(scratch.createPullback?.matches(chart))scratch.chartHits++;
    else {scratch.createPullback=createCompositeJointSurfacePullbackFactory(chart);scratch.chartBuilds++;}
    const createPullback=scratch.createPullback;
    const rows=[],records=normal.samples.map((sample,i)=>{
        if(sample.points?.length!==4)throw failure('An actual inner/outer affine surface pair is required');
        const a=sample.points[0],b=sample.points[2];
        if(sample.points[1].toolId!==a.toolId||sample.points[1].node!==a.node+1||sample.points[3].toolId!==b.toolId||sample.points[3].node!==b.node+1)
            throw failure('Surface points must identify two actual affine edges');
        const tools=[ownEdge(a.toolId,a.node,sample.input.innerMaterialSegmentId),ownEdge(b.toolId,b.node,sample.input.outerMaterialSegmentId)];
        const baseInput=copy({...sample.input,innerStart:null,innerEnd:null,outerStart:null,outerEnd:null});
        const index=retainedIndex.get(i),mapping=index===undefined?null:createPullback(tools);
        const localRows=mapping?[0,1].map(component=>{
            const m=mapping.rows[component],size=mapping.dofCount,row={toolId:'lumen-friction',index:2*i+component,tractionIndex:2*i+component,sampleIndex:i,component,
                anchorNode:sample.row.anchorNode,commonDofs:m.commonDofs,relativeDofs:m.relativeDofs,unit:'mm',
                multiplierDofs:Int32Array.of(normalRowOffset+index,frictionRowOffset+2*index+1-component),multiplierJacobian:new Float64Array(2).fill(NaN),
                jacobian:new Float64Array(size).fill(NaN),forceColumn:new Float64Array(size).fill(NaN),geometricTangent:new Float64Array(size*size).fill(NaN),
                geometricTangentValid:false,residual:NaN,multiplierDerivative:NaN,tolerance:tol.linearConstraint};
            if(sample.feature==='external-capsule')row.constraintSupport={kind:'native-tool-pair',pairId:sample.pairId,tools:tools.map(t=>({id:t.id,edge:t.edge})),spins:true};
            rows.push(row);return row;
        }):[];
        const coefficients=muByPair?muByPair[sample.pairId]:mu;
        if(!coefficients)throw failure('Each original normal pair needs its own Coulomb coefficients');
        return {sample,index:i,mu:vector(coefficients,2,'Pair Coulomb coefficients'),tools,baseInput,mapping,rows:localRows,previous:null};
    });
    if(history&&(historyAddition?Array.from(oldMaps.keys()).some(key=>!Array.from(edges.values()).some(t=>mapKey(t)===key)):oldMaps.size!==edges.size))throw failure('Accepted friction maps contain changed physical support');
    function gauge() {for(const rec of records)if(rec.mapping===null&&(tractions[2*rec.index]!==0||tractions[2*rec.index+1]!==0||normal.normalForces[rec.index]!==0))
        throw failure('Interior normal/Ft must remain exactly zero in the declared endpoint gauge');}
    gauge();
    const nodalForces=new Map([...layout.spins.keys()].map(id=>[id,Array.from({length:n},()=>[0,0,0])])),spinTorques=new Map([...layout.spins.keys()].map(id=>[id,new Float64Array(n-1)]));
    const commonScratch=new Float64Array(layout.dofCount),relativeScratch=new Float64Array(r),sampleTraction=new Float64Array(2);
    let preparedReady=false,commitReady=false,last=null,preparationQueries=0,refreshes=0,surfaceEvaluations=0,zeroConeEliminations=0;
    function invalidate() {
        commitReady=false;last=null;commonScratch.fill(0);relativeScratch.fill(0);
        for(const p of nodalForces.values())p.forEach(v=>v.fill(0));for(const v of spinTorques.values())v.fill(0);
        for(const row of rows){row.knownZeroCone=false;row.geometricTangentValid=false;row.residual=row.multiplierDerivative=NaN;row.jacobian.fill(NaN);row.forceColumn.fill(NaN);row.geometricTangent.fill(NaN);row.multiplierJacobian.fill(NaN);}
    }
    function verify() {
        if(scratch.generation!==generation||scratch.busy)throw failure('Stale or busy prepared lumen friction manager');
        if(normal.signature!==normalSignature||normal.samples.length!==records.length||!same(normal.rowSampleIndices,retained)||normal.samples.some((s,i)=>s!==records[i]?.sample||s.sampleId!==sampleIds[i]))
            throw failure('Normal sample owner/gauge changed during a prepared step');
        if(normal.gauge?.redundantSamples>0||normal.gauge?.transfers?.length>0)throw failure('Coulomb friction cannot inherit an interior pressure transfer');
        if(contacts.mode!=='lumen-coulomb'||contacts.friction!==policy||policy.law!=='coulomb'||policy.materialPath!=='linear-affine-maps'||
            policy.forcePerLength!==penalty||!same(policy.mu,mu)||JSON.stringify(policy.muByPair??null)!==JSON.stringify(muByPair)||implicitRate&&(policy.rateMode!=='backward-euler-grid'||policy.slipModel!=='implicit-backward-euler-surface-rate'||policy.finiteStepSlipKnown!==false))throw failure('Frozen friction policy changed during a prepared step');
        vector(tractions,2*records.length,'Private traction');vector(normal.normalForces,records.length,'Private normal forces');gauge();
    }
    const inputFor=(rec,p)=>({...rec.baseInput,innerStart:p.get(rec.tools[0].id)[rec.tools[0].edge],innerEnd:p.get(rec.tools[0].id)[rec.tools[0].edge+1],
        outerStart:p.get(rec.tools[1].id)[rec.tools[1].edge],outerEnd:p.get(rec.tools[1].id)[rec.tools[1].edge+1]});
    function prepare({consumeQuery}) {
        invalidate();verify();if(preparedReady)throw failure('Friction previous queries are already prepared');
        if(typeof consumeQuery!=='function')throw failure('Every original previous query requires a budget callback');scratch.busy=true;
        try {
            if(implicitRate){preparedReady=true;return {queries:0,samples:records.length,rateMode:policy.rateMode};}
            // This eligibility test depends only on an owned incoming edge,
            // not on its surface sample. The sample normal is checked below.
            const regularEdges=new Map(Array.from(edges.values(),t=>{
                const f=chordUnit(t.previousPositions),a=t.reference;
                return [t,f.length>=1e-4&&f.length<=1e4&&t.previousPositions.every(p=>p.every(v=>Math.abs(v)<=1e4))&&Math.abs(t.previousAngle)<=1e4&&
                    Math.abs(dot3(a.tangent,a.tangent)-1)<=1e-10&&Math.abs(dot3(a.director,a.director)-1)<=1e-10&&
                    Math.abs(dot3(a.tangent,a.director))<=1e-10&&Math.hypot(...a.tangent.map((v,k)=>v-f.tangent[k]))<=1e-10];
            }));
            const previous=records.map(rec=>{
                const input=inputFor(rec,oldPositions);consumeQuery();preparationQueries++;
                const raw=evaluateKirchhoffLumenSegmentContact(input),contact=raw.side,geometry=scratch.previousGeometry;
                if(!contact||raw.samples.length!==1||contact.innerT!==rec.sample.s||!differentiateCompositeLumenSideContact({input,contact},geometry,{order:'gradient'}).supported)
                    throw failure('Previous declared sample is not a supported original strict-side contact');
                const regular=rec.tools.every(t=>regularEdges.get(t))&&1-dot3(geometry.normal,rec.tools[0].reference.tangent)**2>.01;
                return {previous:copy({input,contact}),geometry:{normal:Array.from(geometry.normal)},regular};
            });
            records.forEach((rec,i)=>{rec.previous=previous[i].previous;rec.previousGeometry=previous[i].geometry;rec.previousRegular=previous[i].regular;});
            preparedReady=true;return {queries:preparationQueries,samples:records.length};
        } finally{scratch.busy=false;}
    }
    function configuration(toolPositions,currentFrames=null) {
        const values=[];
        for(const t of edges.values()) {
            const p=toolPositions?.get(t.id);if(p?.length!==n)throw failure('Missing current physical tool geometry');
            values.push(...vector(p[t.edge],3,'Current surface endpoint'),...vector(p[t.edge+1],3,'Current surface endpoint'),finite(candidate.angles.get(t.id)?.[t.edge],'Current own angle'));
            if(currentFrames){
                const points=[p[t.edge],p[t.edge+1]],frame=chordUnit(points);
                frame.regular=frame.length>=1e-4&&frame.length<=1e4&&!points.some(p=>p.some(v=>Math.abs(v)>1e4))&&
                    !(Math.abs(candidate.angles.get(t.id)[t.edge])>1e4)&&!(dot3(frame.tangent,t.reference.tangent)<.99);
                currentFrames.set(t,frame);
            }
        }
        return values;
    }
    function surfaceTools(rec,toolPositions) {
        const fractions=[rec.sample.geometry.rawContact.innerT,rec.sample.geometry.rawContact.outerT];
        return rec.tools.map((t,i)=>{
            const f=fractions[i],dx=t.coordinates[1]-t.coordinates[0];
            const previousFraction=(t.currentMap.sStart+t.currentMap.dsDx*dx*f-t.previousMap.sStart)/(t.previousMap.dsDx*dx);
            return {...t,positions:[toolPositions.get(t.id)[t.edge],toolPositions.get(t.id)[t.edge+1]],angle:candidate.angles.get(t.id)[t.edge],
                trace:trace(f),materialMap:{...t.currentMap,dsDtEnds:t.rates.slice()},materialPath:{kind:'linear-affine-maps',previousEdgeId:t.edgeId,previousMap:{...t.previousMap},previousTrace:trace(previousFraction)}};
        });
    }
    function refresh({toolPositions,commonResidual,relativeResidual,order}) {
        invalidate();verify();if(!preparedReady)throw failure('Friction needs its original previous queries before refresh');scratch.busy=true;
        try {
            vector(commonResidual,layout.dofCount,'Current common residual');vector(relativeResidual,r,'Current relative residual');
            if(order!=='full'&&order!=='gradient')throw failure('Explicit full or gradient friction evaluation is required');
            // Fresh for this exact configuration, shared only among its edge
            // samples. No frame or eligibility result survives a refresh.
            const currentFrames=new Map(),values=configuration(toolPositions,currentFrames),proofs=[];let converged=true,merit=0,lineSearchMerit=0;
            for(const rec of records) {
                const s=rec.sample,g=s.geometry,Fn=normal.normalForces[rec.index],mu=rec.mu,input=inputFor(rec,toolPositions);
                const physicalPoints=[input.innerStart,input.innerEnd,input.outerStart,input.outerEnd];
                const absent=implicitRate&&s.eliminated&&Fn===0&&(s.applicable===false||s.gap>0),
                    zeroTraction=tractions[2*rec.index]===0&&tractions[2*rec.index+1]===0;
                if(!absent&&(!Number.isFinite(s.row.residual)||g.supported!==true||g.rawContact.kind!==(s.currentFeature??s.feature)||s.gap!==g.gap||
                    g.positions.some((p,i)=>!same(p,physicalPoints[i]))||!['distal-rim','external-capsule'].includes(s.feature)&&g.rawContact.innerT!==s.s||!implicitRate&&s.feature!=='side'))
                    throw failure('A fresh original normal strict-side query at the current physical geometry is required');
                if(Fn===0&&zeroTraction&&(absent||s.gap>0&&s.active===false&&
                    s.row.residual===0&&s.row.multiplierDerivative>0&&s.row.jacobian.every(v=>v===0)&&
                    (implicitRate||regularOpenHistory(rec,currentFrames,dt)))) {
                    // The ORIGINAL inactive normal row is dFn/k=0. With
                    // Fn=Ft=0, substitution in the Coulomb row yields
                    // dFt/penalty=0 independently of the unknown slip. Its
                    // mechanical B*dFt contribution and Ft*DB are exactly
                    // zero. These isolated rows are the reduced equations,
                    // not a claim that the physical surface B itself is zero.
                    for(const row of rec.rows){
                        row.knownZeroCone=true;row.residual=0;row.multiplierDerivative=1/penalty;
                        row.multiplierJacobian.fill(0);row.forceColumn.fill(0);
                        if(order==='full'){row.jacobian.fill(0);row.geometricTangent.fill(0);}
                        row.geometricTangentValid=order==='full';
                    }
                    zeroConeEliminations++;scratch.zeroConeEliminations++;
                    proofs.push({sampleId:s.sampleId,sourceFeature:s.feature,feature:s.currentFeature,Fn:0,traction:[0,0],slip:null,slipRequired:false,mu:mu.slice(),equationResidual:[0,0],
                        work:0,minimumWork:0,workGap:0,coneViolation:0,slipResidual:0,converged:true,redundant:rec.mapping===null,normalAdmissible:true,
                        zeroConeProof:'strict-open-inactive-normal-deltaFn-zero-implies-deltaFt-zero',surfaceWitness:'not-required-for-known-zero-increments',
                        ...(implicitRate?{rateMode:policy.rateMode,slipModel:policy.slipModel,finiteStepSlipKnown:false}: {})});
                    continue;
                }
                surfaceEvaluations++;scratch.surfaceEvaluations++;
                const surface=implicitRate?evaluateCompositeJointLumenRateSurface({geometry:g,input,tools:surfaceTools(rec,toolPositions),dt,
                    order:order==='full'?'full':'value'},scratch.rateSurface):evaluateCompositeJointLumenSurface({current:{input,contact:g.rawContact},previous:rec.previous,tools:surfaceTools(rec,toolPositions),dt,
                    order:order==='full'?'full':'value'},scratch.surface);
                const slip=vector(surface.increment,2,'Finite material surface increment');
                if(surface.incrementValid!==true||surface.forceMapValid!==true||order==='full'&&(surface.slipJacobianValid!==true||surface.DforceMapValid!==true))
                    throw failure('Current finite slip/physical B and all requested geometry derivatives are required');
                sampleTraction[0]=tractions[2*rec.index];sampleTraction[1]=tractions[2*rec.index+1];
                const eq=evaluateCompositeFrictionEquation({traction:sampleTraction,slip,normalForce:Fn,mu,penalty},scratch.equation);
                if(!eq.valid||!eq.operatorReady)throw failure('Friction equation evaluation is invalid');
                const equationResidual=Array.from(eq.residual),equation=Math.max(...equationResidual.map(Math.abs));
                const physical=Fn>=0?measureCompositeFriction({traction:sampleTraction,slip,normalForce:Fn,mu,
                    slipTolerance:tol.frictionSlip,coneTolerance:tol.frictionCone,workTolerance:tol.frictionWork}):
                    {converged:false,work:sampleTraction[0]*slip[0]+sampleTraction[1]*slip[1],minimumWork:null,workGap:null,coneViolation:null,slipResidual:null};
                const ok=Fn>=0&&physical.converged&&equation<=tol.frictionEquation;
                const equationMerit=equationResidual.reduce((sum,v)=>sum+(v/tol.frictionEquation)**2,0);
                lineSearchMerit+=equationMerit;
                merit+=Fn>=0?equationMerit+(physical.slipResidual/tol.frictionSlip)**2+(physical.coneViolation/tol.frictionCone)**2+(physical.workGap/tol.frictionWork)**2:Infinity;
                converged=converged&&ok;
                proofs.push({sampleId:s.sampleId,sourceFeature:s.feature,feature:s.currentFeature,Fn,traction:Array.from(sampleTraction),slip,slipRequired:true,mu:mu.slice(),equationResidual,
                    work:physical.work,minimumWork:physical.minimumWork,workGap:physical.workGap,coneViolation:physical.coneViolation,slipResidual:physical.slipResidual,
                    converged:ok,redundant:rec.mapping===null,normalAdmissible:Fn>=0,surfaceWitness:implicitRate?surface.identity.witness:'virtual-midpoint-not-exact-cylinder-intersection',
                    ...(implicitRate?{rateMode:policy.rateMode,slipModel:policy.slipModel,finiteStepSlipKnown:false}: {})});
                if(!rec.mapping)continue;
                // A trial needs its true current B and finite slip. G/DB are
                // formed only for a full Newton assembly, never approximated
                // by B or reused from an earlier geometry.
                const mapped=pullbackCompositeJointSurface(order==='full'?surface:{tools:surface.tools,forceMap:surface.forceMap,forceMapValid:true},rec.mapping),
                    loads=evaluateCompositeJointSurfaceLoads(sampleTraction,mapped),size=mapped.dofCount;
                rec.rows.forEach((row,c)=>{
                    row.residual=eq.residual[c];row.multiplierDerivative=eq.tractionJacobian[2*c+c];
                    row.multiplierJacobian[0]=eq.normalDerivative[c];row.multiplierJacobian[1]=eq.tractionJacobian[2*c+1-c];
                    for(let j=0;j<size;j++){row.jacobian[j]=order==='full'?eq.slipJacobian[2*c]*mapped.slipJacobian[j]+eq.slipJacobian[2*c+1]*mapped.slipJacobian[size+j]:NaN;row.forceColumn[j]=mapped.rows[c].forceColumn[j];}
                    if(order==='full')for(let j=0;j<size*size;j++)row.geometricTangent[j]=-sampleTraction[c]*mapped.rows[c].forceDerivative[j];
                    row.geometricTangentValid=order==='full';
                });
                mapped.commonDofs.forEach((d,i)=>commonScratch[d]-=loads.common[i]);mapped.relativeDofs.forEach((d,i)=>relativeScratch[d]-=loads.relative[i]);
                for(const t of loads.tools){t.nodes.forEach((node,e)=>t.nodalForces[e].forEach((v,k)=>nodalForces.get(t.id)[node][k]+=v));spinTorques.get(t.id)[t.edge]+=t.scalarTorque;}
            }
            if(!Number.isFinite(lineSearchMerit)||!commonScratch.every(Number.isFinite)||!relativeScratch.every(Number.isFinite)||
                commonScratch.some((v,i)=>!Number.isFinite(v+commonResidual[i]))||relativeScratch.some((v,i)=>!Number.isFinite(v+relativeResidual[i])))throw failure('Nonfinite complete friction residual');
            commonScratch.forEach((v,i)=>commonResidual[i]+=v);relativeScratch.forEach((v,i)=>relativeResidual[i]+=v);
            const certificate={scope:implicitRate?'original-native-side-fillet-rim-implicit-rate-Coulomb-rows':'original-strict-side-finite-Coulomb-rows',
                ...(implicitRate?{rateMode:policy.rateMode,slipModel:policy.slipModel,finiteStepSlipKnown:false}:{}),converged,merit,lineSearchMerit,samples:proofs,
                sampleCount:records.length,retainedRows:rows.length};
            last={toolPositions,values,tractions:tractions.slice(),normalForces:normal.normalForces.slice()};commitReady=converged;refreshes++;scratch.refreshes++;
            return certificate;
        } catch(error){invalidate();throw error;} finally{scratch.busy=false;}
    }
    function commit() {
        verify();
        if(!commitReady||!last||!same(configuration(last.toolPositions),last.values)||!same(tractions,last.tractions)||!same(normal.normalForces,last.normalForces))
            throw failure('Friction commit requires unchanged certified positions, own angles, Ft and Fn');
        return {signature,sampleIds:sampleIds.slice(),tractions:tractions.slice(),law:'coulomb',mu:mu.slice(),...(muByPair?{muByPair:copy(muByPair)}:{}),materialPath:'linear-affine-maps',
            ...(implicitRate?{rateMode:policy.rateMode,slipModel:policy.slipModel,finiteStepSlipKnown:false}:{}),
            currentMaps:Array.from(edges.values(),t=>({id:t.id,edge:t.edge,edgeId:t.edgeId,currentMap:{...t.currentMap}}))};
    }
    return {signature,rows,tractions,nodalForces,spinTorques,prepare,refresh,commit,workspace,
        get diagnostics(){return {preparationQueries,refreshes,retainedRows:rows.length,samples:records.length,surfaceEvaluations,zeroConeEliminations};}};
}
