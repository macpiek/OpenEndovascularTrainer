import {compositeJointContactAddition} from './kirchhoffCompositeJointContactHistory.js';
import {createCompositeExternalCapsuleGeometryWorkspace,evaluateCompositeExternalCapsuleContact} from './kirchhoffCompositeExternalCapsuleGeometry.js';
import { evaluateKirchhoffLumenSegmentContact } from './kirchhoffLumenContact.js';
import { materialSegmentContactId } from './kirchhoffContactManifold.js';
import { createCompositeLumenSideGeometryWorkspace, differentiateCompositeLumenSideContact } from './kirchhoffCompositeLumenSideGeometry.js';
import { createCompositeLumenTipGeometryWorkspace, differentiateCompositeLumenTipContact } from './kirchhoffCompositeLumenTipGeometry.js';
import { createCompositeContactPullbackFactory, pullbackCompositeContact } from './kirchhoffCompositeContactPullback.js';

const positive=(v,name)=>{if(!(v>0)||!Number.isFinite(v))throw new RangeError(`${name} must be positive and finite`);return v;};
const nonnegative=(v,name)=>{if(!(v>=0)||!Number.isFinite(v))throw new RangeError(`${name} must be nonnegative and finite`);return v;};
const text=(v,name)=>{if(typeof v!=='string'||!v)throw new RangeError(`${name} must be a nonempty string`);return v;};
const failure=(reason,details=null)=>{const error=new RangeError(reason);error.code='unsupported-lumen-contact';if(details)error.details=details;return error;};
const workspaces=new WeakMap();

/** Retains only the frozen coordinate compiler and its bounded support plans.
 * Each manager still owns its rows, detector outputs, Fn and history; sharing
 * this handle cannot make a previous manager's mutable state current.
 */
export function createCompositeJointLumenRowWorkspace() {
    const scratch={factory:null,chartBuilds:0,chartHits:0};
    const handle=Object.freeze({get diagnostics(){return {chartBuilds:scratch.chartBuilds,chartHits:scratch.chartHits,
        support:scratch.factory?.diagnostics??null};}});
    workspaces.set(handle,scratch);return handle;
}

/** Fixed-chart, explicit per-sample ORIGINAL lumen inequalities. Every s is
 * queried separately with quadrature:[s]; no winner/max owns a shared Fn.
 * R=(Fn-max(0,Fn-k*g))/k is in mm; k is a numerical N/mm scale, not contact
 * stiffness. The physical normal load stays Fn, including signed private
 * Newton trials. Mechanics is -Fn B, d/dq=-Fn DB and d/dFn=-B, each once.
 * On one strict affine side, r(s) is affine and its norm is convex. Only
 * the smallest/largest declared s need independent duals. Interior Fn=0 is
 * an explicit redundant-inequality gauge; EVERY original sample is queried
 * and certified. A nonzero old interior dual requires a separately proved
 * force-column transfer on the old physical geometry, never a DB transfer.
 * Explicit distal-fillet pairs retain each declared sample; distal-rim pairs
 * retain one moving exact plane crossing, with physical B=G/|G_wire| and its
 * full derivative. Neither tip feature uses the affine-side dual gauge.
 * This is a fixed declared feature chart, not portal ownership/CCD or a
 * search over undeclared pairs. No friction, remapping or history mutation.
 * preserveSampleReactions retains every declared pressure unknown; use it
 * when surface friction requires independent loads at interior samples.
 */
export function createCompositeJointLumenRows({layout,coordinates,modes,relativeToolId='wire',contacts,history=null,tolerances,preserveSampleReactions=false,workspace=null}) {
    if(contacts?.mode!=='lumen-normal'||contacts.friction!=='none')throw new RangeError('Explicit lumen-normal contact with friction:none is required');
    if(typeof preserveSampleReactions!=='boolean')throw new TypeError('preserveSampleReactions must be boolean');
    const chartId=text(contacts.chartId,'Contact chart id'),k=positive(contacts.forcePerLength,'NCP forcePerLength scale');
    if(coordinates?.length!==layout.nodeCount||!coordinates.every(Number.isFinite))throw new RangeError('Explicit frozen contact chart coordinates are required');
    if(!Array.isArray(contacts.pairs)||!contacts.pairs.length)throw new RangeError('Explicit local material pairs and quadrature are required');
    const chart={layout,modes,relativeToolId,supportPolicy:contacts.pairs.some(p=>p.feature==='external-capsule')?'declared-tool-pair':'local-two-edge'},scratch=workspace===null?null:workspaces.get(workspace);
    if(workspace!==null&&!scratch)throw new TypeError('Use a prepared lumen row workspace');
    let createPullback;
    if(scratch?.factory?.matches(chart)){createPullback=scratch.factory;scratch.chartHits++;}
    else {createPullback=createCompositeContactPullbackFactory(chart);if(scratch){scratch.factory=createPullback;scratch.chartBuilds++;}}
    const pairIds=new Set(),physicalPairs=new Set(),samples=[],provenance=[],groups=[];
    for(const pair of contacts.pairs) {
        const id=text(pair.id,'Contact pair id');if(pairIds.has(id))throw new RangeError('Duplicate pair id');pairIds.add(id);
        const innerToolId=text(pair.innerToolId,'Inner material id'),outerToolId=text(pair.outerToolId,'Outer material id');
        if(innerToolId!==relativeToolId||outerToolId===innerToolId||!layout.spins.has(outerToolId))throw new RangeError('Lumen requires the wire inside the other physical material');
        const innerEdge=pair.innerEdge,outerEdge=pair.outerEdge;
        for(const [toolId,edge] of [[innerToolId,innerEdge],[outerToolId,outerEdge]])if(!Number.isInteger(edge)||edge<0||edge>=layout.nodeCount-1||!layout.edgeToolIds[edge].includes(toolId))throw new RangeError('Each contact segment must belong to its active material');
        const feature=pair.feature??'side';
        if(!['side','distal-fillet','distal-rim','external-capsule'].includes(feature))throw new RangeError('Explicit side, distal-fillet or distal-rim feature is required');
        const pairKey=JSON.stringify([innerToolId,innerEdge,outerToolId,outerEdge,feature]);
        if(physicalPairs.has(pairKey))throw new RangeError('Duplicate semantic physical material pair');physicalPairs.add(pairKey);
        const external=feature==='external-capsule',rim=feature==='distal-rim'||external,quadrature=Array.from(pair.quadrature??[]);
        if(rim&&pair.quadrature!==undefined)throw new RangeError('A distal-rim pair uses its exact moving crossing, not quadrature');
        if(!rim&&(!quadrature.length||quadrature.some(s=>!Number.isFinite(s)||s<0||s>1)||new Set(quadrature).size!==quadrature.length))throw new RangeError('Explicit unique quadrature samples in [0,1] are required');
        const lumenRadius=external?null:nonnegative(pair.lumenRadius,'Lumen radius'),outerRadius=external?nonnegative(pair.outerRadius,'Outer radius'):null,innerRadius=nonnegative(pair.innerRadius,'Wire radius');
        if(!external&&lumenRadius<innerRadius)throw new RangeError('Wire radius exceeds the declared lumen');
        // Preserve the legacy side-only contract unless the caller explicitly
        // declares its feature. An open side then remains strictly before the
        // original detector's fillet/portal interval at every trial.
        if(pair.feature===undefined&&(pair.openDistal!==false||pair.portalFilletRadius!==0))throw new RangeError('This bounded contact requires explicit openDistal:false and portalFilletRadius:0');
        if(!external&&typeof pair.openDistal!=='boolean')throw new RangeError('An explicit distal opening policy is required');
        const openDistal=external?false:pair.openDistal,portalFilletRadius=external?0:nonnegative(pair.portalFilletRadius,'Portal fillet radius');
        if(external&&typeof pair.openDistalB!=='boolean')throw new RangeError('External capsule needs its actual distal opening policy');
        if(pair.endpointDerivative!==undefined&&pair.endpointDerivative!=='clamped-one-sided')throw new RangeError('Unknown lumen endpoint derivative policy');
        if(openDistal&&layout.edgeToolIds.slice(outerEdge+1).some(ids=>ids.includes(outerToolId)))throw new RangeError('An open distal feature must belong to the actual outer material tip');
        if(!external&&feature!=='side'&&!openDistal)throw new RangeError('Tip contacts require an open distal end');
        if(feature==='distal-fillet'&&!(portalFilletRadius>1e-12))throw new RangeError('A smooth distal fillet needs a positive radius');
        const featurePolicy=pair.featurePolicy??null;
        if(featurePolicy!==null&&(featurePolicy!=='native-side-fillet'||!['side','distal-fillet'].includes(feature)||!openDistal||!(portalFilletRadius>1e-12)||!preserveSampleReactions))throw new RangeError('Native side-fillet transitions need an open rounded tip and unreduced sample reactions');
        const conditionalPortalSamples=Array.from(pair.conditionalPortalSamples??[]);
        if(conditionalPortalSamples.some(s=>feature!=='side'||!openDistal||!quadrature.includes(s))||new Set(conditionalPortalSamples).size!==conditionalPortalSamples.length)throw new RangeError('Conditional portal samples must be declared open-distal side quadrature');
        const materialPairId=materialSegmentContactId(pair.innerMaterialSegmentId,pair.outerMaterialSegmentId,`joint-lumen:${feature}`);
        const points=[{toolId:innerToolId,node:innerEdge},{toolId:innerToolId,node:innerEdge+1},{toolId:outerToolId,node:outerEdge},{toolId:outerToolId,node:outerEdge+1}];
        const metadata={id,materialPairId,points,quadrature,...(featurePolicy?{featurePolicy}:{}),...(conditionalPortalSamples.length?{conditionalPortalSamples}:{}),lumenRadius,innerRadius,openDistal,portalFilletRadius,...(external?{outerRadius,openDistalB:pair.openDistalB}:{}),...(pair.endpointDerivative?{endpointDerivative:pair.endpointDerivative}:{})};provenance.push(metadata);
        const group=[];groups.push(group);
        for(const s of rim?[null]:quadrature) {
            const pulled=createPullback(points),size=pulled.dofCount,index=samples.length;
            const sampleId=JSON.stringify([chartId,materialPairId,points,s]);
            const input={innerStart:null,innerEnd:null,outerStart:null,outerEnd:null,lumenRadius,innerRadius,...(external?{outerRadius,openDistalB:pair.openDistalB}:{}),
                innerMaterialSegmentId:structuredClone(pair.innerMaterialSegmentId),outerMaterialSegmentId:structuredClone(pair.outerMaterialSegmentId),innerSegmentIndex:innerEdge,outerSegmentIndex:outerEdge,
                // Quadrature does not select the exact portal crossing. The
                // detector requires a nonempty list, but only its portal
                // record is consumed by this row. Large activation distance
                // requests that original record on both sides of g=0; it
                // changes no gap, feature, Fn or physical activation rule.
                quadrature:rim?[.5]:[s],openDistal,portalFilletRadius,activationDistance:rim?Number.MAX_VALUE:0,featurePrefix:'joint-lumen',
                ...(pair.endpointDerivative?{endpointDerivative:pair.endpointDerivative}:{})};
            const row={toolId:'lumen-normal',index,anchorNode:pulled.anchorNode,commonDofs:pulled.commonDofs,relativeDofs:pulled.relativeDofs,unit:'mm',
                jacobian:new Float64Array(size),forceColumn:new Float64Array(size),geometricTangent:new Float64Array(size*size),
                geometricTangentValid:false,multiplierDerivative:0,residual:NaN,tolerance:tolerances.linearConstraint};
            if(external)row.constraintSupport={kind:'native-tool-pair',pairId:id,tools:[{id:innerToolId,edge:innerEdge},{id:outerToolId,edge:outerEdge}],spins:false};
            const sample={index,pairId:id,sampleId,s,feature,featurePolicy,currentFeature:feature,conditionalPortal:conditionalPortalSamples.includes(s),input,points,pulled,row,
                geometry:external?createCompositeExternalCapsuleGeometryWorkspace():feature==='side'?createCompositeLumenSideGeometryWorkspace():createCompositeLumenTipGeometryWorkspace(),gap:NaN,active:false,eliminated:false};
            if(featurePolicy)sample.featureGeometry={side:feature==='side'?sample.geometry:createCompositeLumenSideGeometryWorkspace(),'distal-fillet':feature==='distal-fillet'?sample.geometry:createCompositeLumenTipGeometryWorkspace()};
            samples.push(sample);group.push(sample);
        }
    }
    const rowSampleIndices=[];
    for(const group of groups) {
        // A mechanical edge may carry several independent surface samples.
        // Coulomb requires their own normal loads and material histories;
        // a normal-only endpoint force gauge does not preserve friction work.
        if(preserveSampleReactions||group[0].feature!=='side'){group.forEach(sample=>sample.redundant=false);continue;}
        const ordered=group.slice().sort((a,b)=>a.s-b.s),a=ordered[0],b=ordered.at(-1);
        for(const sample of group) {
            sample.redundant=sample!==a&&sample!==b;
            if(sample.redundant)sample.envelope={a,b,u:(sample.s-a.s)/(b.s-a.s)};
        }
    }
    for(const group of groups)for(const sample of group)sample.hasEndpointGauge=group.some(s=>s.redundant);
    for(const sample of samples)if(!sample.redundant)rowSampleIndices.push(sample.index);
    const signature=JSON.stringify({chartId,coordinates:Array.from(coordinates),edges:layout.edgeToolIds,
        modes:modes.map(m=>({node:m.node,basis:m.basis.map(b=>Array.from(b))})),provenance}),sampleIds=samples.map(s=>s.sampleId);
    const historyAddition=contacts.historyUpdate==='preserve-and-append'&&preserveSampleReactions&&history&&history.signature!==signature?compositeJointContactAddition({previousSignature:history.signature,currentSignature:signature,previousSampleIds:history.sampleIds,currentSampleIds:sampleIds}):null;
    if(history!==null&&!historyAddition&&(history.signature!==signature||history.friction!=='none'||history.normalForces?.length!==samples.length||
        history.sampleIds?.length!==samples.length||sampleIds.some((id,i)=>history.sampleIds[i]!==id))) {
        const error=new RangeError('Lumen chart/support/material/radii/sample provenance changed');
        error.details={previousSignature:history.signature,currentSignature:signature,previousSampleIds:history.sampleIds,currentSampleIds:sampleIds};throw error;
    }
    if(historyAddition&&(history.friction!=='none'||history.normalForces?.length!==history.sampleIds.length))throw new RangeError('Original normal history needs all accepted sample forces');
    const normalForces=history===null?new Float64Array(samples.length):historyAddition?Float64Array.from(historyAddition.sourceIndices,i=>i<0?0:nonnegative(history.normalForces[i],'Accepted normal force')):Float64Array.from(history.normalForces,v=>nonnegative(v,'Accepted normal force'));
    const nodalForces=new Map([...layout.spins.keys()].map(id=>[id,Array.from({length:layout.nodeCount},()=>[0,0,0])]));
    const commonScratch=new Float64Array(layout.dofCount),relativeScratch=new Float64Array(3*modes.length);
    const physicalPoints=[...new Map(samples.flatMap(sample=>sample.points).map(point=>[JSON.stringify([point.toolId,point.node]),point])).values()];
    const refreshedPoints=new Float64Array(3*physicalPoints.length),refreshedForces=new Float64Array(normalForces.length);
    let certificate=null,gaugePrepared=false,refreshedPositions=null,commitReady=false;
    const gauge={kind:preserveSampleReactions?'per-sample-reactions':samples.every(s=>s.feature==='side')?'affine-side-endpoints':'explicit-tip-and-affine-side-endpoints',retainedRows:rowSampleIndices.length,redundantSamples:samples.length-rowSampleIndices.length,
        transfers:[],preparationQueries:0};
    function query(sample,toolPositions,consumeQuery,differentialOrder='full') {
        const {input,points,index}=sample,Fn=normalForces[index];let geometry=sample.geometry;
        sample.currentFeature=sample.feature;
        if(!Number.isFinite(Fn))throw failure('Nonfinite private lumen force');
        [input.innerStart,input.innerEnd,input.outerStart,input.outerEnd]=points.map(p=>toolPositions.get(p.toolId)[p.node]);
        consumeQuery();
        if(sample.feature==='external-capsule') {
            const physical=evaluateCompositeExternalCapsuleContact({input,order:differentialOrder},geometry);
            if((physical.openDistalExcluded||!physical.supported&&physical.reason==='coincident-native-capsule-witness')&&Fn===0){sample.gap=null;sample.currentFeature=null;sample.eliminated=true;sample.applicable=false;sample.innerT=sample.outerT=null;return physical;}
            if(physical.openDistalExcluded)throw failure('Loaded external capsule left actual open-distal ownership');
            if(!physical.supported)throw failure(`Unsupported original external capsule: ${physical.reason}`);
            sample.gap=physical.gap;sample.eliminated=false;sample.applicable=true;sample.innerT=physical.innerT;sample.outerT=physical.outerT;return physical;
        }
        const raw=evaluateKirchhoffLumenSegmentContact(input);
        if(sample.conditionalPortal&&!raw.portal.crosses&&!(sample.featurePolicy&&raw.fillet)&&Fn===0) {
            if(sample.hasEndpointGauge)throw failure('Conditional portal eligibility changed inside the affine endpoint pressure gauge');
            // This is an EXTRA distal quadrature site, emitted only while
            // its own segment crosses the original aperture. Primary side
            // samples have independent rows and never enter this branch.
            // Preserve the zero slot for birth, with no invented gap/normal.
            sample.gap=null;sample.currentFeature=null;sample.eliminated=true;sample.applicable=false;sample.innerT=sample.outerT=null;
            return {supported:false,reason:'unloaded-absent-conditional-portal-side'};
        }
        if(sample.feature==='distal-rim'&&!raw.portal.crosses&&Fn===0) {
            // The ORIGINAL detector proves this inequality absent. Retain its
            // zero dual for re-entry, but invent neither a gap nor a normal.
            // A loaded crossing cannot disappear through this branch.
            sample.gap=null;sample.currentFeature=null;sample.eliminated=true;sample.applicable=false;sample.innerT=sample.outerT=null;
            return {supported:false,reason:'unloaded-absent-rim'};
        }
        const contact=sample.featurePolicy?(raw.side??raw.fillet):sample.feature==='side'?raw.side:sample.feature==='distal-fillet'?raw.fillet:raw.portal.contact;
        if(sample.featurePolicy&&contact) {
            // Preparation owns the material sample ID and its four physical
            // nodes. The ORIGINAL current record owns its local side/fillet
            // geometry. Switching that record changes neither Fn/Ft storage
            // nor the frozen common/relative pullback or material history.
            sample.currentFeature=contact.kind;geometry=sample.geometry=sample.featureGeometry[contact.kind];
        }
        const currentFeature=sample.currentFeature;
        if(!contact||currentFeature==='side'&&raw.samples.length!==1||currentFeature!=='distal-rim'&&contact.innerT!==sample.s)
            throw failure('Declared lumen sample left its original side segment/portal support',{feature:sample.feature,s:sample.s,Fn,pairId:sample.pairId,input,raw});
        const physical=currentFeature==='side'?differentiateCompositeLumenSideContact({input,contact},geometry,{order:differentialOrder}):differentiateCompositeLumenTipContact({input,contact},geometry),gap=contact.gap;
        const eliminated=!physical.supported&&Fn===0&&gap>0&&(physical.reason==='zero-or-fallback-radial-normal'||
            sample.feature==='distal-rim'&&physical.reason==='rim-endpoint-or-clamped-crossing');
        if(!physical.supported&&!eliminated)throw failure(`Unsupported original lumen ${sample.feature}: ${physical.reason}`);
        sample.gap=gap;sample.eliminated=eliminated;sample.applicable=true;sample.innerT=contact.innerT;sample.outerT=contact.outerT;
        return physical;
    }
    // Called once on incoming STATE geometry, before any prescribed movement.
    // The accepted history is copied; even a partial/failed preparation cannot
    // mutate it. Endpoint tangents are always recomputed at trial geometry.
    function prepareGauge({toolPositions,consumeQuery}) {
        if(gaugePrepared)throw failure('Lumen gauge already prepared');
        for(const group of groups) {
            const loaded=group.filter(s=>s.redundant&&normalForces[s.index]!==0);
            if(!loaded.length)continue;
            for(const sample of group)query(sample,toolPositions,()=>{consumeQuery();gauge.preparationQueries++;});
            for(const sample of loaded) {
                const {a,b,u}=sample.envelope,source=sample.geometry,A=a.geometry,B=b.geometry;
                // Conservative proof: identical unit normals and exactly
                // affine stored projection t. No physical tolerance or rank
                // threshold can authorize this history change. The final B
                // check allows only arithmetic round-off in the interpolation.
                if(!source.supported||!A.supported||!B.supported||
                    !source.normal.every((v,k)=>v===A.normal[k]&&v===B.normal[k])||
                    sample.outerT!==(1-u)*a.outerT+u*b.outerT||
                    !source.normalForceColumn.every((v,k)=>{
                        const x=(1-u)*A.normalForceColumn[k],y=u*B.normalForceColumn[k];
                        return Math.abs(v-(x+y))<=8*Number.EPSILON*(Math.abs(v)+Math.abs(x)+Math.abs(y));
                    }))throw failure('Nonzero interior lumen history lacks an exact affine force-column transfer');
                const Fn=normalForces[sample.index],left=(1-u)*Fn,right=u*Fn;
                normalForces[a.index]+=left;normalForces[b.index]+=right;normalForces[sample.index]=0;
                if(!Number.isFinite(normalForces[a.index])||!Number.isFinite(normalForces[b.index]))throw failure('Nonfinite lumen gauge transfer');
                gauge.transfers.push({sampleId:sample.sampleId,Fn,endpoints:[a.sampleId,b.sampleId],weights:[1-u,u],forces:[left,right]});
            }
        }
        gaugePrepared=true;
        return gauge;
    }
    function refreshPrepared({toolPositions,commonResidual,relativeResidual,order,consumeQuery}) {
        for(const p of nodalForces.values())p.forEach(v=>v.fill(0));
        let minGap=Infinity,minForce=Infinity,negativeForce=0,penetration=0,ncp=0,complementarity=0,merit=0,lineSearchMerit=0,activeCount=0,eliminatedCount=0;
        const proofs=[];
        for(const sample of samples) {
            const {points,row,pulled,index}=sample,Fn=normalForces[index];
            if(!gaugePrepared)throw failure('Lumen gauge must be prepared on incoming physical geometry');
            if(sample.redundant&&Fn!==0)throw failure('Interior lumen dual changed outside its explicit zero gauge');
            // DB contributes only as -Fn*DB. At exactly zero Fn it is not
            // needed even for a full Newton assembly. The ORIGINAL gap,
            // branch validation, G and force column -B are still evaluated.
            const differentialOrder=order==='full'&&Fn!==0?'full':'gradient';
            const physical=query(sample,toolPositions,consumeQuery,differentialOrder),gap=sample.gap,eliminated=sample.eliminated;
            const active=sample.applicable&&Fn-k*gap>0;sample.active=active;
            row.residual=active?gap:Fn/k;row.multiplierDerivative=active?0:1/k;
            row.geometricTangentValid=order==='full';
            if(eliminated) {
                // The original strictly open inequality proves Fn=0. The
                // eliminated normal has no physical column; its retained
                // storage slot enforces exactly deltaFn=0. No normal invented.
                row.jacobian.fill(0);row.forceColumn.fill(0);if(order==='full')row.geometricTangent.fill(0);eliminatedCount++;
            } else {
                pullbackCompositeContact(physical,pulled,{order:differentialOrder});
                for(let j=0;j<row.jacobian.length;j++){row.jacobian[j]=active?pulled.gapJacobian[j]:0;row.forceColumn[j]=pulled.forceColumn[j];}
                if(order==='full'){
                    if(Fn===0)row.geometricTangent.fill(0);
                    else for(let j=0;j<row.geometricTangent.length;j++)row.geometricTangent[j]=-Fn*pulled.normalDerivative[j];
                }
                row.commonDofs.forEach((d,j)=>commonResidual[d]+=Fn*row.forceColumn[j]);
                row.relativeDofs.forEach((d,j)=>relativeResidual[d]+=Fn*row.forceColumn[row.commonDofs.length+j]);
                points.forEach((p,i)=>{const force=nodalForces.get(p.toolId)[p.node];for(let axis=0;axis<3;axis++)force[axis]+=Fn*physical.normalForceColumn[3*i+axis];});
            }
            const gapViolation=sample.applicable?Math.max(0,-gap):0,forceViolation=Math.max(0,-Fn),residual=Math.abs(row.residual),work=sample.applicable?Math.abs(Fn*gap):0;
            if(sample.applicable)minGap=Math.min(minGap,gap);minForce=Math.min(minForce,Fn);negativeForce=Math.max(negativeForce,forceViolation);
            penetration=Math.max(penetration,gapViolation);ncp=Math.max(ncp,residual);complementarity=Math.max(complementarity,work);if(active)activeCount++;
            merit+=(gapViolation/tolerances.lumenGap)**2+(forceViolation/tolerances.force)**2+(residual/tolerances.lumenNcp)**2+(work/tolerances.lumenWork)**2;
            lineSearchMerit+=(gapViolation/tolerances.lumenGap)**2+(forceViolation/tolerances.force)**2+(residual/tolerances.lumenNcp)**2;
            const proof={sampleId:sample.sampleId,...(sample.conditionalPortal?{conditionalPortal:true,applicable:sample.applicable}:{}),Fn,gap,innerT:sample.innerT,outerT:sample.outerT,active,eliminated,redundant:sample.redundant,ncp:residual,complementarity:work};
            if(sample.feature!=='side'||sample.featurePolicy)Object.assign(proof,{feature:sample.currentFeature,sourceFeature:sample.feature,applicable:sample.applicable,forceScale:physical.supported?physical.forceScale:null});
            proofs.push(proof);
        }
        if(!Number.isFinite(merit))throw failure('Nonfinite original lumen certificate');
        certificate={minGap,minForce,penetration,negativeForce,ncp,complementarity,activeCount,eliminatedCount,sampleCount:samples.length,retainedRows:rowSampleIndices.length,redundantSamples:gauge.redundantSamples,samples:proofs,merit,
            converged:minForce>=0&&penetration<=tolerances.lumenGap&&ncp<=tolerances.lumenNcp&&complementarity<=tolerances.lumenWork};
        // Keep the original work acceptance gate. Fn*g is a redundant product,
        // not another equation in the prepared normal Newton system.
        certificate.lineSearchMerit=lineSearchMerit;
        return certificate;
    }
    function refresh({toolPositions,commonResidual,relativeResidual,order,consumeQuery}) {
        certificate=null;refreshedPositions=null;commitReady=false;
        try {
            if(commonResidual?.length!==commonScratch.length||relativeResidual?.length!==relativeScratch.length||
                !commonResidual.every(Number.isFinite)||!relativeResidual.every(Number.isFinite)||!['full','gradient'].includes(order))
                throw failure('Finite residuals in the current contact chart and explicit evaluation order are required');
            commonScratch.fill(0);relativeScratch.fill(0);
            const result=refreshPrepared({toolPositions,commonResidual:commonScratch,relativeResidual:relativeScratch,order,consumeQuery});
            // Publish forces only after every declared original query succeeds.
            commonScratch.forEach((v,i)=>commonResidual[i]+=v);relativeScratch.forEach((v,i)=>relativeResidual[i]+=v);
            physicalPoints.forEach((p,i)=>refreshedPoints.set(toolPositions.get(p.toolId)[p.node],3*i));
            refreshedForces.set(normalForces);refreshedPositions=toolPositions;
            // Returned diagnostics are mutable caller data, never authority
            // to turn a failed normal law into an accepted contact history.
            commitReady=result.converged===true;
            return result;
        } catch(error) {
            certificate=null;
            for(const p of nodalForces.values())p.forEach(v=>v.fill(0));
            for(const sample of samples) {
                sample.gap=NaN;sample.active=sample.eliminated=false;
                sample.row.residual=NaN;sample.row.geometricTangentValid=false;
                sample.row.jacobian.fill(NaN);sample.row.forceColumn.fill(NaN);sample.row.geometricTangent.fill(NaN);
            }
            throw error;
        }
    }
    function commit() {
        if(!commitReady)throw failure('Cannot commit an uncertified original lumen state');
        if(!refreshedPositions||normalForces.some((v,i)=>!Number.isFinite(v)||v<0||v!==refreshedForces[i])||
            physicalPoints.some((p,i)=>[0,1,2].some(k=>refreshedPositions.get(p.toolId)?.[p.node]?.[k]!==refreshedPoints[3*i+k])))
            throw failure('Lumen forces or physical geometry changed after their last certified refresh');
        return {signature,sampleIds:sampleIds.slice(),normalForces:normalForces.slice(),friction:'none',gauge:gauge.kind};
    }
    return {signature,historyAddition,samples,rows:rowSampleIndices.map(i=>samples[i].row),rowSampleIndices,normalForces,nodalForces,gauge,prepareGauge,refresh,commit};
}
