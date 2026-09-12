import test from 'node:test';
import assert from 'node:assert/strict';
import { createContactResult } from '../src/physics/collision/vesselContactField.js';
import { createCompositeDiscreteWallPoint } from '../src/physics/compositeDiscreteWallPoint.js';
import { captureCompositeReferenceFrames } from '../src/physics/kirchhoffCompositeElement.js';
import { createCompositeJointWallSurfaceWorkspace, evaluateCompositeJointWallSurface } from '../src/physics/kirchhoffCompositeJointWallSurface.js';
const norm=a=>Math.hypot(...a);
const close=(a,b,t=4e-7)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<t,`${a} != ${b}`);
function model(p, side, kind = 'min') {
    const [x, y, z] = p;
    let s, sy, sz, syz;
    if (kind === 'mixed') { s = side === 0 ? y - .5 : 0; sy = side === 0 ? 1 : 0; sz = 0; syz = 0; }
    else if (kind === 'zero') { s = side === 0 ? .5 : 0; sy = sz = syz = 0; }
    else if (kind === 'coincident') { s = side === 0 ? 1 : .5; sy = sz = syz = 0; }
    else if (kind === 'opposite') { s = side === 0 ? 1 : -1; sy = sz = syz = 0; }
    else {
        const lower = kind === 'smooth' || (kind === 'max' ? side === 1 : side === 0);
        s = lower ? .5 + .125 * y + .25 * z + .0625 * y * z : -.375 + .0625 * y - .125 * z - .03125 * y * z;
        sy = lower ? .125 + .0625 * z : .0625 - .03125 * z;
        sz = lower ? .25 + .0625 * y : -.125 - .03125 * y;
        syz = lower ? .0625 : -.03125;
    }
    const flat = ['zero', 'coincident', 'opposite'].includes(kind);
    const base = flat ? 4 : 4 + .25 * y + .5 * z + .125 * y * z;
    const G = [s, (flat ? 0 : .25 + .125 * z) + (x - 1) * sy, (flat ? 0 : .5 + .125 * y) + (x - 1) * sz];
    const H = [0, sy, sz, sy, 0, (flat ? 0 : .125) + (x - 1) * syz, sz, (flat ? 0 : .125) + (x - 1) * syz, 0];
    return { value: base + (x - 1) * s, G, H, n: G.map(v => v / norm(G)) };
}
function grid(kind = 'min', axis = 0) {
    const order = [axis, ...[0, 1, 2].filter(i => i !== axis)], size = 2, dimensions = [2, 2, 2], quantization = 1 / 1024;
    const distances = new Uint32Array(64), lookup = Uint16Array.from({ length: 8 }, (_, i) => i);
    for (let bz = 0; bz < 2; bz++) for (let by = 0; by < 2; by++) for (let bx = 0; bx < 2; bx++) {
        const brick = bx + 2 * (by + 2 * bz);
        for (let z = 0; z < size; z++) for (let y = 0; y < size; y++) for (let x = 0; x < size; x++) {
            const world = [2 * bx + x, 2 * by + y, 2 * bz + z], p = order.map(i => world[i]);
            const encoded = model(p, p[0] < 1 ? 0 : 1, kind).value / quantization;
            assert.equal(encoded, Math.round(encoded)); assert.ok(encoded >= 0);
            distances[brick * 8 + x + 2 * (y + 2 * z)] = encoded;
        }
    }
    return { sdfOrigin: [0, 0, 0], sdfDimensions: dimensions, brickSize: size, voxelSize: 1,
        sdfQuantization: quantization, sdfBrickLookup: lookup, sdfDistances: distances, sdfInsideBits: new Uint8Array(8).fill(255),
        querySphere() { throw Error('Unexpected point query'); }, queryCapsuleCoordinates() { throw Error('Unexpected capsule query'); } };
}

function fixture(branchIndex=0,x=1,fraction=.37){
    const field=grid();field.calls=0;
    field.queryCapsuleCoordinates=function(ax,ay,az,bx,by,bz,r,out=createContactResult()){
        this.calls++;const p=[ax,ay,az],g=model(p,p[0]<1?0:1);
        out.source='sparse-sdf';out.signedDistance=g.value;out.signedGap=g.value-r;out.segmentT=0;out.capsuleSampleCount=1;out.faceIndex=-1;
        out.inward.values.set(g.n);out.closestPoint.values.set(p.map((v,k)=>v-g.value*g.n[k]));return out;
    };
    const center=[x,.375,.625],chord=[.1,.3,-.2],positions=[center.map((v,k)=>v-fraction*chord[k]),center.map((v,k)=>v+(1-fraction)*chord[k])];
    const previousPositions=positions.map((p,i)=>p.map((v,k)=>v-.003*(1+i+k)));
    const tool={id:'wire',edgeId:'wire:edge0',edge:0,radius:4,coordinates:[10,13],positions,previousPositions,
        reference:captureCompositeReferenceFrames(previousPositions)[0],previousAngle:.2,angle:.4,trace:'right',
        materialMap:{sStart:20,dsDx:1.3,dsDt:.4},materialPath:{kind:'linear-affine-maps',previousEdgeId:'wire:edge0',previousMap:{sStart:19.992,dsDx:1.3}},
        positionRates:[[.2,-.1,.07],[-.15,.05,.2]],angleRate:2};
    return refresh({dt:.02,tool,current:{field,sdfBranch:{face:{axis:0,gridIndex:1},domainBox:{lower:[.8,.2,.4],upper:[1.2,.6,.8]},branchIndex,fraction}},wall:{motion:'stationary-material',source:'sparse-sdf',rateMode:'backward-euler-grid'}});
}
function refresh(input){
    input.current.row=createCompositeDiscreteWallPoint({fraction:input.current.sdfBranch?.fraction??input.current.row.t}).refresh({field:input.current.field,positions:input.tool.positions,radius:input.tool.radius});
    input.current.row.owner='wire';input.current.positions=structuredClone(input.tool.positions);return input;
}
const evaluate=input=>structuredClone(evaluateCompositeJointWallSurface(input,createCompositeJointWallSurfaceWorkspace()));
import { createCompositeChainLayout } from '../src/physics/kirchhoffCompositeChain.js';
import { createCompositeJointWallRows } from '../src/physics/kirchhoffCompositeJointWallRows.js';
import { createCompositeJointWallFrictionRows } from '../src/physics/kirchhoffCompositeJointWallFrictionRows.js';
const tolerances={force:1e-7,wallGap:1e-8,wallNcp:1e-8,wallWork:1e-8,frictionSlip:1e-8,frictionCone:1e-8,frictionWork:1e-8,frictionEquation:1e-9,linearConstraint:1e-10};
function managerFixture(x=1.001,loadedSecond=false,twoBranch=false,overlap=false){
    const input=fixture(0,1,0),p=[[x,.375,.625],[x,.475,.675],[x,.575,.725]],layout=createCompositeChainLayout(overlap?[['wire','catheter'],['wire','catheter']]:[['wire'],['wire']]);
    if(loadedSecond){const v=model(p[0],x<1?0:1).value,a=model([x,p[1][1],0],x<1?0:1).value,b=model([x,p[1][1],1],x<1?0:1).value;p[1][2]=(v-a)/(b-a);}
    const state={layout,positions:structuredClone(p),coordinates:[0,2,4],modes:[],relativeToolId:'wire',toolPositions:new Map([['wire',p]]),
        angles:new Map([['wire',new Float64Array(2)]]),relative:new Float64Array(0),tools:[{id:'wire',reference:captureCompositeReferenceFrames(p),dsDx:1}]};
    const wall={mode:'wall-coulomb',chartId:'declared-branch-manager',field:input.current.field,forcePerLength:10,
        contactMode:'material-points',pressureDiscretization:'fixed-material-points',contactUpdate:'current-query',
        pressureSites:[{owner:'wire',edge:0,fraction:0,trace:'right'},
            {owner:'wire',edge:0,fraction:1,trace:'left'}],
        contactOwners:{edges:[{edge:0,wall:{owner:'wire',radius:model(p[0],x<1?0:1).value,materialSegmentId:'wire:0'}},{edge:1,wall:null}]},
        friction:{law:'coulomb',mu:[.3,.6],forcePerLength:5,materialPath:'linear-affine-maps',motion:'stationary-material',source:'sparse-sdf',tangentBasis:'projected-own-tangent',
            rateMode:'backward-euler-grid',slipModel:'implicit-backward-euler-surface-rate',finiteStepSlipKnown:false}};
    if(overlap){
        state.modes=p.map((_,node)=>({node,basis:[[0,1,0],[1,0,0],[0,0,1]],relativeDofs:[3*node,3*node+1,3*node+2]}));
        state.relative=new Float64Array(9);state.toolPositions.set('catheter',structuredClone(p));
        state.angles.set('catheter',new Float64Array(2));state.tools.push({id:'catheter',reference:captureCompositeReferenceFrames(p),dsDx:1});
    }
    const candidate=structuredClone(state),normal=createCompositeJointWallRows({...state,wall:{...wall,mode:'wall-normal',friction:'none'},tolerances});
    normal.normalForces.set([2,loadedSecond?4:0]);
    const args={state,candidate,normal,wall,tolerances,dt:.02,normalRowOffset:0,frictionRowOffset:3,
        prepared:{dt:.02,previousPositions:structuredClone(state.toolPositions),inertiaEdges:[{tools:[{id:'wire',materialMap:{sStart:20,dsDx:1,dsDt:0}}]},{tools:[{id:'wire',materialMap:{sStart:22,dsDx:1,dsDt:0}}]}]}};
    if(twoBranch){
        state.elasticityGeometry=candidate.elasticityGeometry='native-discrete-rod';
        delete wall.friction.mu;Object.assign(wall.friction,{law:'coulomb-static-kinetic',muStatic:[.3,.6],muKinetic:[.1,.2],
            incomingRateHistory:captureCompositeNativeRateHistory({previous:state,current:candidate,prepared:args.prepared,dt:.02,rateModel:'endpoint-derivative-of-linear-grid-pose-path'})});
    }
    const friction=createCompositeJointWallFrictionRows(args);
    function refresh(){
        const common=new Float64Array(layout.dofCount),relative=new Float64Array(state.relative.length),toolPositions=candidate.toolPositions;
        const n=normal.refresh({toolPositions,commonResidual:common,relativeResidual:relative,consumeQuery:()=>{}});
        return {normal:n,friction:friction.refresh({toolPositions,commonResidual:common,relativeResidual:relative,order:"full"}),common};
    }
    normal.refresh({toolPositions:candidate.toolPositions,commonResidual:new Float64Array(layout.dofCount),relativeResidual:new Float64Array(state.relative.length),consumeQuery:()=>{}});
    friction.prepare({toolPositions:candidate.toolPositions});
    return {args,state,candidate,normal,friction,refresh};
}

import {captureCompositeNativeRateHistory} from '../src/physics/kirchhoffCompositeNativeRateHistory.js';
import { migrateCompositeJointWallSeamBirth as migrate } from '../src/physics/compositeJointWallSeamBirth.js';
const seam={face:{axis:0,gridIndex:1},domainBox:{lower:[.8,.2,.4],upper:[1.2,.6,.8]}};
function accepted(x,loadedSecond=false,twoBranch=false,overlap=false){const f=managerFixture(x,loadedSecond,twoBranch,overlap);f.friction.tractions.set([.1,-.07,loadedSecond?.03:0,loadedSecond?-.02:0]);f.refresh();
 if(twoBranch)assert.equal(f.friction.resolveModes({wholeStepConverged:true}).accepted,true);
 f.state.wallContactState=f.normal.commit();f.state.wallFrictionState=f.friction.commit();
 return {state:f.state,wall:f.args.wall,prepared:f.args.prepared,dt:f.args.dt,tolerances,siteIndex:0,sdfSeam:structuredClone(seam)};
}
test('seam birth transfers one accepted Fn/Ft to selected ray without altering physical state or histories',()=>{
 const args=accepted(),before=structuredClone(args.state),wallSites=structuredClone(args.wall.pressureSites),result=migrate(args);
 assert.deepEqual(args.state,before);assert.deepEqual(args.wall.pressureSites,wallSites);
 assert.deepEqual(Array.from(result.state.wallContactState.normalForces),[0,2,0]);
 assert.deepEqual(Array.from(result.state.wallFrictionState.tractions),[0,0,.1,-.07,0,0]);
 assert.deepEqual(result.state.toolPositions,args.state.toolPositions);assert.deepEqual(result.state.angles,args.state.angles);
 assert.deepEqual(result.state.wallFrictionState.currentMaps,args.state.wallFrictionState.currentMaps);
 assert.equal(result.diagnostics.physicalReactionPreserved,true);assert.equal(result.diagnostics.timeAdvanced,false);
 assert.equal(result.diagnostics.requiresWorldProofRepreparation,true);
 assert.notEqual(result.state.wallContactState.signature,args.state.wallContactState.signature);
 assert.equal(result.state.wallFrictionState.provenance.length,3);
});
test('failed seam birth is atomic for bad proof, changed maps/history and missing kinetic rate history',()=>{
 const changes=[a=>a.sdfSeam.domainBox.lower[0]=1.01,a=>a.sdfSeam.face.gridIndex=2,
 a=>a.state.wallFrictionState.currentMaps[0].currentMap.sStart+=.01,a=>a.state.wallContactState.signature+='bad',
 a=>a.wall.friction.law='coulomb-static-kinetic',a=>a.siteIndex=9];
 for(const mutate of changes){const a=accepted();mutate(a);const before=structuredClone(a.state),sites=structuredClone(a.wall.pressureSites);
 assert.throws(()=>migrate(a));assert.deepEqual(a.state,before);assert.deepEqual(a.wall.pressureSites,sites);}
});

test('lower selected branch retains the entire prior reaction and never splits it',()=>{
 const args=accepted(.999),result=migrate(args);
 assert.equal(result.diagnostics.selectedBranch,0);
 assert.deepEqual(Array.from(result.state.wallContactState.normalForces),[2,0,0]);
 assert.deepEqual(Array.from(result.state.wallFrictionState.tractions),[.1,-.07,0,0,0,0]);
});

test('unrelated loaded reaction survives index expansion and both branches retain own spin moment',()=>{
 for(const x of [.999,1.001]){const args=accepted(x,true),out=migrate(args),h=out.state.wallContactState,f=out.state.wallFrictionState;
 assert.equal(h.normalForces[2],4);assert.deepEqual(Array.from(f.tractions.slice(4)),[.03,-.02]);
 const old=createCompositeJointWallRows({...args.state,wall:{...args.wall,mode:'wall-normal',friction:'none'},history:args.state.wallContactState,tolerances});
 const m=createCompositeJointWallFrictionRows({state:args.state,candidate:structuredClone(args.state),normal:old,wall:args.wall,prepared:args.prepared,dt:args.dt,tolerances,normalRowOffset:0,frictionRowOffset:old.rows.length});
 const common=new Float64Array(args.state.layout.dofCount),relative=new Float64Array(0),toolPositions=args.state.toolPositions;
 old.refresh({toolPositions,commonResidual:common,relativeResidual:relative,consumeQuery:()=>{}});m.prepare({toolPositions});m.refresh({toolPositions,commonResidual:common,relativeResidual:relative,order:'full'});
 assert.ok(Math.abs(common[args.state.layout.spins.get('wire')[0]])>1e-4);
 }
});

test('static/kinetic keeps the old mode and leaves newborn mode unaccepted for incoming-rate initialization',()=>{
 for(const x of [.999,1.001]){const a=accepted(x,false,true),old=structuredClone(a.state.wallFrictionState.modeHistory),out=migrate(a),selected=x<1?0:1;
 const modes=out.state.wallFrictionState.modeHistory;assert.equal(modes[1-selected],null);
 assert.deepEqual({...modes[selected],sampleId:old[0].sampleId},old[0]);
 assert.deepEqual(a.state.wallFrictionState.modeHistory,old);
 const noRates={...a,wall:{...a.wall,friction:{...a.wall.friction,incomingRateHistory:null}}};assert.throws(()=>migrate(noRates),/incomingRateHistory/);
 }
});

test('all migration queries are budgeted and budget exhaustion leaves inputs unchanged',()=>{
 const a=accepted(),before=structuredClone(a.state);let count=0;const out=migrate({...a,consumeQuery:()=>count++});
 assert.equal(out.diagnostics.preparationContactQueries,count);assert.ok(count>0);
 assert.throws(()=>migrate({...a,consumeQuery:()=>{throw Error('query-budget');}}),/query-budget/);
 assert.deepEqual(a.state,before);
});

test('same-dt migration evaluates private candidate near seam while preserving distant incoming geometry and maps',()=>{
    const args=accepted(.6),candidate=structuredClone(args.state);
    candidate.positions.forEach(p=>p[0]=.999);candidate.toolPositions= new Map([['wire',structuredClone(candidate.positions)]]);candidate.angles.get('wire')[0]=.17;
    const normalForces=Float64Array.of(7,0),tractions=Float64Array.of(.23,-.11,0,0),before=structuredClone({state:args.state,candidate,prepared:args.prepared,normalForces,tractions});
    const out=migrate({...args,candidate,normalForces,tractions});
    assert.equal(out.diagnostics.selectedBranch,0);assert.equal(out.diagnostics.reactionConfiguration,'candidate');
    assert.equal(out.diagnostics.acceptedGeometryCertified,false);
    assert.deepEqual(out.state.toolPositions,args.state.toolPositions);
    assert.deepEqual(out.state.angles,args.state.angles);
    assert.deepEqual(out.acceptedHistories.wallContactState.normalForces,Float64Array.of(2,0,0));
    assert.deepEqual(out.acceptedHistories.wallFrictionState.tractions,Float64Array.of(.1,-.07,0,0,0,0));
    assert.deepEqual(out.trial.normalForces,Float64Array.of(7,0,0));assert.deepEqual(out.trial.tractions,Float64Array.of(.23,-.11,0,0,0,0));
    assert.deepEqual(out.trial.candidate.toolPositions,candidate.toolPositions);assert.deepEqual(out.trial.candidate.angles,candidate.angles);
    assert.deepEqual(out.acceptedHistories.wallFrictionState.currentMaps,args.state.wallFrictionState.currentMaps);
    assert.deepEqual({state:args.state,candidate,prepared:args.prepared,normalForces,tractions},before);
    assert.throws(()=>migrate(args),/Unproved|domain|seam/i,'incoming point alone is outside the new chart');
});
test('invalid private candidate or trial multipliers reject without altering accepted histories',()=>{
    const a=accepted(.6),candidate=structuredClone(a.state);candidate.positions.forEach(p=>p[0]=.999);candidate.toolPositions= new Map([['wire',structuredClone(candidate.positions)]]);
    const before=structuredClone({state:a.state,candidate});
    for(const extra of [{normalForces:[NaN,0]},{tractions:[1,2]},{candidate:{...candidate,coordinates:[0,3,4]}},
        {prepared:{...a.prepared,previousPositions:structuredClone(candidate.toolPositions)}}])assert.throws(()=>migrate({...a,candidate,...extra}));
    assert.deepEqual({state:a.state,candidate},before);
    const signed=migrate({...a,candidate,normalForces:[-1,0]});
    assert.ok(signed.trial.normalForces.includes(-1));assert.deepEqual(signed.state.wallContactState.normalForces.filter(v=>v!==0),a.state.wallContactState.normalForces.filter(v=>v!==0));
});

test('native materialAt callbacks survive migration by identity while numeric state remains owned',()=>{
 const a=accepted(.6),materialAt=()=>({stiffness:1}),candidate=structuredClone(a.state);
 a.state.tools[0].materialAt=candidate.tools[0].materialAt=materialAt;candidate.positions.forEach(p=>p[0]=.999);candidate.toolPositions= new Map([['wire',structuredClone(candidate.positions)]]);
 const out=migrate({...a,candidate});
 assert.equal(out.state.tools[0].materialAt,materialAt);assert.equal(out.trial.candidate.tools[0].materialAt,materialAt);
 assert.notEqual(out.state.toolPositions,a.state.toolPositions);assert.notEqual(out.trial.candidate.angles,candidate.angles);
 candidate.tools[0].materialAt=()=>0;assert.throws(()=>migrate({...a,candidate}),/material callbacks/);
});

test('stale candidate toolPositions cache never overrides canonical common geometry',()=>{
 const a=accepted(.6),candidate=structuredClone(a.state);candidate.positions.forEach(p=>p[0]=.999);
 const stale=structuredClone(candidate.toolPositions),out=migrate({...a,candidate});
 assert.deepEqual(candidate.toolPositions,stale);assert.equal(stale.get('wire')[0][0],.6);
 assert.equal(out.trial.candidate.toolPositions.get('wire')[0][0],.999);
 assert.deepEqual(out.trial.candidate.toolPositions.get('wire'),candidate.positions);
});

test('canonical reconstruction applies relative basis only to the designated tool',()=>{
 const a=accepted(.6,false,false,true),candidate=structuredClone(a.state);
 candidate.positions.forEach(p=>p[0]=.8);for(let node=0;node<3;node++)candidate.relative[3*node+1]=.199;
 const out=migrate({...a,candidate});
 out.trial.candidate.toolPositions.get('wire').forEach(p=>close(p[0],.999,1e-14));
 out.trial.candidate.toolPositions.get('catheter').forEach(p=>close(p[0],.8,1e-14));
 assert.equal(candidate.toolPositions.get('wire')[0][0],.6);
 assert.equal(out.diagnostics.physicalReactionPreserved,true);
});

test('native cold-start migration retains trial reactions without inventing accepted contact history',()=>{
    for(const twoBranch of [false,true]){
        const a=accepted(.6,false,twoBranch);delete a.state.wallContactState;delete a.state.wallFrictionState;
        const candidate=structuredClone(a.state);candidate.positions.forEach(p=>p[0]=.999);
        const normalForces=Float64Array.of(7,0),tractions=Float64Array.of(.23,-.11,0,0),before=structuredClone({state:a.state,candidate});
        const out=migrate({...a,candidate,normalForces,tractions});
        assert.equal(out.diagnostics.coldStart,true);assert.equal(out.state.wallContactState,undefined);assert.equal(out.state.wallFrictionState,undefined);
        assert.deepEqual(out.acceptedHistories,{wallContactState:undefined,wallFrictionState:undefined});
        assert.deepEqual(out.trial.normalForces,Float64Array.of(7,0,0));assert.deepEqual(out.trial.tractions,Float64Array.of(.23,-.11,0,0,0,0));
        assert.deepEqual({state:a.state,candidate},before);
        assert.throws(()=>migrate({...a,candidate,normalForces}),/explicit trial/);
        assert.throws(()=>migrate({...a,candidate,tractions}),/explicit trial/);
        assert.throws(()=>migrate({...a,state:{...a.state,wallContactState:{}},candidate,normalForces,tractions}),/both histories/);
    }
});
