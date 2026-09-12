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
for(const branchIndex of [0,1])test(`explicit SDF branch ${branchIndex} preserves all seven BE G/DB columns across exact seam`,()=>{
    const input=fixture(branchIndex),raw=structuredClone(input.current.row),calls=input.current.field.calls,base=evaluate(input);
    assert.deepEqual(input.current.row,raw);assert.equal(input.current.field.calls,calls);assert.equal(base.queryCount,0);
    assert.equal(base.identity.sdfBranch.branchIndex,branchIndex);assert.equal(base.identity.wallWitness,'proved-sdf-branch-normal-projection');
    const h=1e-6;
    for(let j=0;j<7;j++){
        const values=[1,-1].map(sign=>{const q={...input,tool:structuredClone(input.tool),current:{...input.current}};
            if(j===6)q.tool.angle+=sign*h;else q.tool.positions[Math.floor(j/3)][j%3]+=sign*h;
            return evaluate(refresh(q));});
        for(let c=0;c<2;c++)close((values[0].increment[c]-values[1].increment[c])/(2*h),base.slipJacobian[c*7+j]);
        for(let i=0;i<14;i++)close((values[0].forceMap[i]-values[1].forceMap[i])/(2*h),base.DforceMap[i*7+j]);
    }
});
test('selected explicit branch equals unchanged original one-branch surface on its open cell',()=>{
    for(const [branchIndex,x] of [[0,.95],[1,1.05]]){
        const input=fixture(branchIndex,x),branch=evaluate(input);delete input.current.sdfBranch;const original=evaluate(input);
        for(const key of ['increment','forceMap','DforceMap','slipJacobian','point','normal'])branch[key].forEach((v,k)=>close(v,original[key][k],1e-11));
        assert.equal(original.identity.sdfBranch,undefined);
    }
});
test('explicit branch rejects invalid identity, unproved chart and stale original raw witness',()=>{
    const mutations=[q=>q.current.sdfBranch.branchIndex=2,q=>q.current.sdfBranch.fraction=.2,q=>q.current.row.role='capsule',
        q=>q.current.row.owner='catheter',q=>q.current.row.normal[0]+=.01,q=>q.current.row.rawContact.signedDistance+=.01,
        q=>q.current.sdfBranch.domainBox.lower[0]=1.01,q=>q.current.field.sdfInsideBits.fill(0)];
    for(const mutate of mutations){const q=fixture();mutate(q);assert.throws(()=>evaluate(q));}
    const q=fixture();q.current.row.derivativeUnavailable=true;q.current.row.derivativeReason='cell-boundary';
    q.current.row.gapJacobian.fill(NaN);q.current.row.forceColumn.fill(NaN);q.current.row.normalDerivative.fill(NaN);
    assert.equal(evaluate(q).supported,true);
    delete q.current.sdfBranch;assert.throws(()=>evaluate(q));
});

import { createCompositeChainLayout } from '../src/physics/kirchhoffCompositeChain.js';
import { createCompositeJointWallRows } from '../src/physics/kirchhoffCompositeJointWallRows.js';
import { createCompositeJointWallFrictionRows } from '../src/physics/kirchhoffCompositeJointWallFrictionRows.js';
const tolerances={force:1e-7,wallGap:1e-8,wallNcp:1e-8,wallWork:1e-8,frictionSlip:1e-8,frictionCone:1e-8,frictionWork:1e-8,frictionEquation:1e-9,linearConstraint:1e-10};
function managerFixture(){
    const input=fixture(0,1,0),p=[[1,.375,.625],[1,.475,.675],[1,.575,.725]],layout=createCompositeChainLayout([['wire'],['wire']]);
    const state={layout,coordinates:[0,2,4],modes:[],relativeToolId:'wire',toolPositions:new Map([['wire',p]]),
        angles:new Map([['wire',new Float64Array(2)]]),relative:new Float64Array(0),tools:[{id:'wire',reference:captureCompositeReferenceFrames(p),dsDx:1}]};
    const wall={mode:'wall-coulomb',chartId:'declared-branch-manager',field:input.current.field,forcePerLength:10,
        contactMode:'material-points',pressureDiscretization:'fixed-material-points',contactUpdate:'current-query',
        pressureSites:[{owner:'wire',edge:0,fraction:0,trace:'right',sdfSeam:{face:{axis:0,gridIndex:1},domainBox:input.current.sdfBranch.domainBox}},
            {owner:'wire',edge:0,fraction:1,trace:'left'}],
        contactOwners:{edges:[{edge:0,wall:{owner:'wire',radius:model(p[0],1).value,materialSegmentId:'wire:0'}},{edge:1,wall:null}]},
        friction:{law:'coulomb',mu:[.3,.6],forcePerLength:5,materialPath:'linear-affine-maps',motion:'stationary-material',source:'sparse-sdf',tangentBasis:'projected-own-tangent',
            rateMode:'backward-euler-grid',slipModel:'implicit-backward-euler-surface-rate',finiteStepSlipKnown:false}};
    const candidate=structuredClone(state),normal=createCompositeJointWallRows({...state,wall:{...wall,mode:'wall-normal',friction:'none'},tolerances});
    normal.normalForces.set([2,3,0]);
    const args={state,candidate,normal,wall,tolerances,dt:.02,normalRowOffset:0,frictionRowOffset:3,
        prepared:{dt:.02,previousPositions:structuredClone(state.toolPositions),inertiaEdges:[{tools:[{id:'wire',materialMap:{sStart:20,dsDx:1,dsDt:0}}]},{tools:[{id:'wire',materialMap:{sStart:22,dsDx:1,dsDt:0}}]}]}};
    const friction=createCompositeJointWallFrictionRows(args);
    function refresh(){
        const common=new Float64Array(layout.dofCount),relative=new Float64Array(0),toolPositions=candidate.toolPositions;
        const n=normal.refresh({toolPositions,commonResidual:common,relativeResidual:relative,consumeQuery:()=>{}});
        return {normal:n,friction:friction.refresh({toolPositions,commonResidual:common,relativeResidual:relative,order:"full"}),common};
    }
    normal.refresh({toolPositions:candidate.toolPositions,commonResidual:new Float64Array(layout.dofCount),relativeResidual:new Float64Array(0),consumeQuery:()=>{}});
    friction.prepare({toolPositions:candidate.toolPositions});
    return {args,state,candidate,normal,friction,refresh};
}
test('declared seam normal and Coulomb managers retain two reactions, own spin moments and rollback provenance',()=>{
    const f=managerFixture(),{normal,friction,state,candidate}=f;
    assert.equal(normal.rows.length,3);assert.equal(friction.rows.length,6);
    assert.deepEqual(normal.surfaceRecords.map(r=>r.sdfBranch?.branchIndex),[0,1,undefined]);
    const initial=f.refresh();assert.equal(initial.normal.converged,true);assert.equal(initial.friction.converged,true);
    assert.deepEqual(initial.friction.samples.map(s=>s.Fn),[2,3,0]);
    const normalHistory=normal.commit(),frictionHistory=friction.commit(),nt=normal.checkpoint(),ft=friction.tractions.slice();
    friction.tractions.set([.1,-.07,-.05,.09,0,0]);
    const loaded=f.refresh(),sum=new Float64Array(state.layout.dofCount);
    for(let i=0;i<normal.rows.length;i++){
        const row=normal.rows[i];for(let j=0;j<row.commonDofs.length;j++)sum[row.commonDofs[j]]+=row.forceColumn[j]*normal.normalForces[normal.rowForceIndices[i]];
    }
    for(let i=0;i<friction.rows.length;i++){
        const row=friction.rows[i];for(let j=0;j<row.commonDofs.length;j++)sum[row.commonDofs[j]]+=row.forceColumn[j]*friction.tractions[i];
    }
    sum.forEach((v,k)=>close(v,loaded.common[k],1e-12));
    const spin=state.layout.spins.get('wire')[0];assert.ok(Math.abs(loaded.common[spin])>1e-4,'branch wall levers retain own torque');
    normal.normalForces[0]=8;friction.tractions.fill(5);normal.restore(nt);friction.tractions.set(ft);
    f.refresh();assert.deepEqual(normal.commit(),normalHistory);assert.deepEqual(friction.commit(),frictionHistory);
    const next={...state,wallState:normalHistory,wallFrictionState:frictionHistory};
    const other=createCompositeJointWallRows({...next,history:normalHistory,wall:{...f.args.wall,mode:'wall-normal',friction:'none'},tolerances});
    assert.deepEqual(other.normalForces,normal.normalForces);
    const rebuilt=createCompositeJointWallFrictionRows({...f.args,state:next,candidate:structuredClone(next),normal:other});
    assert.equal(rebuilt.signature,friction.signature);assert.deepEqual(rebuilt.tractions,friction.tractions);
    const badWall=structuredClone({...f.args.wall,field:null});badWall.field=f.args.wall.field;
    badWall.pressureSites[0].sdfSeam.domainBox.lower[0]-=.01;
    assert.throws(()=>createCompositeJointWallRows({...state,history:normalHistory,wall:{...badWall,mode:'wall-normal',friction:'none'},tolerances}),/history|provenance/);
    f.args.wall.pressureSites[0].sdfSeam.face.gridIndex=2;
    assert.throws(()=>f.refresh(),/changed|contract|signature/i);
});
