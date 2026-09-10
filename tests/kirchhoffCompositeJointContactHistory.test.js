import test from 'node:test';
import assert from 'node:assert/strict';
import {createCompositeChainLayout} from '../src/physics/kirchhoffCompositeChain.js';
import {captureCompositeReferenceFrames} from '../src/physics/kirchhoffCompositeElement.js';
import {createCompositeJointLumenRows} from '../src/physics/kirchhoffCompositeJointLumenRows.js';
import {createCompositeJointLumenFrictionRows} from '../src/physics/kirchhoffCompositeJointLumenFrictionRows.js';
const tolerances={force:1e-7,lumenGap:1e-8,lumenNcp:1e-8,lumenWork:1e-8,frictionSlip:1e-8,frictionCone:1e-8,frictionWork:1e-8,frictionEquation:1e-9,linearConstraint:1e-10};
function fixture({birth=false,history=null,change=()=>{}}={}) {
    const dt=.02,coordinates=[0,2,4],layout=createCompositeChainLayout([['wire','catheter'],['wire','catheter']]),
        modes=coordinates.map((_,node)=>({node,basis:[[.8,.6,0],[-.6,.8,0],[0,0,1]],relativeDofs:[3*node,3*node+1,3*node+2]})),
        toolPositions=new Map([['catheter',coordinates.map(x=>[x,0,0])],['wire',coordinates.map(x=>[x+.125,.25,0])]]),
        state={layout,coordinates,modes,relativeToolId:'wire',toolPositions,angles:new Map([['wire',new Float64Array(2)],['catheter',new Float64Array(2)]]),
            relative:new Float64Array(9),tools:[...toolPositions].map(([id,p])=>({id,reference:captureCompositeReferenceFrames(p),dsDx:1})),
            ...(history?{lumenFrictionState:structuredClone(history.friction)}:{})},candidate=structuredClone(state),
        pair=(edge,quadrature)=>({id:`pair${edge}`,innerToolId:'wire',outerToolId:'catheter',innerEdge:edge,outerEdge:edge,
            innerMaterialSegmentId:`wire:${edge}`,outerMaterialSegmentId:`catheter:${edge}`,lumenRadius:.5,innerRadius:.25,quadrature,openDistal:false,portalFilletRadius:0}),
        contacts={mode:'lumen-coulomb',chartId:'birth-regression',historyUpdate:'preserve-and-append',forcePerLength:1,
            friction:{law:'coulomb',mu:[.3,.6],muByPair:{pair0:[.3,.6],...(birth?{pair1:[.2,.4]}:{})},forcePerLength:5,materialPath:'linear-affine-maps',
                rateMode:'backward-euler-grid',slipModel:'implicit-backward-euler-surface-rate',finiteStepSlipKnown:false},
            // Deliberately insert the new pair BEFORE the old pair and a new
            // quadrature point between old points: array indices must move.
            pairs:birth?[pair(1,[.25]),pair(0,[.25,.5,.75])]:[pair(0,[.25,.75])]},
        prepared={dt,previousPositions:structuredClone(toolPositions),inertiaEdges:layout.edgeToolIds.map((ids,e)=>({tools:ids.map(id=>({id,
            materialMap:{sStart:20+2*e+(id==='catheter'?10:0),dsDx:1,dsDt:0}}))}))};
    change({contacts,prepared,state});
    const normal=createCompositeJointLumenRows({...state,contacts:{...contacts,mode:'lumen-normal',friction:'none'},
        history:history?.normal??null,tolerances,preserveSampleReactions:true});
    normal.prepareGauge({toolPositions,consumeQuery:()=>{}});
    const friction=createCompositeJointLumenFrictionRows({state,candidate,prepared,normal,contacts,dt,tolerances,normalRowOffset:0,frictionRowOffset:normal.rows.length});
    friction.prepare({consumeQuery:()=>{}});
    return {state,candidate,contacts,normal,friction,prepared};
}
function refresh(f) {
    const options={toolPositions:f.candidate.toolPositions,commonResidual:new Float64Array(f.state.layout.dofCount),relativeResidual:new Float64Array(9),order:'full',consumeQuery:()=>{}};
    const normal=f.normal.refresh(options),friction=f.friction.refresh(options);
    assert.equal(normal.converged,true);assert.equal(friction.converged,true);
    return {normal:f.normal.commit(),friction:f.friction.commit()};
}
function loadedHistory() {
    const f=fixture();f.normal.normalForces.set([1.5,2]);f.friction.tractions.set([.1,-.15,-.12,.2]);return refresh(f);
}
test('BE contact birth preserves every loaded material sample Fn/Ft and own maps when new unknowns change array positions',()=>{
    const history=loadedHistory(),before=structuredClone(history),next=fixture({birth:true,history});
    assert.deepEqual(Array.from(next.normal.normalForces),[0,1.5,0,2]);
    assert.deepEqual(Array.from(next.friction.tractions),[0,0,.1,-.15,0,0,-.12,.2]);
    assert.deepEqual(next.normal.historyAddition.sourceIndices,[-1,0,-1,1]);
    const accepted=refresh(next);
    assert.equal(accepted.friction.currentMaps.length,4);
    for(const old of history.friction.currentMaps)assert.deepEqual(accepted.friction.currentMaps.find(m=>m.id===old.id&&m.edge===old.edge),old);
    assert.deepEqual(history,before);
    const repeated=fixture({birth:true,history:accepted});
    assert.deepEqual(refresh(repeated),accepted);
});
test('contact birth cannot hide lost samples, changed old radii, coefficients or own material history',()=>{
    const history=loadedHistory(),before=structuredClone(history);
    for(const change of [
        ({contacts})=>contacts.pairs[1].quadrature.splice(0,1),
        ({contacts})=>contacts.pairs[1].innerRadius+=.001,
        ({contacts})=>contacts.pairs[1].innerMaterialSegmentId='other-wire-material',
        ({contacts})=>contacts.friction.muByPair.pair0=[.31,.6],
        ({prepared})=>prepared.inertiaEdges[0].tools[0].materialMap.sStart+=.001,
        ({state})=>state.lumenFrictionState.currentMaps.pop(),
        ({contacts})=>delete contacts.historyUpdate,
        ({contacts})=>{delete contacts.friction.rateMode;delete contacts.friction.slipModel;delete contacts.friction.finiteStepSlipKnown;},
    ])assert.throws(()=>fixture({birth:true,history,change}),/provenance|history|map|coefficient/);
    assert.deepEqual(history,before);
});
