import {relativeSearchFixture,relativeSearchOptions} from './compositeRelativeSearch.js';
import {createCompositeChainLayout} from '../../src/physics/kirchhoffCompositeChain.js';
import {captureCompositeReferenceFrames} from '../../src/physics/kirchhoffCompositeElement.js';
import {createCompositeJointTimeStepState} from '../../src/physics/kirchhoffCompositeJointTimeStep.js';

export {relativeSearchOptions as mechanicalMeshOptions};
export const gradedMechanicalNodes=[0,1,2,3,4,5,6,7,8,12,16,24,32,48,64];

// SAME synthetic initial material fields, total length, load location and
// 128 physical contact sites for every mechanical grid. This is not a remap
// of accepted frames/forces/history and is not an anatomy calibration.
export function mechanicalMeshFixture(nodes=Array.from({length:65},(_,i)=>i)) {
    if(nodes.length<3||nodes[0]!==0||nodes.at(-1)!==64||nodes.some((v,i)=>!Number.isInteger(v)||(i&&v<=nodes[i-1])))throw new RangeError('An ordered subset of nodes 0..64 with both endpoints is required');
    const full=relativeSearchFixture({n:65,lumen:true,offset:.0406,wireSlope:-.0002}),old=full.state,
        coordinates=nodes.map(i=>old.coordinates[i]),positions=nodes.map(i=>old.positions[i]),layout=createCompositeChainLayout(coordinates.slice(1).map(()=>['wire','catheter']));
    const toolPositions=new Map([...old.toolPositions].map(([id,p])=>[id,nodes.map(i=>p[i])]));
    const tools=old.tools.map(t=>({...t,reference:captureCompositeReferenceFrames(toolPositions.get(t.id)),referenceTwists:new Float64Array(nodes.length-2)}));
    const state=createCompositeJointTimeStepState({layout,coordinates,positions,modes:nodes.map((_,node)=>({node,basis:[[1,0,0],[0,1,0],[0,0,1]]})),
        relative:toolPositions.get('wire').flatMap((p,i)=>p.map((v,k)=>v-positions[i][k])),tools,
        angles:new Map(tools.map(t=>[t.id,new Float64Array(nodes.length-1)])),restLengths:new Map(tools.map(t=>[t.id,Float64Array.from(coordinates.slice(1),(x,i)=>(x-coordinates[i])*t.dsDx)])),materialCoordinate:'reference-arclength'});
    const pairs=coordinates.slice(1).map((end,edge)=>({...full.contacts.pairs[0],id:`mechanical:${edge}`,innerEdge:edge,outerEdge:edge,
        innerMaterialSegmentId:`wire:${edge}`,outerMaterialSegmentId:`catheter:${edge}`,
        quadrature:full.contacts.pairs.flatMap((p,i)=>old.coordinates[i]>=coordinates[edge]&&old.coordinates[i+1]<=end?
            p.quadrature.map(u=>(old.coordinates[i]+u*(old.coordinates[i+1]-old.coordinates[i])-coordinates[edge])/(end-coordinates[edge])):[])}));
    return {state,contacts:{...full.contacts,pairs}};
}

export function mechanicalMeshErrors(reference,candidate) {
    if(!reference.accepted||!candidate.accepted)throw new RangeError('Both original whole steps must be accepted before comparing mesh errors');
    let position=0,reaction=0;
    const coarse=candidate.state.coordinates;
    for(const [id,p] of reference.state.toolPositions){let edge=0;for(let i=0;i<p.length;i++){
        const x=reference.state.coordinates[i];while(edge<coarse.length-2&&x>coarse[edge+1])edge++;
        const u=(x-coarse[edge])/(coarse[edge+1]-coarse[edge]),a=candidate.state.toolPositions.get(id)[edge],b=candidate.state.toolPositions.get(id)[edge+1];
        position=Math.max(position,Math.hypot(...p[i].map((v,k)=>v-a[k]-u*(b[k]-a[k]))));
    }}
    for(const [key,field] of [['lumenContactState','normalForces'],['lumenFrictionState','tractions']]){
        const a=reference.state[key][field],b=candidate.state[key][field];if(a.length!==b.length)throw new RangeError('A mesh must retain all independent physical contact samples');
        a.forEach((v,i)=>reaction=Math.max(reaction,Math.abs(v-b[i])));
    }
    return {position,reaction};
}
