import fs from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {Ray,Vector3,DoubleSide} from 'three';
import {restoreSharedAxisReplay} from '../../src/physics/kirchhoffSharedAxisRestore.js';
import {advanceSharedAxis} from '../../src/physics/kirchhoffSharedAxisAppSystem.js';
import {captureSharedAxisNative} from '../../src/physics/kirchhoffSharedAxisNative.js';
import {sampleCompliantBalloon} from '../../src/devices/compliantBalloon.js';
import {getCompositeJointRenderPath} from '../../src/compositeJointRenderPath.js';
import {DEFAULT_VESSEL_ELASTICITY} from '../../src/physics/vesselElasticity.js';
import {RELIANT} from '../../src/devices/balloonProfile.js';

export function readBalloonRuntimeReplay() {
    return JSON.parse(gunzipSync(fs.readFileSync(new URL('../fixtures/shared-axis/reliant-inflation-iliac.json.gz',import.meta.url))));
}
export function balloonReplayView(state) {
    const material=state.materials.find(m=>m.spec.id==='catheter');
    return {coordinates:material.coordinates.slice(),positions:Float64Array.from(state.positions.slice(0,material.last+1)
        .flatMap(p=>p.map((v,k)=>v+state.origin[k])))};
}
// The previous unbounded ray path is deliberately retained as an independent
// reference. Compare the entire surface, not merely its integrated volume.
export function referenceBalloonProjection(view,tree,volumeMl,deformation=null) {
    const ray=new Ray(),displacement=new Vector3();
    return sampleCompliantBalloon({path:getCompositeJointRenderPath(view),coordinates:view.coordinates,volumeMl,
        wallDistance:(center,direction)=>{
            ray.origin.copy(center);ray.direction.copy(direction);
            const hit=tree.raycastFirst(ray,DoubleSide,0,100);if(!hit)return NaN;
            if(!deformation)return hit.distance;
            deformation.displacementAt(hit.point,displacement);
            const clearance=hit.distance+Math.max(0,displacement.dot(direction));
            return displacement.lengthSq()>1e-12&&clearance>=RELIANT.shaftRadius-DEFAULT_VESSEL_ELASTICITY.maxDisplacement-1e-3
                ?Math.max(RELIANT.shaftRadius,clearance):clearance;
        }});
}
export function runBalloonRuntimeReplay(fixture,field,{optimized=true,fills=[12.6,12.7,12.8,12.7,12.6,12.5,12.4,0]}={}) {
    const request=fixture.stepRequest,steps=[],poses=[];
    let state=restoreSharedAxisReplay(fixture,field),rotations=request.rotations;
    const scratch=field._scalarBvhScratch,previousReuse=scratch.reuseExactQuery;
    scratch.reuseExactQuery=optimized;scratch.lastExactQuery=null;
    try {
        for(const fill of fills) {
            const tools=request.tools.map(t=>({...t,...(t.id==='catheter'?{balloonInflationMl:fill}:{})}));
            const start=performance.now(),it=advanceSharedAxis(state,rotations,request.dt,tools,
                {...request.options,reuseContactCoverage:optimized});let next;
            do {
                next=it.next();
                if(!next.done&&performance.now()-start>15000){it.return();throw new Error('Replay exceeded work limit');}
            }while(!next.done);
            const wallMs=performance.now()-start,r=next.value.result;
            if(!next.value.state)throw new Error(JSON.stringify(r));
            state=next.value.state;rotations=next.value.rotations;
            steps.push({wallMs,fill,iterations:r.iterations,subdivisions:r.subdivisions,certificateBound:r.certificateBound,
                residual:r.residual,quality:r.quality});
            const {geometryKey,...pose}=captureSharedAxisNative(state);
            poses.push({...pose,velocities:structuredClone(state.velocities),angularVelocities:structuredClone(state.angularVelocities),
                patches:structuredClone(state.vesselElasticPatches)});
            if(state.contactCoverageCache!=null)throw new Error('Coverage scratch survived its private timestep');
        }
        return {state,steps,poses};
    }finally{scratch.reuseExactQuery=previousReuse;}
}
