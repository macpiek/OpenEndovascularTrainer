import test from 'node:test';
import assert from 'node:assert/strict';
import {vesselElasticResponse,DEFAULT_VESSEL_ELASTICITY} from '../src/physics/vesselElasticity.js';
import {createSharedAxisNative,captureSharedAxisNative,extendSharedAxisNativeRows} from '../src/physics/kirchhoffSharedAxisNative.js';
import {stepSharedAxis,iterateSharedAxisTimeStep} from '../src/physics/kirchhoffSharedAxisTimeStep.js';
import {BufferGeometry,Float32BufferAttribute} from 'three';
import {defineKirchhoffMaterialProfile} from '../src/physics/kirchhoffMaterialProfile.js';
import {createSharedAxisVesselDiscovery,relaxSharedAxisWithContacts} from '../src/physics/kirchhoffSharedAxisVesselWitnesses.js';

const options={vesselElasticity:{compliance:.001,maxDisplacement:.3},reuseRowBuffers:true,reuseConstraintWork:true,
    reuseMatrixAssembly:true,wasmLinearAssembly:true,projectionMode:'reduced',reuseFrictionAssembly:true,lengthTolerance:1e-10};
function fixture(depth=.02) {
    const s=createSharedAxisNative({tools:[{id:'wire',insertion:20,radius:.45}]});
    s.fixed.fill(1);
    const edge=s.positions.length-2,dofs=[s.layout.positions[edge],s.layout.positions[edge+1]].flatMap(p=>[p,p+1,p+2]);
    extendSharedAxisNativeRows(s,[{kind:'wall',id:'elastic-test-plane',edge,dofs,witness:{face:0,t:1},vesselElasticContact:true,
        evaluate:({b})=>({gap:b[1]-depth,jacobian:[0,0,0,0,1,0]})}]);
    return s;
}

test('bounded vessel wall has a consistent positive tangent and elastic energy',()=>{
    const {compliance:c,maxDisplacement:limit}=DEFAULT_VESSEL_ELASTICITY;
    for(const force of [0,1,1e4,1e6,1e8,1e12]) {
        const r=vesselElasticResponse(force,c,limit);
        assert.ok(r.displacement>=0&&r.displacement<limit);
        assert.ok(r.tangent>0&&r.tangent<=c&&r.energy>=0);
        if(!force)continue;
        const h=force*1e-4,lo=vesselElasticResponse(force-h,c,limit),hi=vesselElasticResponse(force+h,c,limit);
        assert.ok(Math.abs((hi.displacement-lo.displacement)/(2*h)/r.tangent-1)<1e-6);
        assert.ok(Math.abs((hi.energy-lo.energy)/(2*h)/(force*r.tangent)-1)<1e-6);
    }
});

test('full elastic timestep matches analytic wall reaction, publishes deformation and recovers on unloading',()=>{
    const s=fixture();
    const result=stepSharedAxis(s,1/60,options);
    assert.ok(result.converged,JSON.stringify(result));
    const expected=.02/(.001*(1-.02/.3));
    assert.ok(Math.abs(s.multipliers.at(-1)-expected)<1e-6);
    assert.ok(result.quality.maxPenetration<1e-6);
    assert.equal(result.quality.maxReferencePenetration,.02);
    assert.ok(Math.abs(result.quality.maxWallDisplacement-.02)<1e-6);
    assert.equal(s.acceptedWallGaps.get('elastic-test-plane'),-.02);
    assert.equal(s.vesselElasticPatches.length,1);
    const patch=s.vesselElasticPatches[0];
    assert.ok(Math.abs(patch.displacement-.02)<1e-6);
    assert.deepEqual(patch.normal,[-0,-1,-0]);
    assert.ok(Math.abs(patch.center[1]-(.02-.45))<1e-10);
    assert.equal(s.vesselElasticity,undefined);
    // Move the wall away without changing the tool or retained warm-start load.
    s.definitions.at(-1).evaluate=()=>({gap:.1,jacobian:[0,0,0,0,1,0]});
    const unloaded=stepSharedAxis(s,1/60,options);
    assert.ok(unloaded.converged,JSON.stringify(unloaded));
    assert.deepEqual(s.vesselElasticPatches,[]);
});

test('elastic timestep cancellation restores mechanics and retains the last displayed wall',()=>{
    const s=fixture();s.vesselElasticPatches=[{accepted:true}];
    const patches=s.vesselElasticPatches,before=captureSharedAxisNative(s);
    const iterator=iterateSharedAxisTimeStep(s,1/60,options);
    assert.equal(iterator.next().done,false);iterator.return();
    assert.deepEqual(captureSharedAxisNative(s),before);
    assert.equal(s.vesselElasticPatches,patches);assert.equal(s.vesselElasticity,undefined);
});

test('vessel elasticity does not soften sheath contacts or permit unlimited wall travel',()=>{
    const rigid=fixture();rigid.definitions.at(-1).vesselElasticContact=false;
    assert.equal(stepSharedAxis(rigid,1/60,options).converged,false);
    const excessive=fixture(.4);
    assert.equal(stepSharedAxis(excessive,1/60,{...options,maxIterations:8}).converged,false);
    assert.equal(excessive.vesselElasticPatches,undefined);
});

test('wall compliance changes the actual free rod equilibrium and stronger contact bends the wall further',()=>{
    const geometry=new BufferGeometry().setAttribute('position',new Float32BufferAttribute([-1000,1,-1000,1000,1,-1000,0,1,1000],3));
    const field={fallbackGeometry:geometry,voxelSize:1,queryCapsuleSoA(x,y,z,r,i,out){
        out.segmentT=1;out.faceIndex=0;out.signedDistance=1-y[1];out.signedGap=out.signedDistance-r[0];
        out.inward.x=0;out.inward.y=-1;out.inward.z=0;return out;
    }};
    const beam=defineKirchhoffMaterialProfile({id:'elastic-wall-beam',sampleEI1:()=>1e5,sampleGJ:()=>1e5});
    const run=(load,elastic)=>{
        const s=createSharedAxisNative({tools:[{id:'wire',insertion:30,type:beam,radius:.4}],wallSamples:[createSharedAxisVesselDiscovery(field,0)]});
        if(elastic)s.vesselElasticity=options.vesselElasticity;
        s.loads[s.layout.positions.at(-1)+1]=load;
        const result=relaxSharedAxisWithContacts(s);
        assert.ok(result.converged,JSON.stringify(result));
        assert.ok(result.quality.maxPenetration<1e-5);
        return s.positions.at(-1)[1];
    };
    try {
        const rigid=run(20,false),soft=run(20,true),loaded=run(40,true);
        assert.ok(Math.abs(rigid-.6)<1e-5);
        assert.ok(soft>rigid+.005&&loaded>soft+.005);
        assert.ok(loaded<.6+.3);
    }finally{geometry.dispose();}
});
