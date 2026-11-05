import test from 'node:test';
import assert from 'node:assert/strict';
import {
    GUIDEWIRE_TYPE_LUNDERQUIST as TYPE, LUNDERQUIST_LENGTH_MM,
    LUNDERQUIST_CURVED_LENGTH_MM, guidewireMaterialProfile,
    sampleGuidewireRestCenterline
} from '../src/physics/guidewireMaterialProfile.js';
import {kirchhoffMaterialProfile, discretizeKirchhoffProfile} from '../src/physics/kirchhoffMaterialProfile.js';
import {createSharedAxisNative, feedSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
import {buildGuidewireTipPreviewPath} from '../src/ui/guidewireTipPreview.js';
import {GUIDEWIRE_LENGTHS_MM} from '../src/devices/introducerSheath.js';

const near = (a,b,tolerance=1e-10) => assert.ok(Math.abs(a-b)<tolerance, `${a} != ${b}`);

test('Lunderquist has an extra-stiff shaft but retains a soft 4 cm tip and smooth taper',()=>{
    const legacy=guidewireMaterialProfile(TYPE), p=kirchhoffMaterialProfile(TYPE);
    assert.equal(legacy.id,TYPE); assert.equal(p.id,TYPE);
    assert.equal(legacy.tipCoreLength,40); assert.equal(legacy.tipTransitionLength,110);
    near(p.sample(500).EI1/kirchhoffMaterialProfile('steel-j-035').sample(500).EI1,4);
    near(p.sample(500).EI1/kirchhoffMaterialProfile('glidewire').sample(500).EI1,8);
    near(p.sample(0).EI1,kirchhoffMaterialProfile('glidewire').sample(0).EI1);
    near(p.sample(40).EI1,p.sample(0).EI1);
    let previous=p.sample(40).EI1;
    for(let s=41;s<=150;s++) { const value=p.sample(s).EI1; assert.ok(value>previous); previous=value; }
    near(previous,p.sample(500).EI1);
});

test('double curve uses distinct 75/15 mm radii and smooth transitions to the straight shaft',()=>{
    const p=guidewireMaterialProfile(TYPE), length=p.naturalArcLength;
    near(p.intrinsicCurvature(20),1/15);
    near(p.intrinsicCurvature(100),1/75);
    assert.equal(p.intrinsicCurvature(length+1),0);
    for(const s of [15*Math.PI-4,15*Math.PI+4,length-10,length])
        assert.ok(Math.abs(p.intrinsicCurvature(s-1e-5)-p.intrinsicCurvature(s+1e-5))<1e-7);
    near(p.integrateIntrinsicTurn(length/2,length),2*Math.PI);
    near(p.integrateIntrinsicTurn(500,100),0);
    near(p.integrateIntrinsicTurn(0,0),0);
    const sample=sampleGuidewireRestCenterline(TYPE,length+20,length+20);
    near(sample.turnAngle,2*Math.PI,1e-6);
    assert.ok(sample.normalDistance>100 && sample.normalDistance<140,'broad aortic support arc');
    assert.notEqual(buildGuidewireTipPreviewPath(TYPE),buildGuidewireTipPreviewPath('steel-j-035'));
});

test('manufactured curve integral is preserved through coarse and irregular remeshing',()=>{
    const length=LUNDERQUIST_CURVED_LENGTH_MM, p=kirchhoffMaterialProfile(TYPE);
    for(const coordinates of [[0,length], [0,2,7,40,90,150,length],
        Array.from({length:150},(_,i)=>length*i/149)]) {
        let sum=0;
        for(let i=1;i<coordinates.length;i++) sum+=p.integrate(coordinates[i-1],coordinates[i]).kappa01Integral;
        near(sum,2*Math.PI);
    }
    // Actual rod discretization covers Voronoi cells including both endpoints.
    for(const count of [5,31,100]) {
        const x=Float64Array.from({length:count},(_,i)=>length*i/(count-1));
        const discrete=discretizeKirchhoffProfile(TYPE,x,length);
        near(discrete.restRotation1.reduce((a,b)=>a+b,0),2*Math.PI);
    }
});

test('Lunderquist can advance to 260 cm with independent shaft and tip scaling',()=>{
    assert.ok(GUIDEWIRE_LENGTHS_MM.includes(LUNDERQUIST_LENGTH_MM));
    const state=createSharedAxisNative({tools:[{id:'wire',type:TYPE,length:LUNDERQUIST_LENGTH_MM,
        insertion:LUNDERQUIST_LENGTH_MM,shaftStiffness:13.15,tipStiffness:19.4}]});
    assert.equal(state.coordinates.at(-1),2600);
    assert.equal(state.materials[0].spec.type,TYPE);
    const body=state.materials[0].body;
    assert.ok(Array.from(body.kirchhoffBendCompliance1).every(Number.isFinite));
    assert.ok(body.kirchhoffBendCompliance1[2]<body.kirchhoffBendCompliance1[body.count-3]/50);
});

test('triaxial feeding retains the Lunderquist profile when guidewire support is withdrawn',()=>{
    const state=createSharedAxisNative({tools:[
        {id:'wire',type:TYPE,length:2600,insertion:400,shaftStiffness:13.15,tipStiffness:19.4},
        {id:'catheter',type:'berenstein',length:1000,insertion:250},
        {id:'sheath',type:'introducer-sheath',length:450,insertion:200}
    ]});
    const next=feedSharedAxisNative(state,{wire:100});
    assert.equal(next.materials[0].spec.type,TYPE);
    assert.equal(next.materials[0].spec.insertion,100);
    assert.equal(next.materials[1].spec.insertion,250);
});
