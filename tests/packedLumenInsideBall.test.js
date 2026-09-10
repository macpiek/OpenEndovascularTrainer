import test from 'node:test';
import assert from 'node:assert/strict';
import {PackedLumenField} from '../src/physics/collision/packedLumenField.js';

function squareField({halfWidths=[2,2],centers=[0,0],bases=[1,0,0,0,1,0,0,0,1],sliceYs=[-2,2]}={}) {
    const points=[],bounds=[];
    halfWidths.forEach((h,i)=>{
        const c=centers[i];points.push(c-h,-h,c+h,-h,c+h,h,c-h,h);bounds.push(c-h,c+h,-h,h);
    });
    return new PackedLumenField({lumen:{}},{lumenSliceYs:Float64Array.from(sliceYs),lumenSliceContourOffsets:Uint32Array.of(0,1,2),
        lumenContourPointOffsets:Uint32Array.of(0,4,8),lumenContourBounds:Float64Array.from(bounds),lumenContourSamples:new Float64Array(),
        lumenPoints:Float64Array.from(points),lumenAxisBases:Float64Array.from(bases),lumenAxisSliceOffsets:Uint32Array.of(0,2)});
}
const hints=f=>({lower:Array.from(f._lastLower),upper:Array.from(f._lastUpper),interval:Array.from(f._interval),
    lowerScratch:Array.from(f._lower),upperScratch:Array.from(f._upper)});

test('inside ball uses both slice distances and leaves existing query state untouched',()=>{
    const f=squareField();f.queryCoordinates(.1,-1,.2);const before=hints(f),proof=f.certifyInsideBallCoordinates(0,0,0,.25);
    assert.equal(proof.supported,true);assert.equal(proof.axis,0);assert.equal(proof.lowerDistance,2);assert.equal(proof.upperDistance,2);
    assert.equal(proof.localXZRadius,.25*Math.sqrt(2));assert.ok(proof.minimumMargin>0);assert.deepEqual(hints(f),before);
    for(const v of [[.25,0,0],[-.25,0,0],[0,.25,0],[0,-.25,0],[0,0,.25],[0,0,-.25]])assert.equal(f.isInsideCoordinates(...v),true);
});

test('ball crossing a slab boundary or contour is not certified',()=>{
    const f=squareField();
    assert.equal(f.certifyInsideBallCoordinates(0,1.9,0,.2).supported,false);
    assert.equal(f.certifyInsideBallCoordinates(3,0,0,.1).supported,false);
    assert.equal(f.certifyInsideBallCoordinates(1.95,0,0,.1).supported,false);
    assert.equal(f.certifyInsideBallCoordinates(0,3,0,.1).supported,false,'constant extrapolation is outside this finite-slab proof');
});

test('positive interpolated point sign cannot substitute for positive clearance on both slices',()=>{
    const f=squareField({halfWidths:[2,.4],centers:[0,3]});assert.equal(f.isInsideCoordinates(0,-1.5,0),true);
    const before=hints(f),proof=f.certifyInsideBallCoordinates(0,-1.5,0,.1);
    assert.equal(proof.supported,false);assert.equal(proof.reason,'slice-clearance-not-proved');assert.deepEqual(hints(f),before);
});

test('nonorthonormal bases use conservative world-ball projection bounds',()=>{
    const bases=[2,0,0,0,3,0,1,0,1],f=squareField({halfWidths:[.45,.45],bases});
    const proof=f.certifyInsideBallCoordinates(0,0,0,.1);assert.equal(proof.supported,true);
    assert.ok(Math.abs(proof.localXZRadius-.1*Math.sqrt(6))<1e-15);assert.ok(Math.abs(proof.localYRadius-.3)<1e-15);
    assert.equal(f.certifyInsideBallCoordinates(0,0,0,.2).supported,false,'using radius without basis norm would wrongly certify this bound');
    assert.equal(f.certifyInsideBallCoordinates(0,.6,0,.1).supported,false,'Y basis scaling must also bound slab containment');
});

test('invalid balls reject and zero radius still requires positive slice clearance',()=>{
    const f=squareField();for(const args of [[NaN,0,0,1],[0,0,0,-1],[0,0,0,Infinity]])assert.throws(()=>f.certifyInsideBallCoordinates(...args),RangeError);
    assert.equal(f.certifyInsideBallCoordinates(0,0,0,0).supported,true);assert.equal(f.certifyInsideBallCoordinates(2,0,0,0).supported,false);
});
