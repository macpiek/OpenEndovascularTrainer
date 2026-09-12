import test from 'node:test';
import assert from 'node:assert/strict';
import {EndovascularPhysicsWorld} from '../src/physics/endovascularPhysicsWorld.js';
import {applyKirchhoffMaterialProfile} from '../src/physics/applyKirchhoffMaterialProfile.js';
import {assembleKirchhoffCoupledSystem,solveKirchhoffCoupledSystem,applyKirchhoffCoupledCorrection} from '../src/physics/kirchhoffCoupledSystem.js';
import {buildKirchhoffAxialSystemLayout} from '../src/physics/kirchhoffAxialSystemLayout.js';
import {assembleKirchhoffAxialReducedSystem} from '../src/physics/kirchhoffAxialReducedSystem.js';
import {solveKirchhoffAxialCoupledSystem} from '../src/physics/kirchhoffAxialCoupledSolver.js';

const dt=1/120;
function fixture() {
    const world=new EndovascularPhysicsWorld(),body=world.createRod('catheter',9,5,{mass:.07});
    applyKirchhoffMaterialProfile(body,'berenstein'); body.restRotation1.fill(0);body.restRotation2.fill(0);
    body.setPinned(0,true);
    body.setProximalOrientationControl(body.orientationX[0],body.orientationY[0],body.orientationZ[0],body.orientationW[0],0,0);
    body.y[3]+=.001;
    return {body,component:{bodies:[body],kirchhoffContacts:[]}};
}
function closeArray(actual,expected,tolerance=1e-7) {
    assert.equal(actual.length,expected.length);
    expected.forEach((value,i)=>assert.ok(Math.abs(value-actual[i])<=tolerance*Math.max(1,Math.abs(value)),`${i}: ${actual[i]} vs ${value}`));
}

for(const wall of [false,true]) for(const sections of [false,true]) test(`single axial preserves native material/contact correction, wall=${wall}, sections=${sections}`,()=>{
    const {body,component}=fixture();
    const options={tolerance:1e-9,includeSystem:true,activeCondensation:true,simultaneousCoulomb:true,
        additionalRows:wall?[{kind:'wall',strain:-.001,alpha:0,lambda:0,lower:0,upper:Infinity,
            gradients:[{side:0,dof:4*6+1,value:1}]}]:[]};
    const full=solveKirchhoffCoupledSystem(component,dt,options);
    assert.ok(full.diagnostics.converged,JSON.stringify(full.diagnostics));
    const native=assembleKirchhoffCoupledSystem(component,dt,{...options,jacobianOnly:true,includeAxialLayout:true});
    assert.equal(native.bodies.length,1);assert.equal(native.axialLayout.kind,'single');
    assert.equal(native.axialLayout.wireNodes,undefined);assert.equal(native.axialLayout.sites,undefined);
    const reduced=assembleKirchhoffAxialReducedSystem(native,{matrixFormat:'general-band',sectionSpan:sections?20:undefined,sectionScope:'all'});
    assert.equal(reduced.diagnostics.eliminatedOffsetDofs,0);
    assert.equal(reduced.diagnostics.originalDofs,reduced.diagnostics.retainedDofs);
    assert.equal(reduced.diagnostics.softMaterialRows+reduced.diagnostics.explicitRows,native.count);
    if(sections) assert.ok(reduced.localSections.some(section=>section.length>0));
    const before=[...body.y];
    const solved=solveKirchhoffAxialCoupledSystem(component,dt,{...options,sectionSpan:sections?20:undefined,sectionScope:'all'});
    assert.ok(solved.diagnostics.converged,JSON.stringify(solved.diagnostics));
    assert.equal(solved.diagnostics.axialReduction,true);
    assert.equal(solved.responses.length,1);assert.equal(solved.outer,undefined);
    closeArray(solved.responses[0].correction,full.responses[0].correction);
    closeArray(solved.responses[0].lambda,full.responses[0].lambda);
    closeArray(solved.additionalIncrement,full.additionalIncrement);
    assert.ok(solved.diagnostics.reconstructionResidual<=options.tolerance);
    assert.ok(solved.diagnostics.originalMobilityError<1e-8);
    assert.deepEqual([...body.y],before);
    applyKirchhoffCoupledCorrection(component,solved);
    assert.ok(body.y.every(Number.isFinite));
});

test('single layout owns original node identity and coordinates without a second-tool source',()=>{
    const {body,component}=fixture();body.activeStart=2;
    const source=assembleKirchhoffCoupledSystem(component,dt,{jacobianOnly:true});
    const arcs=[Float64Array.from({length:body.count},(_,i)=>i*5)];
    const layout=buildKirchhoffAxialSystemLayout(component,source.material,arcs,{axialOffsets:[3]});
    assert.deepEqual([...layout.nodes],[2,3,4,5,6,7,8]);
    assert.equal(layout.coordinates[0],13);
    const position=layout.positions[0],coordinate=layout.coordinates[0],id=layout.materialIds[0];
    body.x[2]+=1;arcs[0][2]+=1;body.materialCoordinate[2]+=1;
    assert.equal(layout.positions[0],position);assert.equal(layout.coordinates[0],coordinate);assert.equal(layout.materialIds[0],id);
    assert.throws(()=>buildKirchhoffAxialSystemLayout(component,source.material,arcs,{axialOffsets:[0,0]}),/one finite/);
    arcs[0][3]=NaN;
    assert.throws(()=>buildKirchhoffAxialSystemLayout(component,source.material,arcs),/finite/);
});
