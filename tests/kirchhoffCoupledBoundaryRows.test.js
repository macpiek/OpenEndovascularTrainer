import assert from 'node:assert/strict';
import test from 'node:test';
import { EndovascularPhysicsWorld } from '../src/physics/endovascularPhysicsWorld.js';
import {
    beginKirchhoffCoupledBoundaryStep, collectKirchhoffCoupledBoundaryRows,
    applyKirchhoffCoupledBoundaryMultipliers, measureKirchhoffCoupledBoundaryResidual
} from '../src/physics/kirchhoffCoupledBoundaryRows.js';

function fixture() {
    const world = new EndovascularPhysicsWorld();
    const inner = world.createRod('inner', 3, 5, { mass: 2, radius: 0.4 });
    const outer = world.createRod('outer', 3, 5, { mass: 3, radius: 0.8 });
    const constraint = world.addContainment(inner, outer, { enabled: true });
    for (const body of [inner, outer]) {
        body.controlEnabled.fill(0); body.wallActive.fill(0);
        body.collisionStartSegment = 0; body.collisionEndSegment = 1;
    }
    beginKirchhoffCoupledBoundaryStep(constraint);
    return { world, inner, outer, constraint };
}

test('Cartesian control rows preserve isotropic control energy and commit without moving the body', () => {
    const { inner, constraint } = fixture();
    inner.controlEnabled[1] = 1; inner.controlCompliance[1] = 0.01;
    inner.controlX[1] = 4; inner.controlY[1] = 2; inner.controlZ[1] = -3;
    const rows = collectKirchhoffCoupledBoundaryRows(constraint, [], 0.1);
    assert.equal(rows.length, 3);
    assert.deepEqual(rows.map(r => r.strain), [1, -2, 3]);
    assert.ok(rows.every(r => Math.abs(r.alpha - 1) < 1e-7 && r.lower === -Infinity));
    const before = [inner.x[1], inner.y[1], inner.z[1]];
    applyKirchhoffCoupledBoundaryMultipliers(constraint, [0.2, -0.4, 0.6], 0.5);
    assert.deepEqual([inner.x[1], inner.y[1], inner.z[1]], before);
    const next = collectKirchhoffCoupledBoundaryRows(constraint, [], 0.1);
    assert.deepEqual(next.map(r => r.lambda), [0.1, -0.2, 0.3]);
    beginKirchhoffCoupledBoundaryStep(constraint);
    assert.ok(collectKirchhoffCoupledBoundaryRows(constraint, [], 0.1).every(r => r.lambda === 0));
});

test('wall row uses the existing sample normal, interpolation, radius and compliance', () => {
    const { outer, constraint } = fixture();
    outer.wallActive[0] = 1; outer.wallT[0] = 0.25;
    outer.wallNormalX[0] = 0; outer.wallNormalY[0] = 1; outer.wallNormalZ[0] = 0;
    outer.wallY[0] = -0.7; outer.wallCompliance = 1e-5;
    outer.y[0] = -0.1; outer.y[1] = 0.1;
    const rows = collectKirchhoffCoupledBoundaryRows(constraint, [], 0.01);
    assert.equal(rows.length, 1);
    assert.ok(Math.abs(rows[0].strain + 0.15) < 1e-6);
    assert.ok(Math.abs(rows[0].alpha - 0.1) < 1e-10);
    assert.deepEqual(rows[0].gradients.map(g => [g.side, g.dof, g.value]), [[1, 1, 0.75], [1, 7, 0.25]]);
    const before = [...outer.y];
    rows[0].activeHint = true;
    applyKirchhoffCoupledBoundaryMultipliers(constraint, [0.3], 0.5);
    assert.deepEqual([...outer.y], before);
    assert.ok(Math.abs(outer.wallLambda[0] - 0.15) < 1e-7);
    assert.ok(Math.abs(outer.wallProjectionY[0] - 0.15 * 0.75 / 3) < 1e-7);
    assert.equal(collectKirchhoffCoupledBoundaryRows(constraint, [], 0.01)[0].activeHint, true);
});

test('open introducer constrains radial position only inside the supported material and axial slab', () => {
    const { world, inner, outer, constraint } = fixture();
    inner.sheathMaterialEndNode = 1; outer.sheathMaterialEndNode = -1;
    for (let i = 0; i < 3; i++) inner.y[i] = 0.8;
    const sheath = world.addSheath({ start: { x: 0, y: 0, z: 0 }, end: { x: 5, y: 0, z: 0 }, innerRadius: 1, bodies: [inner] });
    const rows = collectKirchhoffCoupledBoundaryRows(constraint, [sheath], 1 / 120);
    assert.equal(rows.length, 2);
    assert.ok(rows.every(row => row.kind === 'sheath' && Math.abs(row.strain + 0.2) < 1e-7));
    assert.ok(rows.every(row => row.gradients.every(g => g.dof % 6 === 1)));
    inner.x[1] = 5.1;
    assert.equal(collectKirchhoffCoupledBoundaryRows(constraint, [sheath], 1 / 120).length, 1);
});

test('boundary residual is complementary and does not consume other blocks or their units', () => {
    assert.equal(measureKirchhoffCoupledBoundaryResidual([
        { kind: 'wall', strain: 1, alpha: 0, lambda: 0, lower: 0 },
        { kind: 'wall', strain: -0.2, alpha: 1, lambda: 0.2, lower: 0 },
        { kind: 'fold', strain: -50, alpha: 0, lambda: 0, lower: 0 }
    ]), 0);
    assert.equal(measureKirchhoffCoupledBoundaryResidual([
        { kind: 'control', strain: -0.2, alpha: 0, lambda: 0, lower: -Infinity }
    ]), 0.2);
});

test('single real body preserves control, wall and sheath rows and multiplier ownership', () => {
    const {world, inner, constraint: pair} = fixture();
    inner.controlEnabled[1] = 1; inner.controlCompliance[1] = 0.01; inner.controlY[1] = 2;
    inner.wallActive[0] = 1; inner.wallT[0] = 0.25; inner.wallNormalY[0] = 1; inner.wallY[0] = -0.2;
    inner.sheathMaterialEndNode = 1; inner.y.fill(0.8);
    const sheath = world.addSheath({start:{x:0,y:0,z:0},end:{x:5,y:0,z:0},innerRadius:1,bodies:[inner]});
    const solo = {bodies:[inner]};
    beginKirchhoffCoupledBoundaryStep(solo);
    const snapshot = rows => rows.map(({kind,side,node,alpha,strain,lower,upper,gradients}) =>
        ({kind,side,node,alpha,strain,lower,upper,gradients:structuredClone(gradients)}));
    const expected = snapshot(collectKirchhoffCoupledBoundaryRows(pair,[sheath],0.1));
    const actual = collectKirchhoffCoupledBoundaryRows(solo,[sheath],0.1);
    assert.deepEqual(snapshot(actual),expected);
    assert.ok(actual.every(row => row.side===0));
    const before = [...inner.y];
    applyKirchhoffCoupledBoundaryMultipliers(solo,actual.map(()=>0.2),0.5);
    assert.deepEqual([...inner.y],before);
    assert.ok(collectKirchhoffCoupledBoundaryRows(solo,[sheath],0.1).every(row=>Math.abs(row.lambda-0.1)<1e-7));
    assert.throws(()=>beginKirchhoffCoupledBoundaryStep({bodies:[inner],outerBody:inner}),/aliases/);
});


test('boundary component membership changes require a new step and fresh multiplier ownership', () => {
    const {inner,outer}=fixture(), component={bodies:[inner]};
    inner.controlEnabled[1]=1;
    beginKirchhoffCoupledBoundaryStep(component);
    const rows=collectKirchhoffCoupledBoundaryRows(component,[],0.1);
    applyKirchhoffCoupledBoundaryMultipliers(component,rows.map(()=>0.2),1);
    component.bodies=[outer]; outer.controlEnabled[1]=1;
    assert.throws(()=>collectKirchhoffCoupledBoundaryRows(component,[],0.1),/topology/);
    beginKirchhoffCoupledBoundaryStep(component);
    assert.ok(collectKirchhoffCoupledBoundaryRows(component,[],0.1).every(row=>row.owner===outer && row.lambda===0));
});

test('boundary natural map is continuous at zero force and has the original complementarity zeros', async () => {
    const {measureKirchhoffCoupledBoundaryMerit: merit,kirchhoffRowNaturalMapMobility: mobility} = await import('../src/physics/kirchhoffCoupledBoundaryRows.js');
    const {inner}=fixture(), c={bodies:[inner]}, row={kind:'wall',strain:.1,alpha:0,lambda:0,lower:0,
        gradients:[{side:0,dof:7,value:1}]};
    assert.equal(mobility([inner],row),inner.inverseMass[1]);
    assert.equal(merit(c,[row]),0);
    for(const lambda of [1e-9,1e-12,1e-18]) {
        row.lambda=lambda;
        assert.equal(merit(c,[row]),inner.inverseMass[1]*lambda);
    }
    row.lambda=1; assert.equal(merit(c,[row]),.1);
    row.strain=0; assert.equal(merit(c,[row]),0);
    row.strain=-.2; assert.equal(merit(c,[row]),.2);
    row.lambda=0; assert.equal(merit(c,[row]),.2);
    row.strain=.1; row.lambda=-.2; assert.ok(merit(c,[row])>0);
    row.kind='control';row.lower=-Infinity;row.lambda=0;
    assert.equal(merit(c,[row]),.1,'equality residual is never suppressed by a zero force');
    row.kind='wall';row.lower=0;row.strain=-.2;inner.inverseMass[1]=0;
    assert.equal(mobility([inner],row),1);assert.equal(merit(c,[row]),.2,'immovable violation remains visible');
    row.alpha=.3; assert.equal(mobility([inner],row),.3);
    assert.throws(()=>merit(c,[row],[0]),/positive/);
});

test('natural-map diagonal sums repeated Jacobian entries before squaring and excludes prescribed rotation', async () => {
    const {kirchhoffRowNaturalMapMobility: mobility}=await import('../src/physics/kirchhoffCoupledBoundaryRows.js');
    const {inner}=fixture();
    const row={alpha:.1,gradients:[{side:0,dof:7,value:1},{side:0,dof:7,value:2}]};
    assert.equal(mobility([inner],row),.1+9*inner.inverseMass[1]);
    inner.orientationControlCompliance=0;inner.orientationControlSegment=1;
    row.gradients=[{side:0,dof:9,value:2}];
    assert.equal(mobility([inner],row),.1);
});

test('boundary Jacobians match directional derivatives of controls, frozen wall witness and nonlinear sheath gap', () => {
    const {world,inner,constraint}=fixture();
    // Differentiate the equations, excluding Float32 storage quantization.
    for(const axis of ['x','y','z']) inner[axis]=Float64Array.from(inner[axis]);
    inner.controlEnabled[1]=1;inner.controlCompliance[1]=.02;inner.controlY[1]=.3;
    inner.wallActive[0]=1;inner.wallT[0]=.37;
    const normal=[.2,.7,-.3],norm=Math.hypot(...normal);
    [inner.wallNormalX[0],inner.wallNormalY[0],inner.wallNormalZ[0]]=normal.map(v=>v/norm);
    inner.sheathMaterialEndNode=1;inner.y.fill(.8);inner.z.fill(.2);
    const sheath=world.addSheath({start:{x:0,y:0,z:0},end:{x:20,y:0,z:0},innerRadius:1,bodies:[inner]});
    const rows=collectKirchhoffCoupledBoundaryRows(constraint,[sheath],.1);
    const direction=Float64Array.from({length:inner.count*6},(_,i)=>Math.sin(i+.2));
    const expected=rows.map(row=>row.gradients.reduce((sum,g)=>sum+(g.side===0?g.value*direction[g.dof]:0),0));
    assert.ok(rows.some(row=>row.kind==='wall'));assert.ok(rows.some(row=>row.kind==='sheath'));
    const snapshot=[inner.x.slice(),inner.y.slice(),inner.z.slice()];
    const at=h=>{
        [inner.x,inner.y,inner.z].forEach((array,axis)=>array.forEach((_,node)=>{array[node]=snapshot[axis][node]+h*direction[node*6+axis];}));
        return collectKirchhoffCoupledBoundaryRows(constraint,[sheath],.1).map(row=>row.strain);
    };
    const h=1e-6,plus=at(h),minus=at(-h);at(0);
    expected.forEach((v,i)=>assert.ok(Math.abs(v-(plus[i]-minus[i])/(2*h))<2e-9, `${i}: Jd=${v}, finiteDifference=${(plus[i]-minus[i])/(2*h)}`));
});

test('boundary diagnostics own worst wall scalars and clear when reused for equality or empty rows', () => {
    const {inner}=fixture(), out={};
    inner.wallT[0]=.25;inner.wallBranchId[0]=2;inner.wallFaceIndex[0]=7;
    inner.wallNormalX[0]=.2;inner.wallNormalY[0]=.3;inner.wallNormalZ[0]=.4;inner.wallGap[0]=-.5;
    const wall={kind:'wall',side:0,node:0,strain:-.5,alpha:.1,lambda:1,lower:0,owner:inner};
    const equality={kind:'control',side:0,node:1,strain:.1,alpha:0,lambda:0,lower:-Infinity};
    assert.equal(measureKirchhoffCoupledBoundaryResidual([wall,equality],out),.4);
    assert.equal(out.kind,'wall');assert.equal(out.residual,-.4);assert.equal(out.maximumResidual,.4);
    assert.equal(out.wallT,.25);assert.equal(out.wallBranchId,2);assert.equal(out.wallFaceIndex,7);
    assert.equal(out.wallGap,-.5);assert.equal(out.strain,-.5);assert.equal(out.alpha,.1);assert.equal(out.lambda,1);
    assert.ok(Object.values(out).every(value=>value===null||typeof value!=='object'));
    inner.wallT[0]=.75;wall.strain=-2;
    assert.equal(out.wallT,.25);assert.equal(out.strain,-.5,'diagnostics do not borrow the row');
    equality.strain=3;
    assert.equal(measureKirchhoffCoupledBoundaryResidual([wall,equality],out),3);
    assert.equal(out.kind,'control');assert.equal(out.node,1);assert.equal(out.residual,3);
    assert.equal(out.wallT,null);assert.equal(out.wallBranchId,null);assert.equal(out.wallNormalX,null);
    assert.equal(measureKirchhoffCoupledBoundaryResidual([],out),0);
    assert.equal(out.maximumResidual,0);
    assert.ok(Object.entries(out).filter(([key])=>key!=='maximumResidual').every(([,value])=>value===null));
    assert.equal(measureKirchhoffCoupledBoundaryResidual([wall]),1.9,'numeric-only API remains unchanged');
});
