import assert from 'node:assert/strict';
import test from 'node:test';
import { createCoupledSolverSelection, resolveAppCoupledSolver } from '../src/physics/coupledSolverSelection.js';
import { EndovascularPhysicsWorld } from '../src/physics/endovascularPhysicsWorld.js';
import { solveKirchhoffCoupledSystem, applyKirchhoffCoupledCorrection } from '../src/physics/kirchhoffCoupledSystem.js';
import { createCoupledRuntimeFixture } from './helpers/coupledRuntimeFixture.js';

const kernel = { solve: solveKirchhoffCoupledSystem, apply: applyKirchhoffCoupledCorrection };

test('application restores position-history and a shared contact block for old experimental bookmarks', () => {
    for (const search of ['', '?coupledSolver=composite-joint', '?coupledSolver=joint-two-channel',
        '?coupledSolver=joint-two-channel&coupledLinearSolver=axial-band']) {
        const selected = createCoupledSolverSelection(resolveAppCoupledSolver(search), kernel);
        assert.equal(selected.id, 'joint-active-coulomb');
        assert.equal(selected.jointMotionMode, 'position-history');
        assert.equal(selected.biasMaterialMode, null);
        assert.equal(typeof selected.coupledSystem.solve, 'function');
        assert.equal(selected.coupledSystem.solveTwoChannel, undefined);
    }
    assert.equal(resolveAppCoupledSolver('?coupledSolver=joint-two-channel&experimentalSplitMotion=1'), 'joint-two-channel');
    assert.equal(resolveAppCoupledSolver('?coupledSolver=reference'), 'reference');
    assert.equal(resolveAppCoupledSolver('?coupledSolver=joint-full-band'), 'joint-full-band');
});

test('composite selection requires and reports the installed complete-step provider without selecting an older coupled kernel',()=>{
    assert.throws(()=>createCoupledSolverSelection('composite-joint',kernel),/whole-step/);
    const system={id:'composite-joint',step(){},reset(){},diagnostics:{accepted:2}},selection=createCoupledSolverSelection('composite-joint',{...kernel,wholeStepSystem:system});
    assert.equal(selection.coupledSystem,null);assert.equal(selection.wholeStepSystem,system);
    assert.equal(selection.getReport({coupledSystem:null}).installed,false);
    assert.equal(selection.getReport({wholeStepSystem:system}).installed,true);
    assert.deepEqual(selection.getReport({wholeStepSystem:system}).wholeStep,{accepted:2});
});

test('reference remains the default and an unknown or incomplete opt-in fails visibly', () => {
    const selected = createCoupledSolverSelection();
    assert.equal(selected.coupledSystem, null);
    assert.equal(selected.getReport({ coupledSystem: null }).installed, true);
    assert.equal(selected.getReport().solveCalls, 0);
    assert.equal(selected.getReport().options, null);
    for (const id of ['', 'active', 'JOINT', null]) assert.throws(() => createCoupledSolverSelection(id, kernel), RangeError);
    assert.throws(() => createCoupledSolverSelection('joint-active-coulomb'), TypeError);
});

for (const id of ['joint', 'joint-active-coulomb', 'joint-full-band']) test(`${id} fixes only its variant options and forwards the complete world solve`, () => {
    const enabled = id !== 'joint', constraint = {};
    const structure = id === 'joint-full-band' ? { coulombStructure: 'full-band' } : {};
    const rows = [], groups = [], workspace = {};
    const runtimeOptions = Object.freeze({ additionalRows: rows, groups, workspace,
        tolerance: 0.0002, numericalShift: 1e-8, basis: 'individual',
        activeCondensation: !enabled, simultaneousCoulomb: !enabled });
    const result = { diagnostics: { converged: true, status: 'converged' } };
    const apply = () => {};
    const selected = createCoupledSolverSelection(id, { apply, solve(c, dt, options) {
        assert.equal(c, constraint); assert.equal(dt, 1 / 120);
        assert.deepEqual(options, { ...runtimeOptions, activeCondensation: enabled, simultaneousCoulomb: enabled, ...structure });
        assert.equal(options.additionalRows, rows); assert.equal(options.groups, groups); assert.equal(options.workspace, workspace);
        return result;
    } });
    assert.equal(selected.coupledSystem.solve(constraint, 1 / 120, runtimeOptions), result);
    assert.equal(selected.coupledSystem.apply, apply);
    assert.equal(runtimeOptions.activeCondensation, !enabled);
    assert.equal(selected.getReport().solveCalls, 1);
});

test('full-band reporting distinguishes its fixed-load seed from an executed LU Newton step', () => {
    let diagnostics = { converged: true, status: 'converged', originalCount: 12, coulombStructure: 'full-band' };
    const selected = createCoupledSolverSelection('joint-full-band', {
        apply() {}, solve() { return { diagnostics }; }
    });
    selected.coupledSystem.solve({}, 1 / 120);
    assert.equal(selected.getReport().fullBandResults, 1);
    assert.equal(selected.getReport().bandLUNewtonResults, 0);
    assert.equal(selected.getReport().activeCondensedResults, 0);
    diagnostics = { ...diagnostics, method: 'simultaneous-coulomb-newton', linearSolver: 'band-lu' };
    selected.coupledSystem.solve({}, 1 / 120);
    assert.equal(selected.getReport().bandLUNewtonResults, 1);
    assert.equal(selected.getReport().lastResult.linearSolver, 'band-lu');
    selected.resetDiagnostics();
    assert.equal(selected.getReport().fullBandResults, 0);
    assert.equal(selected.getReport().bandLUNewtonResults, 0);
});

test('reported results distinguish condensation, Newton and a seed without inventing execution', () => {
    let diagnostics = { converged: true, status: 'converged', originalCount: 12, equalityCount: 9, retainedCount: 3 };
    const selected = createCoupledSolverSelection('joint-active-coulomb', {
        apply() {}, solve() { return { diagnostics }; }
    });
    const world = { coupledSystem: selected.coupledSystem, lastCoupledSolver: 'independent' };
    assert.equal(selected.getReport(world).activeCondensedResults, 0);
    selected.coupledSystem.solve({}, 1 / 120);
    assert.equal(selected.getReport(world).simultaneousCoulombNewtonResults, 0, 'seed success is not a Newton result');
    diagnostics = { ...diagnostics, method: 'simultaneous-coulomb-newton', converged: false, status: 'iteration-limit' };
    selected.coupledSystem.solve({}, 1 / 120);
    const report = selected.getReport(world);
    assert.equal(report.activeCondensedResults, 2);
    assert.equal(report.simultaneousCoulombNewtonResults, 1);
    assert.equal(report.nonconvergedResults, 1);
    assert.equal(report.lastStepSolver, 'independent');
    world.coupledSystem = null;
    assert.equal(selected.getReport(world).installed, false);
    selected.resetDiagnostics();
    assert.equal(selected.getReport().solveCalls, 0);
    assert.equal(selected.getReport().lastResult, null);
    assert.equal(report.solveCalls, 2, 'completed reports remain snapshots');
});

test('runtime fixture installs the selected kernel at construction and keeps it across reset', () => {
    const selected = createCoupledSolverSelection('joint-active-coulomb', kernel);
    const fixture = createCoupledRuntimeFixture({ coupledSystem: selected.coupledSystem });
    try {
        assert.equal(fixture.world.coupledSystem, selected.coupledSystem);
        assert.equal(fixture.world.fixedDt, 1 / 120);
        assert.equal(fixture.world.maxSubsteps, 2);
        assert.equal(Object.hasOwn(fixture.config, 'coupledSystem'), false, 'config stays serializable');
        fixture.reset();
        assert.equal(fixture.world.coupledSystem, selected.coupledSystem);
    } finally { fixture.dispose(); }
});

for (const id of ['joint-active-coulomb', 'joint-full-band']) test(`a loaded two-node ${id} step matches its exact direct wrapper`, () => {
    function pair(coupledSystem) {
        const world = new EndovascularPhysicsWorld({ coupledSystem, fixedDt: 1 / 120 });
        const profile = { radius: 0.889 / 2, linearDamping: 1, angularDamping: 1,
            foldLimitStrength: 0, projectionVelocityRetention: 1, sleepFrames: 1e6 };
        const inner = world.createRod('wire', 2, 10, { ...profile, mass: 1 });
        const outer = world.createRod('catheter', 2, 10, { ...profile, mass: 3 });
        for (let i = 0; i < 2; i++) {
            inner.setNodePosition(i, i * 10, 0.0405, 0);
            inner.velocityX[i] = 6; inner.velocityY[i] = 2;
        }
        inner.angularVelocityX.fill(2);
        world.addContainment(inner, outer, { innerRadius: 0.97 / 2,
            axialFriction: 0.2, torsionalFriction: 0.2, portalFilletRadius: 0,
            coupledBendingRateDamping: 0, radialVelocityDamping: 0 });
        world.stepFixed();
        return world;
    }
    const selected = createCoupledSolverSelection(id, kernel);
    const actual = pair(selected.coupledSystem);
    const expected = pair({ apply: kernel.apply, solve(c, dt, options) {
        return kernel.solve(c, dt, { ...options, ...selected.getReport().options });
    } });
    assert.equal(actual.lastCoupledSolver, 'joint');
    assert.equal(actual.lastCoupledClosureConverged, true);
    assert.ok(selected.getReport(actual)[id === 'joint-full-band' ? 'fullBandResults' : 'activeCondensedResults'] > 0);
    for (let side = 0; side < 2; side++) for (const field of ['x', 'y', 'z',
        'velocityX', 'velocityY', 'velocityZ', 'orientationX', 'orientationY', 'orientationZ', 'orientationW',
        'angularVelocityX', 'angularVelocityY', 'angularVelocityZ',
        'bendTwistLambda1', 'bendTwistLambda2', 'bendTwistLambda3',
        'adaptationLambdaX', 'adaptationLambdaY', 'adaptationLambdaZ']) {
        assert.ok(actual.bodies[side][field], field);
        assert.deepEqual(actual.bodies[side][field], expected.bodies[side][field], field);
    }
});

test('explicit axial variant selects the axial adapter and retains the position-history step',()=>{
    assert.equal(resolveAppCoupledSolver('?coupledSolver=joint-axial-sections'),'joint-axial-sections');
    assert.throws(()=>createCoupledSolverSelection('joint-axial-sections',kernel),/kernel/);
    let calls=0;
    const selected=createCoupledSolverSelection('joint-axial-sections',{...kernel,solveAxial(c,dt,options){
        calls++;assert.equal(dt,1/120);assert.equal(options.sectionSpan,32);assert.equal(options.tolerance,1e-8);
        return {diagnostics:{converged:true,status:'converged',axialReduction:true,globalRowCount:10,localRowCount:5}};
    }});
    selected.coupledSystem.solve({},1/120,{tolerance:1e-8});assert.equal(calls,1);
    assert.equal(selected.jointMotionMode,'position-history');
    assert.equal(selected.getReport().lastResult.localRowCount,5);
    assert.equal(selected.getReport().lastResult.axialReduction,true);
});

test('finite wall witness runtime is explicit and retains the common position-history kernel',()=>{
    const selected=createCoupledSolverSelection(resolveAppCoupledSolver('?coupledSolver=joint-wall-witnesses'),kernel);
    assert.equal(selected.coupledSystem.wallWitnesses,true);
    assert.equal(selected.coupledSystem.independentComponents,true);
    assert.equal(selected.jointMotionMode,'position-history');
    assert.equal(createCoupledSolverSelection(resolveAppCoupledSolver(''),kernel).coupledSystem.wallWitnesses,undefined);
});
