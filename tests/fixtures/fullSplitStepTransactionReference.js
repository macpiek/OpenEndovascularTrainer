// Frozen full-graph reference from Transaction 0990e3ba, before scratch exclusion.
// Independent rollback oracle for the optimized whole-dt transaction.
import { captureKirchhoffCoupledTrialState, restoreKirchhoffCoupledTrialState } from '../../src/physics/kirchhoffCoupledTrialState.js';

const collectionKeys = ['bodies', 'containments', 'toolContacts', 'sheaths'];
const settingKeys = ['fixedDt', 'jointMotionMode', 'contactField', 'coupledSystem'];
const topologyKeys = ['count', 'segmentCount', 'activeStart', 'activeEnd'];
const topologyArrays = ['x', 'y', 'z', 'previousX', 'previousY', 'previousZ',
    'orientationX', 'orientationY', 'orientationZ', 'orientationW'];

function slots(object) {
    return { object, descriptors: Object.getOwnPropertyDescriptors(object) };
}
function restoreSlots({ object, descriptors }) {
    for (const key of Reflect.ownKeys(object)) if (!Object.hasOwn(descriptors, key)) delete object[key];
    Object.defineProperties(object, descriptors);
}

/** Whole physical-dt transaction, captured AFTER the caller prepared inputs
 * but BEFORE wake, damping, integration, multiplier reset or contact refresh.
 *
 * The existing trial snapshot supplies owned numeric bytes and the mutable
 * constraint/manifold/tool graph. It does NOT supply this whole-step contract:
 * we additionally own body reference slots (phase banks replace arrays),
 * collection/sheath references, body topology, simulation time and integrator
 * flags. Restore those first so the trial helper's topology checks apply to
 * the original bodies, including when a failed callback replaced storage.
 * Every containment is captured, including disabled ones touched by beginStep.
 *
 * Solver scratch and old local-trial snapshots are disposable caches. They
 * are invalidated, never published as restored physical history. World timing
 * and work counters remain outside this snapshot, so rejection retains cost.
 * External callbacks/renderer/contact-field internals are not physical state.
 */
export function captureKirchhoffSplitStep(world) {
    const collections = collectionKeys.map(key => ({ key,
        descriptor: Object.getOwnPropertyDescriptor(world, key), array: world[key], values: world[key].slice() }));
    const bodySlots = world.bodies.map(slots), sheathSlots = world.sheaths.map(slots);
    const sheathBodies = world.sheaths.filter(sheath => Array.isArray(sheath.bodies))
        .map(sheath => ({ array: sheath.bodies, values: sheath.bodies.slice() }));
    const topology = world.bodies.map(body => ({ body,
        values: Object.fromEntries([...topologyKeys, ...topologyArrays].map(key => [key, body[key]])) }));
    const localTrialCaches = world.containments.map(constraint => constraint._jointTrialState).filter(Boolean);
    const trials = world.containments.map(constraint => captureKirchhoffCoupledTrialState(constraint,
        { world, external: localTrialCaches }));
    return { collections, bodySlots, sheathSlots, sheathBodies, topology, trials,
        settings: settingKeys.map(key => ({ key, descriptor: Object.getOwnPropertyDescriptor(world, key) })),
        dt: world.fixedDt,
        stepCount: world.stepCount, inCoupledClosure: Object.getOwnPropertyDescriptor(world, '_inCoupledClosure'),
        restored: false };
}

/** No topology/material window may change inside the solver transaction.
 * beforeSubstep may prepare a new window before capture. An internal change
 * is rejected and restored rather than committing a solve on mixed topology.
 */
export function kirchhoffSplitStepTopologyUnchanged(world, snapshot) {
    return snapshot.settings.every(({ key, descriptor }) => world[key] === descriptor.value) &&
        snapshot.collections.every(({ key, array, values }) => world[key] === array &&
        array.length === values.length && values.every((value, index) => array[index] === value)) &&
        snapshot.topology.every(({ body, values }) => Object.entries(values).every(([key, value]) => body[key] === value));
}

export function restoreKirchhoffSplitStep(world, snapshot) {
    if (snapshot.restored) throw new Error('A physical step transaction can be restored only once');
    // Reconnect original objects/storage before restoring any numeric bytes.
    for (const { key, descriptor, array, values } of snapshot.collections) {
        Object.defineProperty(world, key, descriptor);
        array.length = 0; array.push(...values);
    }
    for (const state of [...snapshot.bodySlots, ...snapshot.sheathSlots]) restoreSlots(state);
    for (const { array, values } of snapshot.sheathBodies) { array.length = 0; array.push(...values); }
    for (const trial of snapshot.trials) restoreKirchhoffCoupledTrialState(trial);
    // The numeric-body filter of the trial helper intentionally omits object
    // references; a failed split must also undo _splitPhysicalMotion and bank
    // substitutions, including removing properties first created by that dt.
    for (const state of [...snapshot.bodySlots, ...snapshot.sheathSlots]) restoreSlots(state);
    for (const constraint of world.containments) delete constraint._jointTrialState;
    for (const { key, descriptor } of snapshot.settings) Object.defineProperty(world, key, descriptor);
    world.stepCount = snapshot.stepCount;
    if (snapshot.inCoupledClosure) Object.defineProperty(world, '_inCoupledClosure', snapshot.inCoupledClosure);
    else delete world._inCoupledClosure;
    snapshot.restored = true;
}
