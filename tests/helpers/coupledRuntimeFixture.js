import * as THREE from 'three';
import { DEFAULT_TOOL_PROFILES, EndovascularPhysicsWorld } from '../../src/physics/endovascularPhysicsWorld.js';
import { RodState } from '../../src/physics/rodState.js';
import { GuidewireTransport } from '../../src/physics/guidewireTransport.js';
import { applyGuidewireMaterialProfile } from '../../src/physics/guidewireMaterialProfile.js';
import { applyKirchhoffMaterialProfile } from '../../src/physics/applyKirchhoffMaterialProfile.js';
import { applyProximalTwistBoundary } from '../../src/physics/kirchhoffOrientationBoundary.js';
import { guidewireRelaxationPasses } from '../../src/physics/guidewireRelaxationRate.js';
import { spatiallyCapturedContainmentEnd, firstFreeGuidewireNodeAfterContainment } from '../../src/physics/catheterGuidewireCoupling.js';
import { PigtailCatheter, CATHETER_PROXIMAL_LOADING_SUPPORT_LENGTH_MM } from '../../src/pigtailCatheter.js';
import { GUIDEWIRE_RADIUS_MM, PIGTAIL_CATHETER_INNER_RADIUS_MM, INTRODUCER_SHEATH_INNER_RADIUS_MM } from '../../src/toolDimensions.js';
import { generateVessel } from '../../src/vesselGeometry.js';


// Byte-for-byte algorithm and fields used by ShortCatheterBenchmarkMetrics.
export function poseFingerprint(body) {
    let hash = 2166136261;
    for (const values of [body.x, body.y, body.z, body.velocityX, body.velocityY, body.velocityZ,
        body.orientationX, body.orientationY, body.orientationZ, body.orientationW]) {
        for (const byte of new Uint8Array(values.buffer, values.byteOffset, values.byteLength)) {
            hash = Math.imul(hash ^ byte, 16777619);
        }
    }
    return (hash >>> 0).toString(16).padStart(8, '0');
}

export const COUPLED_RUNTIME_DEFAULTS = Object.freeze({
    jointMotionMode: 'position-history',
    fixedDt: 1 / 120, guidewireLength: 1000, guidewireTargetMm: 999.9,
    guidewireSpacing: 5, guidewireType: 'glidewire',
    guidewireShaftStiffness: 10, guidewireTipStiffness: 4.55,
    catheterType: 'berenstein', catheterShaftStiffness: 25, catheterTipStiffness: 5,
    relaxationRate: 1, catheterRelaxationRate: 1, maximumCatheterMm: 1000,
    catheterPhysicsSpacing: 4, catheterBodyCount: 320,
    wireAdvanceRate: 44, catheterAdvanceRate: 52, catheterWithdrawRate: 32,
    rotationRate: Math.PI * 0.9
});

/** Actuation/synchronization adapter for simulator.js, with rendering omitted.
 * This is source-audited parity, not a claim of browser pose identity. A future
 * solver can inject a compatible World class without replacing the protocol.
 */
export function createCoupledRuntimeFixture({
    vessel = generateVessel(140, 0).vessel, field = null,
    World = EndovascularPhysicsWorld, coupledSystem = null, jointMotionMode = 'position-history', ...overrides
} = {}) {
    const config = { ...COUPLED_RUNTIME_DEFAULTS, ...overrides, jointMotionMode };
    let dt = config.fixedDt;
    const spacing = config.guidewireSpacing;
    const count = Math.round(config.guidewireLength / spacing) + 1;
    if (Math.abs((count - 1) * spacing - config.guidewireLength) > 1e-8) {
        throw new RangeError('guidewire spacing must divide its material length');
    }
    const wire = new RodState(count, spacing);
    applyGuidewireMaterialProfile(wire, {
        segmentLength: spacing, type: config.guidewireType,
        shaftStiffnessScale: config.guidewireShaftStiffness,
        tipStiffnessScale: config.guidewireTipStiffness
    });
    const transport = new GuidewireTransport({
        rod: wire, segmentLength: spacing, guidewireLength: config.guidewireLength,
        sheath: vessel.sheath, advanceRate: config.wireAdvanceRate,
        minInsert: 0, maxInsert: config.guidewireLength,
        lumenClearance: GUIDEWIRE_RADIUS_MM, meshClearance: GUIDEWIRE_RADIUS_MM
    });
    transport.initialize();
    const catheter = new PigtailCatheter({
        wire, segmentLength: spacing, guidewireLength: config.guidewireLength,
        tailProgressRef: () => transport.progress, vessel,
        maxLength: config.maximumCatheterMm, physicsSpacing: config.catheterPhysicsSpacing
    });
    catheter.setStiffnessScales({
        shaftStiffnessScale: config.catheterShaftStiffness,
        tipStiffnessScale: config.catheterTipStiffness
    });
    const world = new World({ coupledSystem, jointMotionMode, contactField: field, fixedDt: dt, maxSubsteps: 2,
        iterations: 6, penetrationIterations: 8, highPenetration: 0.15, contactActivation: 0.2 });
    const wireBody = world.createRod('guidewire', count, spacing, { ...DEFAULT_TOOL_PROFILES.guidewire });
    wireBody.wallStaticFriction = DEFAULT_TOOL_PROFILES.guidewire.wallFriction;
    wireBody.wallKineticFriction = wireBody.wallFriction = 0.002;
    wireBody.syncFromRodState(wire);
    wireBody.captureKirchhoffRestConfiguration({ captureRestRotation: false });
    const axis = new THREE.Vector3().subVectors(vessel.sheath.end, vessel.sheath.start).normalize();
    const preferredD1 = new THREE.Vector3(0, 0, 1).addScaledVector(axis, -axis.z);
    if (preferredD1.lengthSq() < 1e-8) preferredD1.set(1, 0, 0).addScaledVector(axis, -axis.x);
    preferredD1.normalize();
    let wireRotation = 0, executedSteps = 0;
    const boundary = { twist: 0, segment: 0, preferredD1, compliance: 0, out: {} };
    function applyWireBoundary() {
        boundary.twist = wireRotation;
        boundary.segment = wireBody.activeStart;
        applyProximalTwistBoundary(wireBody, boundary);
    }
    function applyWireProfile() {
        applyKirchhoffMaterialProfile(wireBody, config.guidewireType, {
            activeStart: 0, activeEnd: count - 1,
            materialCoordinates: Float64Array.from({ length: count }, (_, index) => index * spacing),
            tipCoordinate: config.guidewireLength,
            shaftStiffnessScale: config.guidewireShaftStiffness,
            tipStiffnessScale: config.guidewireTipStiffness
        });
        applyWireBoundary();
    }
    applyWireProfile();
    const catheterBody = world.createRod('catheter', config.catheterBodyCount, 4, { ...DEFAULT_TOOL_PROFILES.catheter });
    catheter.syncXpbdBody(catheterBody);
    world.addSheath({ start: vessel.sheath.start, end: vessel.sheath.end,
        innerRadius: INTRODUCER_SHEATH_INNER_RADIUS_MM,
        proximalExtension: CATHETER_PROXIMAL_LOADING_SUPPORT_LENGTH_MM,
        bodies: [wireBody, catheterBody] });
    const containment = world.addContainment(wireBody, catheterBody, {
        model: 'kirchhoff', innerRadius: PIGTAIL_CATHETER_INNER_RADIUS_MM,
        friction: DEFAULT_TOOL_PROFILES.catheter.lumenFriction,
        axialFriction: DEFAULT_TOOL_PROFILES.catheter.lumenAxialFriction,
        torsionalFriction: DEFAULT_TOOL_PROFILES.catheter.lumenTorsionalFriction,
        lumenMaxCorrection: 0.4, openProximal: true, openDistal: true, searchWindow: 2,
        outerStartNode: catheter.physicsLumenStartNode, innerResponse: 1, outerResponse: 1,
        enforceDistalPortal: true, containedLength: 0, enabled: false
    });
    const externalContact = world.addToolContact(wireBody, catheterBody, {
        friction: 0.08, openDistalB: true, enabled: false
    });

    function reset() {
        transport.reset();
        catheter.reset();
        wireBody.syncFromRodState(wire);
        wireBody.captureKirchhoffRestConfiguration({ captureRestRotation: false });
        wireRotation = executedSteps = 0;
        applyWireProfile();
        catheter.syncXpbdBody(catheterBody);
        containment.enabled = false;
        containment.enforceDistalPortal = true;
        externalContact.enabled = false;
        world.resetSimulationState();
        for (const body of world.bodies) body.kirchhoffLengthSweepReverse = false;
    }
    reset();

    function step(commands = {}) {
        const rotationCommand = commands.guidewireRotation ?? 0;
        if (rotationCommand) {
            wireRotation += rotationCommand * config.rotationRate * dt;
            wireRotation = Math.atan2(Math.sin(wireRotation), Math.cos(wireRotation));
        }
        transport.advance(commands.guidewireAdvance ?? 0, dt);
        const inserted = transport.progress;
        // Runtime selects the catheter type after wire feed, before catheter feed.
        catheter.setType(commands.catheterType ?? config.catheterType);
        catheter.advance(commands.catheterAdvance ?? 0, dt, inserted);
        catheter.rotate(commands.catheterRotation ?? 0, dt);
        wireBody.syncFromRodState(wire, { resetVelocity: false });
        wireBody.setActiveRange(Math.min(count - 2, Math.max(0, transport.firstInsertedNodeIndex() - 1)), count - 1);
        applyWireBoundary();
        let wallStart = Math.max(0, transport.firstLumenNodeIndex() - 1);
        wireBody.setSheathMaterialEndNode(wallStart);
        let wallEnd = wireBody.segmentCount - 1;
        catheter.stepPhysics(dt);
        const catheterCount = catheter.syncXpbdBody(catheterBody);
        const firstContainedNode = Math.max(0, Math.ceil((config.guidewireLength - inserted) / spacing));
        const materialEndNode = Math.min(count - 1, Math.floor((config.guidewireLength - inserted + catheter.progress) / spacing));
        world.updateContainmentWindow(containment, {
            enabled: catheter.progress > 0.5 && catheterCount >= 2 && materialEndNode >= firstContainedNode,
            outerStartNode: catheter.physicsLumenStartNode,
            startNode: firstContainedNode, endNode: Math.max(firstContainedNode, materialEndNode),
            innerArcOffset: firstContainedNode * spacing - config.guidewireLength + inserted,
            containedLength: Math.min(catheter.progress, inserted), enforceDistalPortal: true
        });
        wireBody.nodeRadius.fill(GUIDEWIRE_RADIUS_MM);
        wireBody.maxFrameDisplacement = Infinity;
        wireBody.frameDisplacementStartNode = Math.max(wireBody.activeStart, containment.endNode);
        if (containment.enabled) {
            const firstExposed = Math.max(wallStart, materialEndNode);
            if (firstExposed <= wireBody.activeEnd - 1) wallStart = firstExposed;
            else { wallStart = wireBody.activeEnd; wallEnd = wireBody.activeEnd - 1; }
        }
        wireBody.setCollisionRange(wallStart, wallEnd);
        const catheterEnd = Math.max(0, catheterCount - 2);
        const firstExternal = Math.max(0, Math.min(wireBody.segmentCount - 1, materialEndNode + 1));
        externalContact.enabled = catheter.progress > 4 && catheterCount >= 2 &&
            inserted > catheter.progress + 0.5 && firstExternal <= wireBody.activeEnd - 1;
        externalContact.startSegmentA = firstExternal;
        externalContact.endSegmentA = Math.min(wireBody.activeEnd - 1, firstExternal + 16);
        externalContact.startSegmentB = Math.max(0, catheterEnd - 8);
        externalContact.endSegmentB = catheterEnd;
        const coupled = containment.enabled || externalContact.enabled;
        wireBody.relaxationPasses = guidewireRelaxationPasses(config.relaxationRate);
        catheterBody.relaxationPasses = guidewireRelaxationPasses(config.catheterRelaxationRate);
        wireBody.projectionVelocityRetention = coupled ? 0.005 : 1;
        wireBody.toolProjectionVelocityRetention = coupled ? 0 : 1;
        wireBody.distalProjectionVelocityRetention = 1;
        wireBody.distalProjectionVelocityRetentionStartNode = coupled
            ? Math.max(wireBody.activeStart, firstFreeGuidewireNodeAfterContainment({
                activeStart: wireBody.activeStart, activeEnd: wireBody.activeEnd,
                containmentEndNode: containment.endNode })) : Infinity;
        const stepResult = world.stepFixed();
        // A diagnostic run stops on rejection. Actuation was prepared before
        // World entered its transaction; it is not a completed physics step.
        if (stepResult?.accepted === false) return {
            wireMm: inserted, catheterMm: catheter.progress, executedSteps,
            accepted: false, status: stepResult.status
        };
        const captured = spatiallyCapturedContainmentEnd({
            innerBody: wireBody, outerBody: catheterBody, firstContainedNode, materialEndNode,
            outerStartNode: containment.outerStartNode, outerInnerRadius: containment.innerRadius,
            closestSegment: containment.closestSegment
        });
        containment.renderEndNode = Math.min(materialEndNode, Math.max(firstContainedNode, materialEndNode - 1, captured));
        wireBody.syncToRodState(wire);
        executedSteps++;
        return { wireMm: inserted, catheterMm: catheter.progress, executedSteps };
    }
    function snapshot() {
        return { wireMm: transport.progress, catheterMm: catheter.progress,
            wireRotation, catheterRotation: catheter.rotation, executedSteps,
            fingerprints: world.bodies.map(body => ({ id: body.id, hash: poseFingerprint(body) })),
            activeNodes: world.bodies.map(body => body.activeEnd - body.activeStart + 1) };
    }
    // Diagnostic phase boundary only: all actuator/component/world clocks
    // change together while the prepared mechanical state stays untouched.
    // Damping conversion is explicit in the timestep-comparison driver.
    function setFixedDtForComparison(nextDt) {
        if (!Number.isFinite(nextDt) || nextDt <= 0) throw new RangeError('A positive finite timestep is required');
        dt = config.fixedDt = world.fixedDt = nextDt;
    }
    return { config, vessel, world, wire, wireBody, catheterBody, catheter, transport, setFixedDtForComparison,
        containment, externalContact, step, reset, snapshot, dispose: () => catheter.dispose() };
}

/** The same transformed STL and packed field settings as aortaModel.js. */
export async function loadCoupledRuntimeAnatomy(root = new URL('../../', import.meta.url)) {
    const [{ readFileSync }, { STLLoader }, { MeshBVH }, { transformAortaGeometry },
        { decodeCollisionAsset }, { VesselContactField }] = await Promise.all([
        import('node:fs'), import('three/examples/jsm/loaders/STLLoader.js'), import('three-mesh-bvh'),
        import('../../src/aortaTransform.js'), import('../../src/physics/collision/collisionAssetFormat.js'),
        import('../../src/physics/collision/vesselContactField.js')
    ]);
    const buffer = name => {
        const bytes = readFileSync(new URL(`res/${name}`, root));
        return bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength);
    };
    const asset = decodeCollisionAsset(buffer('Aorta_plain.collision.bin'));
    const geometry = new STLLoader().parse(buffer('Aorta_plain.stl'));
    const { vessel } = generateVessel(140, 0);
    transformAortaGeometry(geometry, vessel);
    geometry.computeBoundingBox();
    geometry.boundsTree = new MeshBVH(geometry);
    const field = new VesselContactField(asset, { fallbackGeometry: geometry,
        bvhValidationDistance: 0.02, capsuleBvhValidation: -0.1 });
    return { vessel, field, geometry, dispose: () => geometry.dispose() };
}
