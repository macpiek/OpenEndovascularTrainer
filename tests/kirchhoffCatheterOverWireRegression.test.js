import assert from 'node:assert/strict';
import test from 'node:test';
import { ElasticRod } from '../src/physics/elasticRod.js';
import { applyKirchhoffMaterialProfile } from '../src/physics/applyKirchhoffMaterialProfile.js';
import {
    DEFAULT_TOOL_PROFILES,
    EndovascularPhysicsWorld
} from '../src/physics/endovascularPhysicsWorld.js';
import {
    CATHETER_PROXIMAL_LOADING_SUPPORT_LENGTH_MM,
    PigtailCatheter
} from '../src/pigtailCatheter.js';
import {
    INTRODUCER_SHEATH_INNER_RADIUS_MM,
    PIGTAIL_CATHETER_INNER_RADIUS_MM
} from '../src/toolDimensions.js';
import { guidewireRelaxationPasses } from '../src/physics/guidewireRelaxationRate.js';

const DT = 1 / 120;
const CATHETER_SHAFT_STIFFNESS = 25;
const CATHETER_TIP_STIFFNESS = 5;
const GUIDEWIRE_SHAFT_STIFFNESS = 10;
const GUIDEWIRE_TIP_STIFFNESS = 4.55;
const VERBOSE_COUPLING_DIAGNOSTICS =
    process.env.OET_VERBOSE_COUPLING === '1';

function maximumBendDegrees(body) {
    let maximum = 0;
    for (let node = body.activeStart + 1; node < body.activeEnd; node++) {
        const ax = body.x[node] - body.x[node - 1];
        const ay = body.y[node] - body.y[node - 1];
        const az = body.z[node] - body.z[node - 1];
        const bx = body.x[node + 1] - body.x[node];
        const by = body.y[node + 1] - body.y[node];
        const bz = body.z[node + 1] - body.z[node];
        const denominator = Math.hypot(ax, ay, az) * Math.hypot(bx, by, bz);
        if (denominator <= 1e-9) continue;
        const cosine = Math.max(-1, Math.min(1,
            (ax * bx + ay * by + az * bz) / denominator
        ));
        maximum = Math.max(maximum, Math.acos(cosine) * 180 / Math.PI);
    }
    return maximum;
}

for (const constitutiveSolver of ['local', 'direct']) {
test(`a catheter advances over a held guidewire without dragging or kinking it (${constitutiveSolver})`, () => {
    // Match the first full browser-benchmark feed cycle.  The previous
    // 180-step fixture stopped after only 78 mm of catheter travel and could
    // not see the late portal/topology instability which starts after several
    // seconds of continuous insertion.
    const guidewireLength = 1000;
    const guidewireSpacing = 5;
    const guidewireInserted = 660;
    const sheath = {
        start: { x: -20, y: 0, z: 0 },
        end: { x: 0, y: 0, z: 0 }
    };
    const wire = new ElasticRod(
        guidewireLength / guidewireSpacing + 1,
        guidewireSpacing
    );
    for (let index = 0; index < wire.nodes.length; index++) {
        const node = wire.nodes[index];
        node.x = sheath.start.x +
            index * guidewireSpacing - guidewireLength + guidewireInserted;
        node.y = 0;
        node.z = 0;
        node.vx = 0;
        node.vy = 0;
        node.vz = 0;
    }

    const catheter = new PigtailCatheter({
        wire,
        segmentLength: guidewireSpacing,
        guidewireLength,
        tailProgressRef: () => guidewireInserted,
        vessel: { sheath, segments: [] },
        maxLength: 650
    });
    catheter.setType('berenstein');
    catheter.setStiffnessScales({
        shaftStiffnessScale: CATHETER_SHAFT_STIFFNESS,
        tipStiffnessScale: CATHETER_TIP_STIFFNESS
    });
    catheter.setExternalCollisionSolver(true);

    const world = new EndovascularPhysicsWorld({
        fixedDt: DT,
        iterations: 6,
        penetrationIterations: 8,
        coupledClosureMaxPasses: Number.parseInt(
            process.env.OET_COUPLED_CLOSURE_PASSES ?? '32',
            10
        )
    });
    world.captureCoupledClosureTrace =
        process.env.OET_TRACE_COUPLED_CLOSURE === '1';
    const wireBody = world.createRod(
        'catheter-over-wire-guidewire',
        wire.nodes.length,
        guidewireSpacing,
        { ...DEFAULT_TOOL_PROFILES.guidewire, rodModel: 'kirchhoff' }
    );
    wireBody.syncFromElasticRod(wire);
    wireBody.captureKirchhoffRestConfiguration({ captureRestRotation: false });
    const materialCoordinates = Float64Array.from(
        { length: wireBody.count },
        (_, index) => index * guidewireSpacing
    );
    applyKirchhoffMaterialProfile(wireBody, 'glidewire', {
        activeStart: 0,
        activeEnd: wireBody.count - 1,
        materialCoordinates,
        tipCoordinate: guidewireLength,
        shaftStiffnessScale: GUIDEWIRE_SHAFT_STIFFNESS,
        tipStiffnessScale: GUIDEWIRE_TIP_STIFFNESS
    });

    const catheterBody = world.createRod(
        'catheter-over-wire-catheter',
        320,
        4,
        { ...DEFAULT_TOOL_PROFILES.catheter, rodModel: 'kirchhoff' }
    );
    // Match the accepted application defaults. These extra constitutive
    // sweeps must be interleaved with lumen contact; running them body-local
    // is precisely the unstable configuration covered by this regression.
    const relaxationPasses = guidewireRelaxationPasses(30);
    wireBody.constitutiveSolver = constitutiveSolver;
    wireBody.relaxationPasses = constitutiveSolver === 'direct' ? 0 : relaxationPasses;
    catheterBody.relaxationPasses = relaxationPasses;
    if (process.env.OET_UNLIMITED_CATHETER_SPEED === '1') {
        catheterBody.maxSpeed = Infinity;
    }
    catheter.syncXpbdBody(catheterBody);
    world.addSheath({
        start: sheath.start,
        end: sheath.end,
        innerRadius: INTRODUCER_SHEATH_INNER_RADIUS_MM,
        proximalExtension: CATHETER_PROXIMAL_LOADING_SUPPORT_LENGTH_MM,
        bodies: [wireBody, catheterBody]
    });
    const containment = world.addContainment(wireBody, catheterBody, {
        model: 'kirchhoff',
        innerRadius: PIGTAIL_CATHETER_INNER_RADIUS_MM,
        friction: DEFAULT_TOOL_PROFILES.catheter.lumenFriction,
        axialFriction:
            DEFAULT_TOOL_PROFILES.catheter.lumenAxialFriction,
        torsionalFriction:
            DEFAULT_TOOL_PROFILES.catheter.lumenTorsionalFriction,
        openProximal: true,
        openDistal: true,
        searchWindow: 2,
        enabled: false
    });

    const firstInsertedNode = Math.ceil(
        (guidewireLength - guidewireInserted) / guidewireSpacing
    );
    wireBody.setActiveRange(firstInsertedNode - 1, wireBody.count - 1);
    wireBody.setCollisionRange(wireBody.activeEnd, wireBody.activeEnd - 1);
    wireBody.setSheathMaterialEndNode(firstInsertedNode + 10);
    const lastSheathNode = firstInsertedNode +
        Math.ceil(Math.hypot(
            sheath.end.x - sheath.start.x,
            sheath.end.y - sheath.start.y,
            sheath.end.z - sheath.start.z
        ) / guidewireSpacing);
    for (
        let node = firstInsertedNode - 1;
        node <= Math.min(lastSheathNode, wireBody.activeEnd);
        node++
    ) {
        wireBody.setPinned(node, true);
    }
    const initialWireX = Float64Array.from(wireBody.x);
    const initialWireY = Float64Array.from(wireBody.y);
    const initialWireZ = Float64Array.from(wireBody.z);

    let maximumWireDisplacement = 0;
    let maximumWireDisplacementNode = -1;
    let maximumWireDisplacementVector = [0, 0, 0];
    let maximumWireBend = 0;
    let maximumCatheterBend = 0;
    let maximumTipGuideDistance = 0;
    let finalTipGuideDistance = Infinity;
    let firstCatheterBend = null;
    const timingBlocks = [];
    let timingBlockStartedAt = performance.now();
    try {
        const totalSteps = Math.max(
            1,
            Number.parseInt(process.env.OET_LONG_COUPLING_STEPS ?? '1200', 10)
        );
        for (let step = 0; step < totalSteps; step++) {
            catheter.advance(1, DT, guidewireInserted);
            catheter.stepPhysics(DT, { collisions: false });
            const catheterCount = catheter.syncXpbdBody(catheterBody);
            if (process.env.OET_ZERO_CATHETER_CURVATURE === '1') {
                catheterBody.restRotation1.fill(0);
                catheterBody.restRotation2.fill(0);
                catheterBody.restRotation3.fill(0);
            }
            const lastContainedNode = Math.min(
                wireBody.count - 1,
                Math.floor(
                    (guidewireLength - guidewireInserted + catheter.progress) /
                        guidewireSpacing
                )
            );
            world.updateContainmentWindow(containment, {
                enabled:
                    process.env.OET_DISABLE_KIRCHHOFF_CONTAINMENT !== '1' &&
                    catheter.progress > 0.5 &&
                    catheterCount >= 2 &&
                    lastContainedNode >= firstInsertedNode,
                outerStartNode: catheter.physicsLumenStartNode,
                startNode: firstInsertedNode,
                endNode: Math.max(firstInsertedNode, lastContainedNode),
                innerArcOffset:
                    firstInsertedNode * guidewireSpacing -
                    guidewireLength + guidewireInserted,
                containedLength: Math.min(
                    catheter.progress,
                    guidewireInserted
                ),
                enforceDistalPortal: true
            });
            wireBody.projectionVelocityRetention = containment.enabled
                ? 0.005
                : 1;
            wireBody.distalProjectionVelocityRetention = 1;
            wireBody.distalProjectionVelocityRetentionStartNode =
                containment.enabled ? containment.endNode + 19 : Infinity;

            world.stepFixed();
            maximumWireBend = Math.max(
                maximumWireBend,
                maximumBendDegrees(wireBody)
            );
            const currentCatheterBend = maximumBendDegrees(catheterBody);
            maximumCatheterBend = Math.max(
                maximumCatheterBend,
                currentCatheterBend
            );
            if (!firstCatheterBend && currentCatheterBend > 1) {
                const stats = world.getStats();
                const catheterStats = stats.bodies.find(
                    entry => entry.id === catheterBody.id
                );
                firstCatheterBend = {
                    step: step + 1,
                    progress: catheter.progress,
                    count: catheterCount,
                    bend: currentCatheterBend,
                    maxBendNode: catheterStats?.maxBendNode,
                    maxSpeed: catheterStats?.maxSpeed,
                    lumenStart: catheter.physicsLumenStartNode,
                    sheathEnd: catheterBody.sheathMaterialEndNode,
                    nodes: Array.from(
                        { length: 12 },
                        (_, offset) => 12 + offset
                    ).map(node => ({
                        node,
                        material: catheterBody.materialCoordinate[node],
                        x: catheterBody.x[node],
                        y: catheterBody.y[node],
                        z: catheterBody.z[node],
                        previousX: catheterBody.previousX[node],
                        previousY: catheterBody.previousY[node],
                        previousZ: catheterBody.previousZ[node],
                        vx: catheterBody.velocityX[node],
                        vy: catheterBody.velocityY[node],
                        vz: catheterBody.velocityZ[node],
                        restLength: catheterBody.restLength[node]
                    }))
                };
            }
            for (let node = wireBody.activeStart; node <= wireBody.activeEnd; node++) {
                const displacement = [
                    wireBody.x[node] - initialWireX[node],
                    wireBody.y[node] - initialWireY[node],
                    wireBody.z[node] - initialWireZ[node]
                ];
                const displacementMagnitude = Math.hypot(...displacement);
                if (displacementMagnitude > maximumWireDisplacement) {
                    maximumWireDisplacement = displacementMagnitude;
                    maximumWireDisplacementNode = node;
                    maximumWireDisplacementVector = displacement;
                }
            }
            const tip = catheterBody.activeEnd;
            const wireCoordinate = (
                guidewireLength - guidewireInserted + catheter.progress
            ) / guidewireSpacing;
            const wireNode = Math.min(
                wireBody.activeEnd - 1,
                Math.max(wireBody.activeStart, Math.floor(wireCoordinate))
            );
            const wireT = Math.max(0, Math.min(1, wireCoordinate - wireNode));
            finalTipGuideDistance = Math.hypot(
                catheterBody.x[tip] - (
                    wireBody.x[wireNode] +
                    (wireBody.x[wireNode + 1] - wireBody.x[wireNode]) * wireT
                ),
                catheterBody.y[tip] - (
                    wireBody.y[wireNode] +
                    (wireBody.y[wireNode + 1] - wireBody.y[wireNode]) * wireT
                ),
                catheterBody.z[tip] - (
                    wireBody.z[wireNode] +
                    (wireBody.z[wireNode + 1] - wireBody.z[wireNode]) * wireT
                )
            );
            if (catheter.progress >= 8) {
                maximumTipGuideDistance = Math.max(
                    maximumTipGuideDistance,
                    finalTipGuideDistance
                );
            }
            if ((step + 1) % 120 === 0) {
                const now = performance.now();
                const stats = world.getStats();
                let mappingReversals = 0;
                let maximumMappingJump = 0;
                let previousOuterSegment = -1;
                for (
                    let innerSegment = containment.startNode;
                    innerSegment < containment.endNode;
                    innerSegment++
                ) {
                    const outerSegment =
                        containment.kirchhoffOuterSegmentByInner[innerSegment];
                    if (outerSegment < 0) continue;
                    if (previousOuterSegment >= 0) {
                        if (outerSegment < previousOuterSegment) {
                            mappingReversals++;
                        }
                        maximumMappingJump = Math.max(
                            maximumMappingJump,
                            Math.abs(outerSegment - previousOuterSegment)
                        );
                    }
                    previousOuterSegment = outerSegment;
                }
                timingBlocks.push({
                    step: step + 1,
                    catheterProgressMm: Number(catheter.progress.toFixed(2)),
                    averageStepMs: Number(((now - timingBlockStartedAt) / 120).toFixed(3)),
                    constraintsMs: Number(stats.phases.constraints.averageMs.toFixed(3)),
                    primaryMs: Number(stats.phases.constraintPrimary.averageMs.toFixed(3)),
                    bodyClosureMs: Number(stats.phases.constraintBodyClosure.averageMs.toFixed(3)),
                    coupledClosureMs: Number(stats.phases.constraintCoupledClosure.averageMs.toFixed(3)),
                    coupledClosurePasses: world.lastCoupledClosurePasses,
                    containmentViolation: containment.kirchhoffMaxViolation,
                    containmentContacts: containment.kirchhoffContacts.length,
                    manifoldSize: containment.manifold.size,
                    mappingReversals,
                    maximumMappingJump,
                    maxWireBendDegrees: Number(maximumWireBend.toFixed(2)),
                    maxCatheterBendDegrees: Number(maximumCatheterBend.toFixed(2))
                });
                timingBlockStartedAt = now;
                world.resetPerformanceStats();
            }
        }

        let catheterRestLength = 0;
        let catheterActualLength = 0;
        let catheterMaximumSegmentError = 0;
        let catheterMaximumSegmentErrorIndex = -1;
        let catheterMaximumSegmentActualLength = 0;
        let catheterMaximumSegmentRestLength = 0;
        for (
            let segment = catheter.physicsLumenStartNode;
            segment < catheterBody.activeEnd;
            segment++
        ) {
            const actualLength = Math.hypot(
                catheterBody.x[segment + 1] - catheterBody.x[segment],
                catheterBody.y[segment + 1] - catheterBody.y[segment],
                catheterBody.z[segment + 1] - catheterBody.z[segment]
            );
            catheterRestLength += catheterBody.restLength[segment];
            catheterActualLength += actualLength;
            const segmentError = Math.abs(
                actualLength - catheterBody.restLength[segment]
            );
            if (segmentError > catheterMaximumSegmentError) {
                catheterMaximumSegmentError = segmentError;
                catheterMaximumSegmentErrorIndex = segment;
                catheterMaximumSegmentActualLength = actualLength;
                catheterMaximumSegmentRestLength =
                catheterBody.restLength[segment];
            }
        }
        let guidewireMaximumSegmentError = 0;
        for (
            let segment = wireBody.activeStart;
            segment < wireBody.activeEnd;
            segment++
        ) {
            const actualLength = Math.hypot(
                wireBody.x[segment + 1] - wireBody.x[segment],
                wireBody.y[segment + 1] - wireBody.y[segment],
                wireBody.z[segment + 1] - wireBody.z[segment]
            );
            guidewireMaximumSegmentError = Math.max(
                guidewireMaximumSegmentError,
                Math.abs(actualLength - wireBody.restLength[segment])
            );
        }
        let guidewireAnchorError = 0;
        for (
            let node = firstInsertedNode - 1;
            node <= Math.min(lastSheathNode, wireBody.activeEnd);
            node++
        ) {
            guidewireAnchorError = Math.max(
                guidewireAnchorError,
                Math.hypot(
                    wireBody.x[node] - initialWireX[node],
                    wireBody.y[node] - initialWireY[node],
                    wireBody.z[node] - initialWireZ[node]
                )
            );
        }
        const inlet = catheter.physicsLumenStartNode;
        const tip = catheterBody.activeEnd;
        const inletError = Math.hypot(
            catheterBody.x[inlet] - sheath.start.x,
            catheterBody.y[inlet] - sheath.start.y,
            catheterBody.z[inlet] - sheath.start.z
        );
        const finalWorldStats = world.getStats();
        const finalCatheterStats = finalWorldStats.bodies.find(
            entry => entry.id === catheterBody.id
        );
        const catheterWindow = [];
        for (
            let node = Math.max(0, finalCatheterStats.maxBendNode - 3);
            node <= Math.min(
                catheterBody.activeEnd,
                finalCatheterStats.maxBendNode + 3
            );
            node++
        ) {
            const segment = Math.min(node, catheterBody.segmentCount - 1);
            const qx = catheterBody.orientationX[segment];
            const qy = catheterBody.orientationY[segment];
            const qz = catheterBody.orientationZ[segment];
            const qw = catheterBody.orientationW[segment];
            catheterWindow.push({
                node,
                materialCoordinate: catheterBody.materialCoordinate[node],
                position: [
                    Number(catheterBody.x[node].toFixed(3)),
                    Number(catheterBody.y[node].toFixed(3)),
                    Number(catheterBody.z[node].toFixed(3))
                ],
                restLength: catheterBody.restLength[segment],
                director3: [
                    Number((2 * (qx * qz + qw * qy)).toFixed(3)),
                    Number((2 * (qy * qz - qw * qx)).toFixed(3)),
                    Number((1 - 2 * (qx * qx + qy * qy)).toFixed(3))
                ],
                restRotation: [
                    catheterBody.restRotation1[segment],
                    catheterBody.restRotation2[segment],
                    catheterBody.restRotation3[segment]
                ]
            });
        }
        if (VERBOSE_COUPLING_DIAGNOSTICS) {
            console.log('first catheter bend diagnostics', JSON.stringify(
                firstCatheterBend,
                null,
                2
            ));
            console.log('long catheter-over-wire diagnostics', {
                timingBlocks,
                firstCatheterBend,
                finalBodyStats: finalWorldStats.bodies,
                catheterWindow,
                catheterActiveRange: [
                    catheterBody.activeStart,
                    catheterBody.activeEnd
                ],
                catheterLumenStartNode: catheter.physicsLumenStartNode,
                catheterCollisionStartSegment:
                    catheterBody.collisionStartSegment,
                catheterSheathMaterialEndNode:
                    catheterBody.sheathMaterialEndNode,
                catheterControlNode:
                    catheterBody.controlEnabled.findIndex(value => value === 1),
                catheterMaximumSegmentErrorIndex,
                catheterMaximumSegmentActualLength,
                catheterMaximumSegmentRestLength,
                catheterPostPasses: catheterBody.lastPostStabilizationPasses,
                catheterPostResidual: catheterBody.lastPostStabilizationResidual,
                inletErrorMm: inletError,
                finalTipGuideDistanceMm: finalTipGuideDistance,
                maximumTipGuideDistanceMm: maximumTipGuideDistance,
                catheterMaximumSegmentErrorMm: catheterMaximumSegmentError,
                guidewireMaximumSegmentErrorMm:
                    guidewireMaximumSegmentError,
                guidewireAnchorErrorMm: guidewireAnchorError,
                maximumCatheterBendDegrees: maximumCatheterBend,
                maximumWireDisplacementMm: maximumWireDisplacement,
                maximumWireDisplacementNode,
                maximumWireDisplacementVectorMm:
                    maximumWireDisplacementVector,
                maximumWireBendDegrees: maximumWireBend,
                coupledClosureTrace: world.coupledClosureTrace
            });
            if (world.captureCoupledClosureTrace) {
                console.log('coupled closure trace', JSON.stringify(
                    world.coupledClosureTrace,
                    null,
                    2
                ));
            }
        }
        assert.equal(
            catheterBody.controlEnabled.findIndex(value => value === 1),
            inlet,
            'proximal feed must act at the physical introducer inlet'
        );
        assert.ok(inletError < 0.5,
            `catheter slipped backwards through the introducer (${inletError} mm)`);
        assert.ok(finalTipGuideDistance < 0.75,
            `catheter tip left the guidewire path (${finalTipGuideDistance} mm)`);
        assert.ok(maximumTipGuideDistance < 1.25,
            `catheter failed to track the guidewire (${maximumTipGuideDistance} mm)`);
        assert.ok(catheterMaximumSegmentError < 0.02,
            `catheter lost inextensibility (${catheterMaximumSegmentError} mm)`);
        assert.ok(maximumCatheterBend < 35,
            `catheter kinked while tracking the guidewire (${maximumCatheterBend} degrees)`);
        assert.ok(guidewireAnchorError < 1e-6,
            `held guidewire slipped at the inlet (${guidewireAnchorError} mm)`);
        assert.ok(guidewireMaximumSegmentError < 0.02,
            `guidewire lost inextensibility (${guidewireMaximumSegmentError} mm)`);
        assert.ok(maximumWireBend < 35,
            `held guidewire kinked during catheter feed (${maximumWireBend} degrees)`);
    } finally {
        catheter.dispose();
    }
});
}
