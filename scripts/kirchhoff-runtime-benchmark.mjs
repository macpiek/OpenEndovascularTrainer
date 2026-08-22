import {
    DEFAULT_TOOL_PROFILES,
    EndovascularPhysicsWorld
} from '../src/physics/endovascularPhysicsWorld.js';

const WARMUP_STEPS = 20;
const MEASURED_STEPS = 40;

function createScenario({
    lumen = false,
    lumenOffset = 0.5,
    catheterPostStabilization = 0,
    wireCount = 201,
    wireSpacing = 2,
    wireActiveStart = 0,
    catheterCount = 121,
    catheterSpacing = 2,
    catheterActiveEnd = catheterCount - 1
} = {}) {
    const world = new EndovascularPhysicsWorld({
        fixedDt: 1 / 120,
        iterations: 6,
        penetrationIterations: 8
    });
    const wire = world.createRod('benchmark-wire', wireCount, wireSpacing, {
        ...DEFAULT_TOOL_PROFILES.guidewire,
        rodModel: 'kirchhoff',
        postStabilizationPasses: 0
    });
    const catheter = world.createRod(
        'benchmark-catheter',
        catheterCount,
        catheterSpacing,
        {
        ...DEFAULT_TOOL_PROFILES.catheter,
        rodModel: 'kirchhoff',
        postStabilizationPasses: catheterPostStabilization
        }
    );
    for (let index = 0; index < wire.count; index++) {
        wire.x[index] = (index - wireActiveStart) * wireSpacing;
        wire.previousX[index] = wire.x[index];
        wire.y[index] = lumen ? lumenOffset : 0;
        wire.previousY[index] = wire.y[index];
    }
    wire.setActiveRange(wireActiveStart, wire.count - 1);
    catheter.setActiveRange(0, catheterActiveEnd);
    wire.captureRestConfiguration();
    catheter.captureRestConfiguration();
    if (lumen) {
        world.addContainment(wire, catheter, {
            model: 'kirchhoff',
            innerRadius: 1,
            startNode: 0,
            endNode: catheter.count,
            outerStartNode: 0,
            openDistal: true,
            friction: 0.04,
            innerResponse: 1,
            outerResponse: 0,
            portalInnerResponse: 1,
            portalOuterResponse: 0
        });
    }
    return { world, wire, catheter };
}

function run(name, options) {
    const { world, wire, catheter } = createScenario(options);
    for (let step = 0; step < WARMUP_STEPS; step++) {
        wire.wake();
        catheter.wake();
        world.stepFixed();
    }
    world.resetPerformanceStats();
    const stepTimes = new Float64Array(MEASURED_STEPS);
    for (let step = 0; step < MEASURED_STEPS; step++) {
        wire.wake();
        catheter.wake();
        const startedAt = performance.now();
        world.stepFixed();
        stepTimes[step] = performance.now() - startedAt;
    }
    const orderedStepTimes = Array.from(stepTimes).sort((a, b) => a - b);
    const percentile = fraction => orderedStepTimes[
        Math.floor((orderedStepTimes.length - 1) * fraction)
    ];
    const phases = world.getStats().phases;
    return {
        name,
        nodes: wire.count + catheter.count,
        measuredSteps: MEASURED_STEPS,
        totalMedianMs: percentile(0.5),
        totalP25Ms: percentile(0.25),
        constraintsAverageMs: phases.constraints.averageMs,
        constraintsP95Ms: phases.constraints.p95Ms,
        constraintSectionsAverageMs: {
            primary: phases.constraintPrimary.averageMs,
            bodyClosure: phases.constraintBodyClosure.averageMs,
            coupledClosure: phases.constraintCoupledClosure.averageMs,
            movingClosure: phases.constraintMovingClosure.averageMs
        },
        totalAverageMs: phases.total.averageMs,
        totalP95Ms: phases.total.p95Ms
    };
}

const scenarioDefinitions = [
    ['free-rods', { lumen: false }],
    ['nearby-unloaded-lumen', { lumen: true }],
    ['loaded-lumen-with-catheter-closure', {
        lumen: true,
        lumenOffset: 0.8,
        catheterPostStabilization: 4
    }],
    // Exact material discretization and active ranges of the browser's
    // 40 cm wire + 24 cm catheter smoke scenario. Inactive storage stays
    // allocated, just as it does in the application.
    ['browser-active-40cm-wire-24cm-catheter', {
        lumen: true,
        lumenOffset: 0.8,
        catheterPostStabilization: 4,
        wireCount: 201,
        wireSpacing: 5,
        wireActiveStart: 120,
        catheterCount: 320,
        catheterSpacing: 4,
        catheterActiveEnd: 60
    }]
];
const requestedScenario = process.argv[2];
const selectedDefinitions = requestedScenario
    ? scenarioDefinitions.filter(([name]) => name === requestedScenario)
    : scenarioDefinitions;
if (requestedScenario && selectedDefinitions.length === 0) {
    throw new RangeError(`Unknown benchmark scenario: ${requestedScenario}`);
}

console.log(JSON.stringify({
    runtime: 'node-engineering-benchmark',
    note: 'Compare revisions on the same machine; browser acceptance remains authoritative.',
    scenarios: selectedDefinitions.map(([name, options]) => run(name, options))
}, null, 2));
