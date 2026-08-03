import assert from 'node:assert/strict';
import * as THREE from 'three';
import {
    computeInjectionHydraulics,
    DEFAULT_DEVICE_HYDRAULIC_PROFILES,
    mergeDeviceHydraulicProfiles,
    normalizeDeviceHydraulicProfile
} from '../src/contrast/injectionHydraulics.js';
import {
    CONTRAST_SOURCE_CATHETER,
    CONTRAST_SOURCE_SHEATH,
    HybridContrastSystem
} from '../src/contrast/hybridContrastSystem.js';
import { ContrastFlowNetwork } from '../src/contrast/flowNetwork.js';
import { LocalContrastInjectionSolver } from '../src/contrast/localInjectionSolver.js';
import { PressureDrivenRetrogradeColumn } from '../src/contrast/pressureDrivenRetrogradeColumn.js';

const medium = {
    iodineMgPerMl: 300,
    densityKgPerM3: 1349,
    viscosityPaS: 0.0063
};

function port(radiusMm, direction = [0, -1, 0]) {
    const areaMm2 = Math.PI * radiusMm ** 2;
    return {
        position: new THREE.Vector3(0, 40, 0),
        direction: new THREE.Vector3(...direction).normalize(),
        radiusMm,
        areaMm2,
        weight: areaMm2,
        valid: true
    };
}

const sheathPorts = [{ ...port(0.9), kind: 'sheath-end' }];
const berensteinPorts = [{ ...port(0.485), kind: 'berenstein-end' }];
const pigtailPorts = Array.from({ length: 8 }, (_, index) => ({
    ...port(0.22, [Math.cos(index * Math.PI / 4), 0, Math.sin(index * Math.PI / 4)]),
    kind: 'pigtail-side'
}));

const sheathFast = computeInjectionHydraulics({
    requestedRateMlPerSec: 29.1,
    ports: sheathPorts,
    deviceProfile: DEFAULT_DEVICE_HYDRAULIC_PROFILES.sheath,
    medium
});
const berensteinFast = computeInjectionHydraulics({
    requestedRateMlPerSec: 29.1,
    ports: berensteinPorts,
    deviceProfile: DEFAULT_DEVICE_HYDRAULIC_PROFILES.berenstein,
    medium
});
const pigtailFast = computeInjectionHydraulics({
    requestedRateMlPerSec: 29.1,
    ports: pigtailPorts,
    deviceProfile: DEFAULT_DEVICE_HYDRAULIC_PROFILES.pigtail,
    medium
});
const berensteinSlow = computeInjectionHydraulics({
    requestedRateMlPerSec: 4,
    ports: berensteinPorts,
    deviceProfile: DEFAULT_DEVICE_HYDRAULIC_PROFILES.berenstein,
    medium
});

assert.equal(
    sheathFast.actualRateMlPerSec,
    29.1,
    'a short large-bore sheath should achieve the requested 29.1 ml/s within its pressure rating'
);
assert.equal(sheathFast.pressureLimited, false);
assert.ok(
    berensteinFast.actualRateMlPerSec < 15,
    `a 5F Berenstein must not silently deliver 29.1 ml/s (${berensteinFast.actualRateMlPerSec})`
);
assert.equal(berensteinFast.pressureLimited, true);
assert.ok(
    Math.abs(
        berensteinFast.actualRateMlPerSec -
        berensteinFast.maximumAchievableRateMlPerSec
    ) < 1e-9,
    'a pressure-limited request should stop at the calculated slider maximum'
);
assert.ok(
    Math.abs(
        berensteinSlow.maximumAchievableRateMlPerSec -
        berensteinFast.maximumAchievableRateMlPerSec
    ) < 1e-9,
    'maximum achievable flow must be independent of the currently requested rate'
);
assert.equal(berensteinSlow.actualRateMlPerSec, 4);
assert.equal(berensteinFast.limitingComponent, 'device');
assert.ok(
    Math.abs(berensteinFast.appliedPressurePsi - 1050) < 1e-6,
    'Berenstein delivery should stop at the explicit device pressure rating'
);
assert.ok(
    pigtailFast.actualRateMlPerSec > berensteinFast.actualRateMlPerSec,
    'multiple pigtail side holes and its profile should increase achievable flow'
);
assert.ok(
    pigtailFast.maximumAchievableRateMlPerSec >
        berensteinFast.maximumAchievableRateMlPerSec,
    'the pigtail slider maximum should exceed the Berenstein maximum'
);
assert.equal(pigtailFast.outletCount, 8);
assert.ok(
    pigtailFast.portDirections.every(direction =>
        Math.abs(direction.y) < 1e-12
    ),
    'the quantitative profile should preserve the direction of every side hole'
);
assert.ok(
    berensteinFast.requiredPressurePsi >
        berensteinFast.pressureLimitPsi * 4,
    'the UI warning must have a meaningful required-versus-allowed pressure difference'
);

const lowInjectorPressure = computeInjectionHydraulics({
    requestedRateMlPerSec: 29.1,
    ports: pigtailPorts,
    deviceProfile: DEFAULT_DEVICE_HYDRAULIC_PROFILES.pigtail,
    injector: { maximumPressurePsi: 400 },
    medium
});
assert.equal(lowInjectorPressure.limitingComponent, 'injector');
assert.ok(
    lowInjectorPressure.actualRateMlPerSec <
        pigtailFast.actualRateMlPerSec,
    'reducing injector pressure must reduce actual flow'
);

const viscousContrast = computeInjectionHydraulics({
    requestedRateMlPerSec: 29.1,
    ports: berensteinPorts,
    deviceProfile: DEFAULT_DEVICE_HYDRAULIC_PROFILES.berenstein,
    medium: { ...medium, viscosityPaS: 0.02 }
});
assert.ok(
    viscousContrast.actualRateMlPerSec <
        berensteinFast.actualRateMlPerSec,
    'higher viscosity must reduce the achievable catheter flow'
);

const longerBerenstein = computeInjectionHydraulics({
    requestedRateMlPerSec: 29.1,
    ports: berensteinPorts,
    deviceProfile: {
        ...DEFAULT_DEVICE_HYDRAULIC_PROFILES.berenstein,
        lengthMm: 1250
    },
    medium
});
assert.ok(
    longerBerenstein.actualRateMlPerSec <
        berensteinFast.actualRateMlPerSec,
    'a longer catheter must produce more pressure loss'
);

assert.throws(
    () => normalizeDeviceHydraulicProfile({
        ...DEFAULT_DEVICE_HYDRAULIC_PROFILES.berenstein,
        innerDiameterMm: 0
    }),
    /inner diameter/,
    'invalid quantitative device parameters must fail explicitly'
);
assert.throws(
    () => computeInjectionHydraulics({
        requestedRateMlPerSec: 10,
        ports: [],
        deviceProfile: DEFAULT_DEVICE_HYDRAULIC_PROFILES.sheath,
        medium
    }),
    /outlet/,
    'an injection cannot be solved without an outlet'
);

const branchSegments = [
    {
        id: 0,
        nodeStartId: 0,
        nodeEndId: 1,
        start: new THREE.Vector3(0, 0, 0),
        end: new THREE.Vector3(0, 60, 0),
        radiusStart: 10,
        radiusEnd: 9,
        safeRadius: 9
    },
    {
        id: 1,
        nodeStartId: 1,
        nodeEndId: 2,
        start: new THREE.Vector3(0, 60, 0),
        end: new THREE.Vector3(-30, 110, 0),
        radiusStart: 5,
        radiusEnd: 4,
        safeRadius: 4
    },
    {
        id: 2,
        nodeStartId: 1,
        nodeEndId: 3,
        start: new THREE.Vector3(0, 60, 0),
        end: new THREE.Vector3(30, 110, 0),
        radiusStart: 5,
        radiusEnd: 4,
        safeRadius: 4
    }
];
const genericColumnNetwork = new ContrastFlowNetwork(branchSegments, {
    cardiacOutputMlPerMin: 900
});
const genericColumn = new PressureDrivenRetrogradeColumn(
    genericColumnNetwork
);
const genericSourceEdge = genericColumnNetwork.edges[1];
const genericBerensteinPort = {
    ...port(0.485),
    kind: 'berenstein-end',
    position: genericSourceEdge.start.clone().lerp(
        genericSourceEdge.end,
        0.6
    ),
    direction: genericSourceEdge.axis.clone().multiplyScalar(-1)
};
const genericColumnEmission = genericColumn.emit({
    ports: [genericBerensteinPort],
    volumeMl: 0.5,
    rateMlPerSec: 15,
    emissionWindowSeconds: 1 / 30,
    medium
});
assert.equal(
    genericColumnEmission?.mode,
    'pressure-driven-retrograde-column',
    'the bulk reflux transition must be available to a catheter without a source-kind exception'
);
const radialPigtailColumn = new PressureDrivenRetrogradeColumn(
    new ContrastFlowNetwork(branchSegments, {
        cardiacOutputMlPerMin: 900
    })
);
assert.equal(
    radialPigtailColumn.emit({
        ports: pigtailPorts.map(pigtailPort => ({
            ...pigtailPort,
            position: genericBerensteinPort.position
        })),
        volumeMl: 0.5,
        rateMlPerSec: 15,
        emissionWindowSeconds: 1 / 30,
        medium
    }),
    null,
    'radial pigtail outlets should avoid an axial plug through their directions, not a hard-coded device block'
);

const identicalKindNetworkA = new ContrastFlowNetwork(branchSegments, {
    cardiacOutputMlPerMin: 4500
});
const identicalKindNetworkB = new ContrastFlowNetwork(branchSegments, {
    cardiacOutputMlPerMin: 4500
});
const identicalKindSolverA = new LocalContrastInjectionSolver({
    flowNetwork: identicalKindNetworkA,
    randomSeed: 20260802
});
const identicalKindSolverB = new LocalContrastInjectionSolver({
    flowNetwork: identicalKindNetworkB,
    randomSeed: 20260802
});
const identicalSourceEdgeA = identicalKindNetworkA.edges[1];
const identicalSourceEdgeB = identicalKindNetworkB.edges[1];
const identicalPortGeometry = {
    radiusMm: 0.485,
    areaMm2: Math.PI * 0.485 ** 2,
    weight: Math.PI * 0.485 ** 2,
    valid: true
};
identicalKindSolverA.emit({
    ports: [{
        ...identicalPortGeometry,
        kind: 'sheath-end',
        position: identicalSourceEdgeA.start.clone().lerp(
            identicalSourceEdgeA.end,
            0.6
        ),
        direction: identicalSourceEdgeA.axis.clone().multiplyScalar(-1)
    }],
    volumeMl: 0.2,
    rateMlPerSec: 4,
    emissionWindowSeconds: 1 / 30,
    medium
});
identicalKindSolverB.emit({
    ports: [{
        ...identicalPortGeometry,
        kind: 'berenstein-end',
        position: identicalSourceEdgeB.start.clone().lerp(
            identicalSourceEdgeB.end,
            0.6
        ),
        direction: identicalSourceEdgeB.axis.clone().multiplyScalar(-1)
    }],
    volumeMl: 0.2,
    rateMlPerSec: 4,
    emissionWindowSeconds: 1 / 30,
    medium
});
identicalKindSolverA.update(1 / 120);
identicalKindSolverB.update(1 / 120);
assert.equal(identicalKindSolverA.count, identicalKindSolverB.count);
assert.deepEqual(
    [...identicalKindSolverA.positionX.subarray(0, identicalKindSolverA.count)],
    [...identicalKindSolverB.positionX.subarray(0, identicalKindSolverB.count)],
    'identical outlet geometry must produce identical plume physics regardless of sheath/catheter label'
);
assert.deepEqual(
    identicalKindSolverA.getDiagnostics(),
    identicalKindSolverB.getDiagnostics(),
    'source kind must not select a hidden contrast-injection algorithm'
);

const segments = [{
    id: 0,
    nodeStartId: 0,
    nodeEndId: 1,
    start: new THREE.Vector3(0, 0, 0),
    end: new THREE.Vector3(0, 120, 0),
    radiusStart: 7,
    radiusEnd: 7,
    safeRadius: 7
}];
const catheter = {
    type: 'berenstein',
    getInjectionPorts(out) {
        out.length = 0;
        out.push(berensteinPorts[0]);
        return out;
    }
};
const system = new HybridContrastSystem({
    centerlineSegments: segments,
    sheath: {
        start: new THREE.Vector3(0, -30, 0),
        end: new THREE.Vector3(0, 10, 0),
        radius: 1
    },
    catheter,
    medium
});
const catheterPreview = system.getInjectionPreview({
    source: CONTRAST_SOURCE_CATHETER,
    volumeMl: 30,
    rateMlPerSec: 29.1
});
assert.equal(catheterPreview.valid, true);
assert.ok(catheterPreview.maximumAchievableRateMlPerSec > 0);
assert.ok(catheterPreview.durationSeconds > 2);
const started = system.startInjection({
    source: CONTRAST_SOURCE_CATHETER,
    volumeMl: 30,
    rateMlPerSec: 29.1
});
assert.equal(started.ok, true);
assert.equal(
    started.actualRateMlPerSec,
    catheterPreview.actualRateMlPerSec
);
system.update(1 / 30);
assert.ok(
    system.totalDeliveredVolumeMl < 29.1 / 30,
    'delivered mass per step must use actual rather than requested catheter flow'
);
assert.ok(
    Math.abs(
        system.totalInjectedIodineMassMg -
        system.totalDeliveredVolumeMl * medium.iodineMgPerMl
    ) < 1e-8,
    'pressure limiting must preserve iodine mass exactly'
);

system.stopInjection();
for (let step = 0; step < 72; step++) system.update(1 / 120);
const mixedJetMetrics = system.getMetrics();
assert.equal(
    mixedJetMetrics.activeParticleCount,
    0,
    'a pressure-limited Berenstein plume must leave no detached local fragments after 0.6 s'
);
assert.ok(
    mixedJetMetrics.lastResolvedJetSpeedMmPerSec <= 1200,
    'the resolved plume must start after near-field blood entrainment instead of using the ballistic nozzle speed'
);
assert.ok(
    mixedJetMetrics.lastJetMixingLengthMm < 30,
    `a 0.97 mm Berenstein outlet should mix on an outlet-scale distance (${mixedJetMetrics.lastJetMixingLengthMm} mm)`
);
assert.ok(
    mixedJetMetrics.maximumRetrogradeProgressMm < 40,
    `the Berenstein stream must not shoot through a long aortic segment (${mixedJetMetrics.maximumRetrogradeProgressMm} mm)`
);
assert.ok(
    Math.abs(mixedJetMetrics.relativeBalanceError) < 1e-8,
    'rapid jet entrainment must conserve iodine mass'
);
const sheathPreview = system.getInjectionPreview({
    source: CONTRAST_SOURCE_SHEATH,
    volumeMl: 30,
    rateMlPerSec: 29.1
});
assert.equal(sheathPreview.valid, true);
assert.equal(sheathPreview.actualRateMlPerSec, 29.1);
assert.ok(
    sheathPreview.maximumAchievableRateMlPerSec >
        catheterPreview.maximumAchievableRateMlPerSec,
    'the sheath rate control should expose its own larger hydraulic range'
);

const blockedCatheter = {
    type: 'berenstein',
    getInjectionPorts(out) {
        out.length = 0;
        return out;
    }
};
const blockedSystem = new HybridContrastSystem({
    centerlineSegments: segments,
    catheter: blockedCatheter,
    medium
});
const blockedBerensteinPreview = blockedSystem.getInjectionPreview({
    source: CONTRAST_SOURCE_CATHETER,
    volumeMl: 30,
    rateMlPerSec: 15
});
assert.equal(blockedBerensteinPreview.valid, false);
assert.ok(
    blockedBerensteinPreview.maximumAchievableRateMlPerSec > 0,
    'the selected catheter should expose its nominal slider maximum before its ports leave the sheath'
);
blockedCatheter.type = 'pigtail';
const blockedPigtailPreview = blockedSystem.getInjectionPreview({
    source: CONTRAST_SOURCE_CATHETER,
    volumeMl: 30,
    rateMlPerSec: 15
});
assert.ok(
    blockedPigtailPreview.maximumAchievableRateMlPerSec >
        blockedBerensteinPreview.maximumAchievableRateMlPerSec,
    'changing the selected catheter must change the inactive-source slider maximum too'
);

const editedProfiles = mergeDeviceHydraulicProfiles({
    berenstein: { innerDiameterMm: 1.1 }
});
system.setInjectionHydraulicParameters({
    maximumPressurePsi: 600,
    viscosityPaS: 0.008,
    deviceProfiles: editedProfiles
});
const editedPreview = system.getInjectionPreview({
    source: CONTRAST_SOURCE_CATHETER,
    volumeMl: 30,
    rateMlPerSec: 29.1
});
assert.equal(editedPreview.injectorPressureLimitPsi, 600);
assert.equal(editedPreview.viscosityPaS, 0.008);
assert.equal(editedPreview.deviceProfile.innerDiameterMm, 1.1);

console.log('injection hydraulics tests passed', {
    sheath29_1: sheathFast.actualRateMlPerSec,
    berenstein29_1: berensteinFast.actualRateMlPerSec,
    pigtail29_1: pigtailFast.actualRateMlPerSec,
    berensteinRequiredPsi: berensteinFast.requiredPressurePsi
});
