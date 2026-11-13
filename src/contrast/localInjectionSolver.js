import * as THREE from 'three';
import { createContactResult } from '../physics/collision/vesselContactField.js';
import { arterialWaveform } from './flowNetwork.js';

const MM3_PER_ML = 1000;
const DEFAULT_PARTICLE_CAPACITY = 16000;
const DEFAULT_PARCEL_VOLUME_ML = 0.0025;
const MIN_LOCAL_AGE_SECONDS = 0.04;
const MIXED_HANDOFF_AGE_SECONDS = 0.12;
const SOURCE_NETWORK_HANDOFF_AGE_SECONDS = 0.055;
const TARGETED_JET_MIXING_AGE_SECONDS = 0.08;
const MAX_LOCAL_AGE_SECONDS = 1.5;
const MAX_RETROGRADE_LOCAL_AGE_SECONDS = 0.45;
// The sub-millimetre nozzle core is below the spatial resolution of the
// vascular solver. Its true speed is retained for pressure/momentum
// diagnostics, while the resolved plume starts after near-field entrainment
// with blood. This prevents a numerically ballistic streak through a large
// artery without pretending that the physical nozzle velocity is lower.
const MAX_RESOLVED_JET_SPEED_MM_PER_S = 1200;

export const DEFAULT_CONTRAST_MEDIUM = Object.freeze({
    name: 'Generic iohexol 300',
    iodineMgPerMl: 300,
    densityKgPerM3: 1349,
    viscosityPaS: 0.0063
});

function clamp(value, min, max) {
    return Math.max(min, Math.min(max, value));
}

function chainChild(edges, edge) {
    const child = edge.childEdgeIndices.length === 1 ? edges[edge.childEdgeIndices[0]] : null;
    return child && !child.transportExcluded ? child : null;
}

function projectedAxial(edge, x, y, z) {
    return clamp((x - edge.start.x) * edge.axis.x + (y - edge.start.y) * edge.axis.y +
        (z - edge.start.z) * edge.axis.z, 0, edge.length);
}

function projectedDistanceSquared(edge, axial, x, y, z) {
    return (x - edge.start.x - edge.axis.x * axial) ** 2 +
        (y - edge.start.y - edge.axis.y * axial) ** 2 +
        (z - edge.start.z - edge.axis.z * axial) ** 2;
}

function insideTransportLumen(edge, axial, distanceSquared) {
    const radius = edge.radiusStart + (edge.radiusEnd - edge.radiusStart) * axial / edge.length;
    // Match source selection's allowance for the sampled medial axis.
    // Explicitly excluded/occluded paths remain barriers.
    return !edge.transportExcluded && (!edge.lumenMappingExcluded || edge.intraluminalAorticConnector) &&
        distanceSquared <= (radius * 1.05) ** 2;
}

function portArea(port) {
    if (port.areaMm2 > 0) return port.areaMm2;
    const radius = Math.max(0.05, port.radiusMm || 0.25);
    return Math.PI * radius * radius;
}

function normalizeDirection(direction, fallback = null) {
    const x = direction?.x ?? direction?.[0] ?? fallback?.x ?? 0;
    const y = direction?.y ?? direction?.[1] ?? fallback?.y ?? 1;
    const z = direction?.z ?? direction?.[2] ?? fallback?.z ?? 0;
    const length = Math.hypot(x, y, z) || 1;
    return { x: x / length, y: y / length, z: z / length };
}

export class LocalContrastInjectionSolver {
    constructor({
        flowNetwork,
        contactField = null,
        capacity = DEFAULT_PARTICLE_CAPACITY,
        parcelVolumeMl = DEFAULT_PARCEL_VOLUME_ML,
        randomSeed = 0x5eed1234
    } = {}) {
        if (!flowNetwork) throw new TypeError('A flow network is required');
        this.flowNetwork = flowNetwork;
        this.contactField = contactField;
        this.capacity = capacity;
        this.parcelVolumeMl = parcelVolumeMl;
        this.count = 0;
        this.positionX = new Float32Array(capacity);
        this.positionY = new Float32Array(capacity);
        this.positionZ = new Float32Array(capacity);
        this.outletPositionX = new Float32Array(capacity);
        this.outletPositionY = new Float32Array(capacity);
        this.outletPositionZ = new Float32Array(capacity);
        this.outletDirectionX = new Float32Array(capacity);
        this.outletDirectionY = new Float32Array(capacity);
        this.outletDirectionZ = new Float32Array(capacity);
        this.outletRadiusMm = new Float32Array(capacity);
        this.resolvedOutletRadiusMm = new Float32Array(capacity);
        this.initialTransportSpeedMmPerSec = new Float32Array(capacity);
        this.jetVelocityX = new Float32Array(capacity);
        this.jetVelocityY = new Float32Array(capacity);
        this.jetVelocityZ = new Float32Array(capacity);
        this.jetDecayPerSecond = new Float32Array(capacity);
        // Keep mass in double precision. Pressure-limited rates are generally
        // irrational values, so Float32 parcel masses accumulated a visible
        // bookkeeping error over long injections even though transport itself
        // remained conservative.
        this.iodineMassMg = new Float64Array(capacity);
        this.initialIodineMassMg = new Float64Array(capacity);
        this.ageSeconds = new Float32Array(capacity);
        this.birthDelaySeconds = new Float32Array(capacity);
        this.travelDistanceMm = new Float32Array(capacity);
        this.handoffDistanceMm = new Float32Array(capacity);
        this.preferredHandoffEdgeIndex = new Int32Array(capacity);
        this.preferredHandoffCellIndex = new Int32Array(capacity);
        this.sourceEdgeIndex = new Int32Array(capacity);
        this.sourceCellIndex = new Int32Array(capacity);
        this.handoffEdgeIndex = new Int32Array(capacity).fill(-1);
        this.retrogradeProgressMm = new Float32Array(capacity);
        this.preferredHandoffEdgeIndex.fill(-1);
        this.preferredHandoffCellIndex.fill(-1);
        this.sourceEdgeIndex.fill(-1);
        this.sourceCellIndex.fill(-1);
        this.totalEmittedIodineMassMg = 0;
        this.totalHandedOffIodineMassMg = 0;
        this.directHandoffIodineMassMg = 0;
        this.retrogradeTargetedIodineMassMg = 0;
        this.retrogradeHandoffIodineMassMg = 0;
        this.retrogradeEntrainedIodineMassMg = 0;
        this.sourceHandoffIodineMassMg = 0;
        this.maximumRetrogradeProgressMm = 0;
        this.maxWallPenetrationMm = 0;
        this.lastSourceEdgeIndex = -1;
        this.lastSourceCellIndex = -1;
        this.lastSourceRadiusMm = 0;
        this.lastSourceDistanceMm = Infinity;
        this.lastSourceSelectionMode = '';
        this.lastSourceT = 0;
        this.lastSourcePortPosition = { x: 0, y: 0, z: 0 };
        this.lastSourcePortDirection = { x: 0, y: 0, z: 0 };
        this.lastDirectionAgainstFlow = 0;
        this.lastRetrogradeAlignment = 0;
        this.lastRetrogradeMomentumFluxRatio = 0;
        this.lastPhysicalJetSpeedMmPerSec = 0;
        this.lastResolvedJetSpeedMmPerSec = 0;
        this.lastEquivalentOutletDiameterMm = 0;
        this.lastJetMixingLengthMm = 0;
        this.lastTargetedCoreFraction = 0;
        this.sourceMappingChangeCount = 0;
        this.sourceMassByEdge = new Map();
        this._previousSourceEdgeIndex = -1;
        this._updateCounter = 0;
        this._randomState = randomSeed >>> 0 || 1;
        this._velocityScratch = {};
        this._locationScratch = {};
        this._handoffLocationScratch = {};
        this._sourceFlowTime = NaN;
        this._sourceFlowHeartRate = NaN;
        this._sourceFlowWaveform = 1;
        this._contactScratch = createContactResult();
        this._pointScratch = new THREE.Vector3();
    }

    _randomSigned() {
        let state = this._randomState;
        state ^= state << 13;
        state ^= state >>> 17;
        state ^= state << 5;
        this._randomState = state >>> 0;
        return (this._randomState / 0xffffffff) * 2 - 1;
    }

    emit({
        ports,
        volumeMl,
        rateMlPerSec,
        emissionWindowSeconds = 0,
        medium = DEFAULT_CONTRAST_MEDIUM,
        allowPressureDrivenRetrograde = true,
        portFlowRatesMlPerSec = null,
        portJetVelocitiesMPerSec = null,
        portInjectionLocations = null
    }) {
        if (!(volumeMl > 0) || !Array.isArray(ports) || !ports.length) {
            return { emittedVolumeMl: 0, emittedIodineMassMg: 0, particleCount: 0 };
        }
        const activePorts = ports.filter(port => port?.position && port.valid !== false);
        if (!activePorts.length) {
            return { emittedVolumeMl: 0, emittedIodineMassMg: 0, particleCount: 0 };
        }
        const hasResolvedPortFlows =
            Array.isArray(portFlowRatesMlPerSec) &&
            portFlowRatesMlPerSec.length === activePorts.length &&
            portFlowRatesMlPerSec.every(rate =>
                Number.isFinite(rate) && rate >= 0
            );
        let totalWeight = hasResolvedPortFlows
            ? portFlowRatesMlPerSec.reduce((sum, rate) => sum + rate, 0)
            : activePorts.reduce(
                (sum, port) =>
                    sum + Math.max(0, port.weight ?? portArea(port)),
                0
            );
        if (!(totalWeight > 0)) totalWeight = activePorts.length;
        const totalMassMg = volumeMl * medium.iodineMgPerMl;
        let emittedParticles = 0;
        let emittedMassMg = 0;

        for (let portIndex = 0; portIndex < activePorts.length; portIndex++) {
            const port = activePorts[portIndex];
            const weight = hasResolvedPortFlows
                ? portFlowRatesMlPerSec[portIndex] / totalWeight
                : Math.max(0, port.weight ?? portArea(port)) / totalWeight;
            const portMassMg = totalMassMg * weight;
            const portRate = hasResolvedPortFlows
                ? portFlowRatesMlPerSec[portIndex]
                : Math.max(0, rateMlPerSec) * weight;
            const parcelVolumeMl = this.parcelVolumeMl;
            const particleCount = Math.max(
                1,
                Math.ceil(volumeMl * weight / Math.max(1e-6, parcelVolumeMl))
            );
            const massPerParticle = portMassMg / particleCount;
            const direction = normalizeDirection(port.direction);
            const areaMm2 = Math.max(0.01, portArea(port));
            const hydraulicJetSpeedMPerSec =
                Array.isArray(portJetVelocitiesMPerSec) &&
                Number.isFinite(portJetVelocitiesMPerSec[portIndex]) &&
                portJetVelocitiesMPerSec[portIndex] >= 0
                    ? portJetVelocitiesMPerSec[portIndex]
                    : null;
            const idealJetSpeed = hydraulicJetSpeedMPerSec == null
                ? portRate * MM3_PER_ML / areaMm2
                : hydraulicJetSpeedMPerSec * 1000;
            const prescribedLocation =
                Array.isArray(portInjectionLocations) &&
                portInjectionLocations[portIndex]?.edgeIndex >= 0
                    ? portInjectionLocations[portIndex]
                    : null;
            const location = prescribedLocation ||
                this.flowNetwork.findInjectionLocation(
                    port.position,
                    port.direction,
                    this._locationScratch
                );
            const backgroundVelocity = this.flowNetwork.sampleVelocityCoordinates(
                port.position.x,
                port.position.y,
                port.position.z,
                this._velocityScratch
            );
            const vesselDiameter = Math.max(
                2,
                port.localVesselDiameterMm || (location.radius || 2) * 2
            );
            const momentumRatio = idealJetSpeed /
                Math.max(1, backgroundVelocity.meanSpeed ?? backgroundVelocity.speed);
            const equivalentOutletDiameterMm = Math.max(
                0.1,
                2 * Math.sqrt(areaMm2 / Math.PI)
            );
            // A free jet entrains ambient fluid over outlet diameters, not
            // vessel diameters. The previous vessel-diameter scaling allowed
            // a 1 mm catheter jet to retain momentum for 100-200 mm in the
            // aorta and left late, detached parcels after injection stopped.
            const entrainmentDiameters = clamp(
                6 + 1.8 * Math.sqrt(Math.max(0, momentumRatio)),
                6,
                22
            );
            let decayDistance = clamp(
                equivalentOutletDiameterMm * entrainmentDiameters,
                Math.max(3, equivalentOutletDiameterMm * 4),
                Math.max(5, vesselDiameter * 1.5)
            );
            let handoffDistance = Math.max(4, decayDistance * 0.9);
            const localMixingHandoffDistance = handoffDistance;
            const effectiveJetSpeed = Math.min(
                MAX_RESOLVED_JET_SPEED_MM_PER_S,
                idealJetSpeed
            );
            this.lastPhysicalJetSpeedMmPerSec = idealJetSpeed;
            this.lastResolvedJetSpeedMmPerSec = effectiveJetSpeed;
            this.lastEquivalentOutletDiameterMm =
                equivalentOutletDiameterMm;
            this.lastJetMixingLengthMm = decayDistance;
            let preferredHandoffEdgeIndex = -1;
            let preferredHandoffCellIndex = -1;
            let targetedCoreFraction = 0;
            let sourceEdgeIndex = -1;
            let sourceCellIndex = -1;
            if (location.edgeIndex >= 0) {
                const sourceEdge = this.flowNetwork.edges[location.edgeIndex];
                sourceEdgeIndex = location.edgeIndex;
                sourceCellIndex = location.cellIndex;

                // At the finite volume shared by a parent and its children, a
                // nearest-centreline query can select either child even though
                // the catheter port is still physically inside the common
                // junction lumen. Anchor such an injection at the parent
                // terminal cell so physiological outflow can divide the bolus
                // between every accepting child.
                const junction = this.flowNetwork.nodes.get(
                    sourceEdge.startNodeId
                );
                const parent = sourceEdge.parentEdgeIndex >= 0
                    ? this.flowNetwork.edges[sourceEdge.parentEdgeIndex]
                    : null;
                if (
                    !prescribedLocation &&
                    parent &&
                    junction?.childEdgeIndices.length >= 2 &&
                    location.t * sourceEdge.length <=
                        Math.max(
                            parent.radiusEnd,
                            sourceEdge.radiusStart
                        ) * 1.15
                ) {
                    sourceEdgeIndex = parent.index;
                    sourceCellIndex = parent.cellCount - 1;
                }
                this.lastSourceEdgeIndex = sourceEdgeIndex;
                this.lastSourceCellIndex = sourceCellIndex;
                this.lastSourceRadiusMm =
                    location.radius || 0;
                this.lastSourceDistanceMm =
                    location.distance;
                this.lastSourceSelectionMode =
                    location.selectionMode || 'nearest-centerline';
                this.lastSourceT = location.t || 0;
                this.lastSourcePortPosition.x = port.position.x;
                this.lastSourcePortPosition.y = port.position.y;
                this.lastSourcePortPosition.z = port.position.z;
                this.lastSourcePortDirection.x = direction.x;
                this.lastSourcePortDirection.y = direction.y;
                this.lastSourcePortDirection.z = direction.z;
                if (
                    this._previousSourceEdgeIndex >= 0 &&
                    this._previousSourceEdgeIndex !== sourceEdgeIndex
                ) {
                    this.sourceMappingChangeCount++;
                }
                this._previousSourceEdgeIndex = sourceEdgeIndex;
                this.sourceMassByEdge.set(
                    sourceEdgeIndex,
                    (this.sourceMassByEdge.get(sourceEdgeIndex) || 0) +
                        portMassMg
                );
            }
            const directionAgainstFlow =
                direction.x * location.tangentX +
                direction.y * location.tangentY +
                direction.z * location.tangentZ;
            const sourceFlowMm3PerS = Math.max(
                1,
                this.flowNetwork.edges[location.edgeIndex]
                    ?.meanFlowMm3PerS || 1
            );
            const retrogradeDriveRatio =
                portRate * MM3_PER_ML / sourceFlowMm3PerS;
            const retrogradeAlignment = clamp(
                -directionAgainstFlow,
                0,
                1
            );
            const bloodSpeed = Math.max(
                1,
                backgroundVelocity.meanSpeed ?? backgroundVelocity.speed
            );
            const effectiveRetrogradeJetSpeed =
                idealJetSpeed * retrogradeAlignment;
            const retrogradeMomentumFluxRatio =
                retrogradeDriveRatio * effectiveRetrogradeJetSpeed /
                bloodSpeed;
            this.lastDirectionAgainstFlow = directionAgainstFlow;
            this.lastRetrogradeAlignment = retrogradeAlignment;
            this.lastRetrogradeMomentumFluxRatio =
                retrogradeMomentumFluxRatio;
            if (
                allowPressureDrivenRetrograde &&
                retrogradeAlignment > 0
            ) {
                const bloodToJetRatio = clamp(
                    bloodSpeed /
                        Math.max(1, effectiveRetrogradeJetSpeed),
                    0,
                    0.98
                );
                const netPenetrationFraction =
                    1 - bloodToJetRatio +
                    bloodToJetRatio * Math.log(Math.max(1e-6, bloodToJetRatio));
                const estimatedNetPenetration =
                    decayDistance * netPenetrationFraction;
                const mixingTarget = this.flowNetwork.findUpstreamMixingJunction(
                    location,
                    {
                        minimumParentRadiusMm:
                            Math.max(0.35, vesselDiameter * 0.425),
                        minimumChildRadiusMm:
                            Math.max(0.35, vesselDiameter * 0.275),
                        maximumDistanceMm:
                            estimatedNetPenetration / 1.05,
                        preferFarthest: true
                    }
                );
                const pressureDominates =
                    retrogradeMomentumFluxRatio > 1.1;
                if (
                    mixingTarget &&
                    pressureDominates &&
                    estimatedNetPenetration >= mixingTarget.distanceMm * 1.05
                ) {
                    // Only a junction inside the entrainment-limited physical
                    // penetration length may receive a coherent local jet.
                    // Sustained flow reversal is handled conservatively by
                    // PressureDrivenRetrogradeColumn; individual parcels are
                    // never granted extra lifetime merely to reach a target.
                    const targetEdge = this.flowNetwork.edges[
                        mixingTarget.edgeIndex
                    ];
                    const targetCellLengthMm = targetEdge
                        ? targetEdge.length / targetEdge.cellCount
                        : 0;
                    handoffDistance = Math.max(
                        handoffDistance,
                        Math.max(
                            12,
                            // The particle must physically reach the target
                            // control volume before its iodine is deposited
                            // there. The old six-diameter shortcut removed the
                            // local plume tens of millimetres before the
                            // junction and made the bolus appear to teleport.
                            // One target-cell length is the finite-volume
                            // spatial tolerance, independent of anatomy.
                            mixingTarget.distanceMm - targetCellLengthMm
                        )
                    );
                    preferredHandoffEdgeIndex = mixingTarget.edgeIndex;
                    preferredHandoffCellIndex = mixingTarget.cellIndex;
                    // A turbulent jet is an entraining shear layer, not a
                    // solid plug. Only the coherent core can reach a nearby
                    // upstream junction; the remaining contrast mixes into
                    // the source vessel continuously. Distributing this
                    // fraction across every emission step avoids two
                    // temporally separated boluses.
                    targetedCoreFraction = clamp(
                        0.85 * Math.exp(
                            -mixingTarget.distanceMm /
                                Math.max(1e-6, decayDistance)
                        ),
                        0.08,
                        0.55
                    );
                }
            }
            this.lastJetMixingLengthMm = decayDistance;
            this.lastTargetedCoreFraction = targetedCoreFraction;
            const decay = effectiveJetSpeed / decayDistance;

            if (!allowPressureDrivenRetrograde && sourceEdgeIndex >= 0 &&
                this._hasBidirectionalSourceOutflow(sourceEdgeIndex, sourceCellIndex)) {
                // Both faces carry fluid away from this source: no blood is
                // entering its finite mixing volume. All newly supplied fluid
                // is therefore the injection itself. Its iodine must enter
                // with that volume source, not additionally occupy a delayed
                // local core on top of an already contrast-filled lumen.
                if (this.flowNetwork.queueIodineSource(sourceEdgeIndex, sourceCellIndex, portMassMg)) {
                    this.totalHandedOffIodineMassMg += portMassMg;
                    this.directHandoffIodineMassMg += portMassMg;
                    this.sourceHandoffIodineMassMg += portMassMg;
                    emittedMassMg += portMassMg;
                    continue;
                }
            }

            for (let particleIndex = 0; particleIndex < particleCount; particleIndex++) {
                if (this.count >= this.capacity) {
                    const handedOff = sourceEdgeIndex >= 0
                        ? this._depositOverflowIodine(sourceEdgeIndex, sourceCellIndex, location, massPerParticle)
                        : this.flowNetwork.depositIodineAtPoint(
                            port.position,
                            massPerParticle
                        );
                    if (handedOff) {
                        this.totalHandedOffIodineMassMg += massPerParticle;
                        this.directHandoffIodineMassMg += massPerParticle;
                        if (sourceEdgeIndex >= 0) {
                            this.sourceHandoffIodineMassMg +=
                                massPerParticle;
                        }
                        emittedMassMg += massPerParticle;
                    }
                    continue;
                }
                const index = this.count++;
                const radialJitter = Math.max(0.02, Math.sqrt(areaMm2 / Math.PI) * 0.18);
                const jitterX = this._randomSigned() * radialJitter;
                const jitterY = this._randomSigned() * radialJitter;
                const jitterZ = this._randomSigned() * radialJitter;
                this.positionX[index] = port.position.x + direction.x * 0.15 + jitterX;
                this.positionY[index] = port.position.y + direction.y * 0.15 + jitterY;
                this.positionZ[index] = port.position.z + direction.z * 0.15 + jitterZ;
                this.outletPositionX[index] = port.position.x;
                this.outletPositionY[index] = port.position.y;
                this.outletPositionZ[index] = port.position.z;
                this.outletDirectionX[index] = direction.x;
                this.outletDirectionY[index] = direction.y;
                this.outletDirectionZ[index] = direction.z;
                this.outletRadiusMm[index] = Math.sqrt(areaMm2 / Math.PI);
                // The resolved near-field jet moves more slowly than the
                // sub-millimetre nozzle core. Preserve its injected volume
                // flux by increasing its effective area by the same ratio.
                this.resolvedOutletRadiusMm[index] = this.outletRadiusMm[index] *
                    Math.sqrt(idealJetSpeed / Math.max(1e-9, effectiveJetSpeed));
                const angularSpread = clamp(0.035 + idealJetSpeed / 50000, 0.035, 0.16);
                let jetX = direction.x +
                    this._randomSigned() * angularSpread;
                let jetY = direction.y +
                    this._randomSigned() * angularSpread;
                let jetZ = direction.z +
                    this._randomSigned() * angularSpread;
                const jetLength = Math.hypot(jetX, jetY, jetZ) || 1;
                jetX /= jetLength;
                jetY /= jetLength;
                jetZ /= jetLength;
                this.initialTransportSpeedMmPerSec[index] = Math.max(1,
                    effectiveJetSpeed + backgroundVelocity.velocityX * jetX +
                    backgroundVelocity.velocityY * jetY + backgroundVelocity.velocityZ * jetZ);
                this.jetVelocityX[index] = jetX * effectiveJetSpeed;
                this.jetVelocityY[index] = jetY * effectiveJetSpeed;
                this.jetVelocityZ[index] = jetZ * effectiveJetSpeed;
                this.jetDecayPerSecond[index] = Math.max(8, decay);
                this.iodineMassMg[index] = massPerParticle;
                this.initialIodineMassMg[index] = massPerParticle;
                this.ageSeconds[index] = 0;
                this.birthDelaySeconds[index] = Math.max(
                    0,
                    emissionWindowSeconds
                ) * (particleIndex + 0.5) / particleCount;
                this.travelDistanceMm[index] = 0;
                const particleTargetsJunction =
                    preferredHandoffEdgeIndex >= 0 &&
                    ((particleIndex * 0.61803398875) % 1) <
                        targetedCoreFraction;
                this.handoffDistanceMm[index] = particleTargetsJunction
                    ? handoffDistance
                    : localMixingHandoffDistance;
                this.preferredHandoffEdgeIndex[index] =
                    particleTargetsJunction
                        ? preferredHandoffEdgeIndex
                        : -1;
                this.preferredHandoffCellIndex[index] =
                    particleTargetsJunction
                        ? preferredHandoffCellIndex
                        : -1;
                this.sourceEdgeIndex[index] = sourceEdgeIndex;
                this.sourceCellIndex[index] = sourceCellIndex;
                this.handoffEdgeIndex[index] = sourceEdgeIndex;
                this.retrogradeProgressMm[index] = 0;
                if (!allowPressureDrivenRetrograde && sourceEdgeIndex >= 0) {
                    // A narrow lumen may not contain the Q/v-resolved core.
                    // Mix the excess iodine into its connected blood volume
                    // instead of compressing it above stock concentration or
                    // enlarging the rendered plume beyond the vessel wall.
                    const sourceEdge = this.flowNetwork.edges[sourceEdgeIndex];
                    const maximumSigma = .4 * Math.max(sourceEdge.radiusStart, sourceEdge.radiusEnd);
                    const resolvedSigma = this.resolvedOutletRadiusMm[index] / Math.SQRT2;
                    const coherentFraction = Math.min(1, maximumSigma * maximumSigma /
                        Math.max(1e-12, resolvedSigma * resolvedSigma));
                    const excess = massPerParticle * (1 - coherentFraction);
                    if (excess > 0) {
                        // All source/location metadata is initialized before
                        // using the same conservative hydraulic handoff as
                        // subsequent entrainment, including reflux splitting.
                        const deposited = this._depositSourceEntrainment(index, excess);
                        this.iodineMassMg[index] -= deposited;
                        this.initialIodineMassMg[index] -= deposited;
                        this.totalHandedOffIodineMassMg += deposited;
                        this.sourceHandoffIodineMassMg += deposited;
                    }
                }
                if (particleTargetsJunction) {
                    this.retrogradeTargetedIodineMassMg += massPerParticle;
                }
                emittedParticles++;
                emittedMassMg += massPerParticle;
            }
        }
        this.totalEmittedIodineMassMg += emittedMassMg;
        return {
            emittedVolumeMl: emittedMassMg / Math.max(1e-9, medium.iodineMgPerMl),
            emittedIodineMassMg: emittedMassMg,
            particleCount: emittedParticles
        };
    }

    update(dt) {
        if (!(dt > 0) || this.count === 0) return;
        this._updateCounter++;
        let index = 0;
        while (index < this.count) {
            const particleDt = Math.max(
                0,
                dt - this.birthDelaySeconds[index]
            );
            this.birthDelaySeconds[index] = 0;
            if (!(particleDt > 0)) {
                index++;
                continue;
            }
            const pressureDrivenRetrograde =
                this.preferredHandoffEdgeIndex[index] >= 0;
            const oldX = this.positionX[index];
            const oldY = this.positionY[index];
            const oldZ = this.positionZ[index];
            const velocity = this.flowNetwork.sampleVelocityCoordinates(
                oldX,
                oldY,
                oldZ,
                this._velocityScratch
            );
            if (pressureDrivenRetrograde) {
                const jetSpeed = Math.hypot(
                    this.jetVelocityX[index],
                    this.jetVelocityY[index],
                    this.jetVelocityZ[index]
                );
                if (jetSpeed > 1) {
                    const guideBlend = clamp(
                        1 - Math.exp(-18 * particleDt),
                        0,
                        0.35
                    );
                    let guidedX =
                        this.jetVelocityX[index] / jetSpeed * (1 - guideBlend) -
                        velocity.tangentX * guideBlend;
                    let guidedY =
                        this.jetVelocityY[index] / jetSpeed * (1 - guideBlend) -
                        velocity.tangentY * guideBlend;
                    let guidedZ =
                        this.jetVelocityZ[index] / jetSpeed * (1 - guideBlend) -
                        velocity.tangentZ * guideBlend;
                    const guidedLength = Math.hypot(guidedX, guidedY, guidedZ) || 1;
                    guidedX /= guidedLength;
                    guidedY /= guidedLength;
                    guidedZ /= guidedLength;
                    this.jetVelocityX[index] = guidedX * jetSpeed;
                    this.jetVelocityY[index] = guidedY * jetSpeed;
                    this.jetVelocityZ[index] = guidedZ * jetSpeed;
                }
            }
            const decay = Math.exp(-this.jetDecayPerSecond[index] * particleDt);
            const integratedJetScale = this.jetDecayPerSecond[index] > 1e-6
                ? (1 - decay) / this.jetDecayPerSecond[index]
                : particleDt;
            const dispersionMm2PerS = 2.5 +
                Math.min(28, Math.hypot(
                    this.jetVelocityX[index],
                    this.jetVelocityY[index],
                    this.jetVelocityZ[index]
                ) * 0.008);
            const randomScale = Math.sqrt(2 * dispersionMm2PerS * particleDt / 3);
            let nextX = oldX +
                velocity.velocityX * particleDt +
                this.jetVelocityX[index] * integratedJetScale +
                this._randomSigned() * randomScale;
            let nextY = oldY +
                velocity.velocityY * particleDt +
                this.jetVelocityY[index] * integratedJetScale +
                this._randomSigned() * randomScale;
            let nextZ = oldZ +
                velocity.velocityZ * particleDt +
                this.jetVelocityZ[index] * integratedJetScale +
                this._randomSigned() * randomScale;

            this.jetVelocityX[index] *= decay;
            this.jetVelocityY[index] *= decay;
            this.jetVelocityZ[index] *= decay;

            const nextLocation = this.flowNetwork.findNearestLocationCoordinates(
                nextX,
                nextY,
                nextZ,
                this._locationScratch
            );
            const approximateRadius = Math.max(0.35, nextLocation.radius || 0.35);
            const approximateBoundaryFactor = pressureDrivenRetrograde ? 0.78 : 0.94;
            const outsideApproximateLumen =
                nextLocation.distance > approximateRadius * approximateBoundaryFactor;
            if (outsideApproximateLumen && nextLocation.edgeIndex >= 0) {
                const radialX = nextX - nextLocation.centerX;
                const radialY = nextY - nextLocation.centerY;
                const radialZ = nextZ - nextLocation.centerZ;
                const radialLength = Math.hypot(radialX, radialY, radialZ) || 1;
                const targetRadius = approximateRadius * (
                    pressureDrivenRetrograde ? 0.72 : 0.9
                );
                nextX = nextLocation.centerX + radialX / radialLength * targetRadius;
                nextY = nextLocation.centerY + radialY / radialLength * targetRadius;
                nextZ = nextLocation.centerZ + radialZ / radialLength * targetRadius;
                const outwardX = radialX / radialLength;
                const outwardY = radialY / radialLength;
                const outwardZ = radialZ / radialLength;
                const outwardVelocity =
                    this.jetVelocityX[index] * outwardX +
                    this.jetVelocityY[index] * outwardY +
                    this.jetVelocityZ[index] * outwardZ;
                if (outwardVelocity > 0) {
                    this.jetVelocityX[index] -= outwardX * outwardVelocity * 1.15;
                    this.jetVelocityY[index] -= outwardY * outwardVelocity * 1.15;
                    this.jetVelocityZ[index] -= outwardZ * outwardVelocity * 1.15;
                }
                const approximateRetention = pressureDrivenRetrograde ? 0.96 : 0.55;
                this.jetVelocityX[index] *= approximateRetention;
                this.jetVelocityY[index] *= approximateRetention;
                this.jetVelocityZ[index] *= approximateRetention;
            }

            const nearWall = nextLocation.distance > approximateRadius * 0.72;
            const validateExactWall = pressureDrivenRetrograde
                ? (
                    outsideApproximateLumen &&
                    ((index + this._updateCounter) & 15) === 0
                )
                : (
                    outsideApproximateLumen ||
                    (nearWall && ((index + this._updateCounter) & 3) === 0)
                );
            if ((validateExactWall || this.graftSurface?.bounds.containsPoint({x:nextX,y:nextY,z:nextZ})) && this.contactField?.querySphere) {
                this._pointScratch.set(nextX, nextY, nextZ);
                const previousPosition={x:oldX,y:oldY,z:oldZ};
                const queryContact=this.contactField.querySphereFrom
                    ?(position,radius,out)=>this.contactField.querySphereFrom(position,radius,out,previousPosition)
                    :(position,radius,out)=>this.contactField.querySphere(position,radius,out);
                const contact = queryContact(
                    this._pointScratch,
                    0.05,
                    this._contactScratch
                );
                if (contact.violation) {
                    const inwardDot =
                        this.jetVelocityX[index] * contact.inward.x +
                        this.jetVelocityY[index] * contact.inward.y +
                        this.jetVelocityZ[index] * contact.inward.z;
                    if (inwardDot < 0) {
                        this.jetVelocityX[index] -= contact.inward.x * inwardDot * 1.12;
                        this.jetVelocityY[index] -= contact.inward.y * inwardDot * 1.12;
                        this.jetVelocityZ[index] -= contact.inward.z * inwardDot * 1.12;
                    }
                    const exactRetention = pressureDrivenRetrograde ? 0.94 : 0.42;
                    this.jetVelocityX[index] *= exactRetention;
                    this.jetVelocityY[index] *= exactRetention;
                    this.jetVelocityZ[index] *= exactRetention;
                    let settledContact = contact;
                    for (let pass = 0; pass < 2 && settledContact.violation; pass++) {
                        nextX = settledContact.target.x + settledContact.inward.x * 0.01;
                        nextY = settledContact.target.y + settledContact.inward.y * 0.01;
                        nextZ = settledContact.target.z + settledContact.inward.z * 0.01;
                        this._pointScratch.set(nextX, nextY, nextZ);
                        settledContact = queryContact(
                            this._pointScratch,
                            0.05,
                            this._contactScratch
                        );
                    }
                    if (settledContact.violation) {
                        // At closely spaced branch surfaces successive SDF
                        // projections can alternate between two triangles.
                        // The centerline sample is a conservative, known-lumen
                        // fallback for the small number of unresolved jets.
                        nextX = nextLocation.centerX;
                        nextY = nextLocation.centerY;
                        nextZ = nextLocation.centerZ;
                        this._pointScratch.set(nextX, nextY, nextZ);
                        settledContact = queryContact(
                            this._pointScratch,
                            0.05,
                            this._contactScratch
                        );
                        if (settledContact.violation) {
                            this.maxWallPenetrationMm = Math.max(
                                this.maxWallPenetrationMm,
                                settledContact.penetration
                            );
                        }
                    }
                }
            }

            this.positionX[index] = nextX;
            this.positionY[index] = nextY;
            this.positionZ[index] = nextZ;
            this.ageSeconds[index] += particleDt;
            this.travelDistanceMm[index] += Math.hypot(
                nextX - oldX,
                nextY - oldY,
                nextZ - oldZ
            );
            if (this.preferredHandoffEdgeIndex[index] >= 0) {
                const retrogradeStep = -(
                    (nextX - oldX) * velocity.tangentX +
                    (nextY - oldY) * velocity.tangentY +
                    (nextZ - oldZ) * velocity.tangentZ
                );
                this.retrogradeProgressMm[index] = Math.max(
                    0,
                    this.retrogradeProgressMm[index] + retrogradeStep
                );
                this.maximumRetrogradeProgressMm = Math.max(
                    this.maximumRetrogradeProgressMm,
                    this.retrogradeProgressMm[index]
                );
            }

            const jetSpeed = Math.hypot(
                this.jetVelocityX[index],
                this.jetVelocityY[index],
                this.jetVelocityZ[index]
            );
            if (this.sourceEdgeIndex[index] >= 0 && !pressureDrivenRetrograde) {
                // Entrainment transfers iodine continuously from the resolved
                // outlet jet into blood along its finite travelled path. A
                // fixed-age return of the entire parcel to its source cell
                // produced a stationary full-lumen plug at the catheter tip.
                const distanceProgress = (
                    this.travelDistanceMm[index] /
                        Math.max(1, this.handoffDistanceMm[index]) - 0.2
                ) / 1.05;
                // Once a weak jet has lost momentum, its residual core mixes
                // locally too; it must not wait indefinitely for more travel.
                const ageProgress = (this.ageSeconds[index] - 0.025) / 0.095;
                const progress = clamp(Math.max(distanceProgress, ageProgress), 0, 1);
                const bloodAlongJet = (
                    velocity.velocityX * this.jetVelocityX[index] +
                    velocity.velocityY * this.jetVelocityY[index] +
                    velocity.velocityZ * this.jetVelocityZ[index]
                ) / Math.max(1e-6, jetSpeed);
                // Opposing blood compresses the spacing of emitted parcels
                // as the jet slows, then turns them back downstream. Preserve
                // volume continuity (Q = v A): the coherent share decreases
                // with its net forward speed, while the rest joins the blood.
                // Otherwise packets pile up at the turning point at several
                // times the concentration of the injected preparation.
                const counterflowMixedFraction = bloodAlongJet < 0 ? 1 - clamp(
                    (jetSpeed + bloodAlongJet) / Math.max(1, this.initialTransportSpeedMmPerSec[index]), 0, 1
                ) : 0;
                const mixedFraction = Math.max(progress * progress * (3 - 2 * progress), counterflowMixedFraction);
                const retainedMass = this.initialIodineMassMg[index] * (1 - mixedFraction);
                const release = Math.max(0, this.iodineMassMg[index] - retainedMass);
                if (release > 0) {
                    const deposited = this._depositSourceEntrainment(index, release);
                    this.iodineMassMg[index] -= deposited;
                    this.totalHandedOffIodineMassMg += deposited;
                    this.sourceHandoffIodineMassMg += deposited;
                }
                if (!(this.iodineMassMg[index] > 0)) {
                    this._removeParticle(index);
                    continue;
                }
                index++;
                continue;
            }
            const mixedWithBlood =
                this.sourceEdgeIndex[index] < 0 &&
                this.ageSeconds[index] >= MIXED_HANDOFF_AGE_SECONDS &&
                jetSpeed <= Math.max(20, velocity.speed * 0.2);
            const leftLocalDomain =
                this.sourceEdgeIndex[index] < 0 &&
                this.ageSeconds[index] >= MIN_LOCAL_AGE_SECONDS &&
                this.travelDistanceMm[index] >= this.handoffDistanceMm[index];
            const sourceReadyForNetwork =
                this.sourceEdgeIndex[index] >= 0 &&
                this.ageSeconds[index] >=
                    SOURCE_NETWORK_HANDOFF_AGE_SECONDS;
            const targetedJetMixed =
                this.preferredHandoffEdgeIndex[index] >= 0 &&
                this.ageSeconds[index] >=
                    TARGETED_JET_MIXING_AGE_SECONDS &&
                jetSpeed <= Math.max(80, velocity.speed * 0.75);
            const expired = this.ageSeconds[index] >= (
                pressureDrivenRetrograde
                    ? MAX_RETROGRADE_LOCAL_AGE_SECONDS
                    : MAX_LOCAL_AGE_SECONDS
            );
            if (
                sourceReadyForNetwork ||
                mixedWithBlood ||
                leftLocalDomain ||
                expired
            ) {
                const preferredEdgeIndex = this.preferredHandoffEdgeIndex[index];
                const reachedPreferredEdge =
                    preferredEdgeIndex >= 0 &&
                    (
                        nextLocation.edgeIndex === preferredEdgeIndex ||
                        this.retrogradeProgressMm[index] >=
                            this.handoffDistanceMm[index]
                    );
                if (
                    preferredEdgeIndex >= 0 &&
                    !reachedPreferredEdge &&
                    !targetedJetMixed &&
                    !expired
                ) {
                    index++;
                    continue;
                }
                let handedOff = this.flowNetwork.depositGraftSacAtCoordinates(nextX,nextY,nextZ,this.iodineMassMg[index]);
                if (handedOff) {
                    // Preserve the fabric side at local-to-network handoff.
                } else if (reachedPreferredEdge) {
                    handedOff = this.flowNetwork.depositIodine(
                        preferredEdgeIndex,
                        this.preferredHandoffCellIndex[index],
                        this.iodineMassMg[index]
                    );
                } else if (
                    preferredEdgeIndex < 0 &&
                    sourceReadyForNetwork
                ) {
                    handedOff = this.flowNetwork.depositIodine(
                        this.sourceEdgeIndex[index],
                        this.sourceCellIndex[index],
                        this.iodineMassMg[index]
                    );
                } else if (
                    preferredEdgeIndex >= 0 &&
                    (targetedJetMixed || expired)
                ) {
                    // An entrained jet joins the blood where its visible front
                    // actually mixed. Depositing it at the remote target (or
                    // back at the catheter tip) would create a second bolus
                    // and leave detached fragments drifting after injection.
                    handedOff = this.flowNetwork.depositIodineAtCoordinates(
                        nextX,
                        nextY,
                        nextZ,
                        this.iodineMassMg[index]
                    );
                } else {
                    handedOff = this.flowNetwork.depositIodineAtCoordinates(
                        nextX,
                        nextY,
                        nextZ,
                        this.iodineMassMg[index]
                    );
                }
                if (handedOff) {
                    this.totalHandedOffIodineMassMg += this.iodineMassMg[index];
                    if (reachedPreferredEdge) {
                        this.retrogradeHandoffIodineMassMg += this.iodineMassMg[index];
                    } else if (
                        preferredEdgeIndex >= 0 &&
                        (targetedJetMixed || expired)
                    ) {
                        this.retrogradeEntrainedIodineMassMg +=
                            this.iodineMassMg[index];
                    } else if (
                        preferredEdgeIndex < 0 &&
                        sourceReadyForNetwork
                    ) {
                        this.sourceHandoffIodineMassMg +=
                            this.iodineMassMg[index];
                    }
                    this._removeParticle(index);
                    continue;
                }
            }
            index++;
        }
    }

    // Follow the parcel through connected blood lumens. A true fork is not a
    // wall: the upstream-directed jet may physically cross its parent plane,
    // and a downstream jet may enter a containing daughter. Re-querying the
    // whole atlas here could jump to an unrelated nearby artery.
    resolveSourceHandoffLocation(index, out = {}) {
        const edges = this.flowNetwork.edges;
        let edgeIndex = this.handoffEdgeIndex[index] >= 0
            ? this.handoffEdgeIndex[index] : this.sourceEdgeIndex[index];
        let edge = edges[edgeIndex];
        if (!edge) { out.edgeIndex = -1; return out; }
        const x = this.positionX[index], y = this.positionY[index], z = this.positionZ[index];
        let axial = projectedAxial(edge, x, y, z);
        let distance = projectedDistanceSquared(edge, axial, x, y, z);
        const source = edges[this.sourceEdgeIndex[index]];
        if (source && (x - source.start.x) * source.axis.x +
            (y - source.start.y) * source.axis.y + (z - source.start.z) * source.axis.z < 0) {
            // Near the outer wall of a curved arch the closest point may lie
            // several short atlas edges upstream, despite an intervening
            // segment being slightly farther away. Look through only this
            // finite connected parent neighbourhood, never another artery.
            const reach = Math.max(source.radiusStart, source.radiusEnd) * 2;
            let parent = edges[source.parentEdgeIndex], travelled = 0;
            while (parent && travelled < reach && !parent.transportExcluded &&
                (!parent.lumenMappingExcluded || parent.intraluminalAorticConnector)) {
                const s = projectedAxial(parent, x, y, z);
                const d = projectedDistanceSquared(parent, s, x, y, z);
                if (insideTransportLumen(parent, s, d) && d < distance) {
                    edge = parent; edgeIndex = edge.index; axial = s; distance = d;
                }
                travelled += parent.length;
                parent = edges[parent.parentEdgeIndex];
            }
        }
        for (let hop = 0; hop < edges.length; hop++) {
            let candidate = null, candidateAxial = axial, candidateDistance = distance;
            const parent = edges[edge.parentEdgeIndex];
            for (let side = -1; side < edge.childEdgeIndices.length; side++) {
                const adjacent = side < 0 ? parent : edges[edge.childEdgeIndices[side]];
                if (!adjacent || adjacent.transportExcluded) continue;
                const s = projectedAxial(adjacent, x, y, z);
                const d = projectedDistanceSquared(adjacent, s, x, y, z);
                const backwards = adjacent === parent;
                const fork = backwards ? parent.childEdgeIndices.length > 1 : edge.childEdgeIndices.length > 1;
                if (fork) {
                    const from = backwards ? edge : adjacent;
                    const signed = (x - from.start.x) * from.axis.x +
                        (y - from.start.y) * from.axis.y + (z - from.start.z) * from.axis.z;
                    if ((backwards ? signed > 0 : signed < 0) || !insideTransportLumen(adjacent, s, d)) continue;
                    // A small ostial axis runs inside the larger parent blood
                    // volume. Its shorter centreline distance cannot siphon
                    // a jet out of that shared volume. Select the daughter
                    // once the parcel really exits the parent lumen; until
                    // then ordinary junction outflow divides entrained iodine.
                    if (!backwards && adjacent.radiusStart < edge.radiusEnd * .85 &&
                        insideTransportLumen(edge, axial, distance)) continue;
                }
                // At a bent fork both incident polylines can initially clamp
                // to their identical node. Permit the upstream crossing of
                // that shared endpoint; the next parent segment then locates
                // the real parcel instead of trapping it at the source cap.
                const crossesSharedStart = backwards && axial === 0 && s === adjacent.length &&
                    (x - edge.start.x) * edge.axis.x + (y - edge.start.y) * edge.axis.y +
                    (z - edge.start.z) * edge.axis.z < 0;
                if (d + 1e-10 < candidateDistance || (crossesSharedStart && d <= candidateDistance + 1e-10)) {
                    candidate = adjacent; candidateAxial = s; candidateDistance = d;
                }
            }
            if (!candidate) break;
            edge = candidate; edgeIndex = edge.index;
            axial = candidateAxial; distance = candidateDistance;
        }
        this.handoffEdgeIndex[index] = edgeIndex;
        const cellPosition = axial / edge.cellLength - 0.5;
        let lowerEdge = edge, upperEdge = edge;
        let lower = clamp(Math.floor(cellPosition), 0, edge.cellCount - 1);
        let upper = clamp(Math.ceil(cellPosition), 0, edge.cellCount - 1);
        let upperWeight = clamp(cellPosition - lower, 0, 1);
        if (cellPosition < 0) {
            const candidate = edges[edge.parentEdgeIndex];
            const parent = candidate && !candidate.transportExcluded &&
                (candidate.childEdgeIndices.length === 1 || insideTransportLumen(candidate,
                    projectedAxial(candidate, x, y, z), projectedDistanceSquared(candidate,
                        projectedAxial(candidate, x, y, z), x, y, z))) ? candidate : null;
            if (parent) {
                lowerEdge = parent; lower = parent.cellCount - 1; upper = 0;
                // At a fork its parent terminal volume owns the shared node.
                // Blend from that same volume into the reached daughter, so
                // crossing the node cannot switch a whole parcel's handoff.
                upperWeight = parent.childEdgeIndices.length > 1 ? axial / (edge.cellLength * .5) :
                    (axial + parent.cellLength * .5) / ((parent.cellLength + edge.cellLength) * .5);
            }
        } else if (cellPosition > edge.cellCount - 1) {
            const child = chainChild(edges, edge);
            if (child) {
                upperEdge = child; upper = 0; lower = edge.cellCount - 1;
                upperWeight = (axial - edge.length + edge.cellLength * 0.5) /
                    ((edge.cellLength + child.cellLength) * 0.5);
            }
        }
        out.edgeIndex = edgeIndex;
        out.cellIndex = clamp(Math.floor(axial / edge.cellLength), 0, edge.cellCount - 1);
        out.t = axial / edge.length;
        out.lowerEdgeIndex = lowerEdge.index; out.lowerCellIndex = lower;
        out.upperEdgeIndex = upperEdge.index; out.upperCellIndex = upper;
        out.upperWeight = clamp(upperWeight, 0, 1);
        return out;
    }

    _depositSourceEntrainment(index, mass) {
        const net = this.flowNetwork;
        if (net.depositGraftSacAtCoordinates(
            this.positionX[index], this.positionY[index], this.positionZ[index], mass
        )) return mass;
        const sourceEdge = this.sourceEdgeIndex[index], sourceCell = this.sourceCellIndex[index];
        if (this._hasBidirectionalSourceOutflow(sourceEdge, sourceCell)) {
            // Opposing source outflows share one finite mixing volume. Moving
            // all entrained iodine to the jet's upstream side bypasses this
            // hydraulic split and sends too much contrast into reflux. Keep
            // the resolved jet separate, but release its mixed fraction into
            // that shared volume so the signed face fluxes divide it once.
            return net.queueIodineSource(sourceEdge, sourceCell, mass) ? mass : 0;
        }
        const location = this.resolveSourceHandoffLocation(index, this._handoffLocationScratch);
        if (location.edgeIndex < 0) return 0;
        const upperMass = mass * location.upperWeight, lowerMass = mass - upperMass;
        let deposited = 0;
        if (lowerMass > 0 && net.queueIodineSource(location.lowerEdgeIndex, location.lowerCellIndex, lowerMass))
            deposited += lowerMass;
        if (upperMass > 0 && net.queueIodineSource(location.upperEdgeIndex, location.upperCellIndex, upperMass))
            deposited += upperMass;
        return deposited;
    }

    _hasBidirectionalSourceOutflow(edgeIndex, cellIndex) {
        const net = this.flowNetwork;
        const heartRate = net.hemodynamics.heartRateBpm;
        if (this._sourceFlowTime !== net.time || this._sourceFlowHeartRate !== heartRate) {
            this._sourceFlowTime = net.time;
            this._sourceFlowHeartRate = heartRate;
            this._sourceFlowWaveform = arterialWaveform(net.time, heartRate);
        }
        const waveform = this._sourceFlowWaveform;
        return net.getFaceSignedFlowMm3PerS(edgeIndex, cellIndex, waveform) < 0 &&
            net.getFaceSignedFlowMm3PerS(edgeIndex, cellIndex + 1, waveform) > 0;
    }

    _depositOverflowIodine(edgeIndex, sourceCell, location, mass) {
        // Capacity is a last-resort unresolved mixing fallback. Keep its
        // source topology, but retain the outlet's subcell position rather
        // than introducing a new hard bin boundary at high injection rates.
        const edge = this.flowNetwork.edges[edgeIndex];
        if (this._hasBidirectionalSourceOutflow(edgeIndex, sourceCell))
            return this.flowNetwork.queueIodineSource(edgeIndex, sourceCell, mass);
        const position = location.edgeIndex === edgeIndex
            ? location.t * edge.cellCount - 0.5 : sourceCell;
        const lower = clamp(Math.floor(position), 0, edge.cellCount - 1);
        const upper = clamp(Math.ceil(position), 0, edge.cellCount - 1);
        const fraction = clamp(position - lower, 0, 1);
        const upperMass = mass * fraction;
        if (mass - upperMass > 0 && !this.flowNetwork.queueIodineSource(edgeIndex, lower, mass - upperMass)) return false;
        return !(upperMass > 0) || this.flowNetwork.queueIodineSource(edgeIndex, upper, upperMass);
    }

    _removeParticle(index) {
        const last = --this.count;
        if (index === last) return;
        this.positionX[index] = this.positionX[last];
        this.positionY[index] = this.positionY[last];
        this.positionZ[index] = this.positionZ[last];
        this.outletPositionX[index] = this.outletPositionX[last];
        this.outletPositionY[index] = this.outletPositionY[last];
        this.outletPositionZ[index] = this.outletPositionZ[last];
        this.outletDirectionX[index] = this.outletDirectionX[last];
        this.outletDirectionY[index] = this.outletDirectionY[last];
        this.outletDirectionZ[index] = this.outletDirectionZ[last];
        this.outletRadiusMm[index] = this.outletRadiusMm[last];
        this.resolvedOutletRadiusMm[index] = this.resolvedOutletRadiusMm[last];
        this.initialTransportSpeedMmPerSec[index] = this.initialTransportSpeedMmPerSec[last];
        this.jetVelocityX[index] = this.jetVelocityX[last];
        this.jetVelocityY[index] = this.jetVelocityY[last];
        this.jetVelocityZ[index] = this.jetVelocityZ[last];
        this.jetDecayPerSecond[index] = this.jetDecayPerSecond[last];
        this.iodineMassMg[index] = this.iodineMassMg[last];
        this.initialIodineMassMg[index] = this.initialIodineMassMg[last];
        this.ageSeconds[index] = this.ageSeconds[last];
        this.birthDelaySeconds[index] = this.birthDelaySeconds[last];
        this.travelDistanceMm[index] = this.travelDistanceMm[last];
        this.handoffDistanceMm[index] = this.handoffDistanceMm[last];
        this.preferredHandoffEdgeIndex[index] = this.preferredHandoffEdgeIndex[last];
        this.preferredHandoffCellIndex[index] = this.preferredHandoffCellIndex[last];
        this.sourceEdgeIndex[index] = this.sourceEdgeIndex[last];
        this.sourceCellIndex[index] = this.sourceCellIndex[last];
        this.handoffEdgeIndex[index] = this.handoffEdgeIndex[last];
        this.retrogradeProgressMm[index] = this.retrogradeProgressMm[last];
    }

    getIodineMassMg() {
        let mass = 0;
        for (let index = 0; index < this.count; index++) mass += this.iodineMassMg[index];
        return mass;
    }

    getDiagnostics() {
        return {
            particleCount: this.count,
            capacity: this.capacity,
            localIodineMassMg: this.getIodineMassMg(),
            totalEmittedIodineMassMg: this.totalEmittedIodineMassMg,
            totalHandedOffIodineMassMg: this.totalHandedOffIodineMassMg,
            directHandoffIodineMassMg: this.directHandoffIodineMassMg,
            retrogradeTargetedIodineMassMg: this.retrogradeTargetedIodineMassMg,
            retrogradeHandoffIodineMassMg: this.retrogradeHandoffIodineMassMg,
            retrogradeEntrainedIodineMassMg: this.retrogradeEntrainedIodineMassMg,
            sourceHandoffIodineMassMg:
                this.sourceHandoffIodineMassMg,
            catheterSourceHandoffIodineMassMg:
                this.sourceHandoffIodineMassMg,
            maximumRetrogradeProgressMm: this.maximumRetrogradeProgressMm,
            lastSourceEdgeIndex: this.lastSourceEdgeIndex,
            lastSourceCellIndex: this.lastSourceCellIndex,
            lastSourceRadiusMm: this.lastSourceRadiusMm,
            lastSourceDistanceMm: this.lastSourceDistanceMm,
            lastSourceSelectionMode: this.lastSourceSelectionMode,
            lastSourceT: this.lastSourceT,
            lastSourcePortPosition: { ...this.lastSourcePortPosition },
            lastSourcePortDirection: { ...this.lastSourcePortDirection },
            // Compatibility aliases for existing scenario reports.
            lastCatheterSourceEdgeIndex:
                this.lastSourceEdgeIndex,
            lastCatheterSourceCellIndex:
                this.lastSourceCellIndex,
            lastCatheterSourceRadiusMm:
                this.lastSourceRadiusMm,
            lastCatheterSourceDistanceMm:
                this.lastSourceDistanceMm,
            lastCatheterSourceSelectionMode:
                this.lastSourceSelectionMode,
            lastCatheterSourceT: this.lastSourceT,
            lastCatheterPortPosition: {
                ...this.lastSourcePortPosition
            },
            lastCatheterPortDirection: {
                ...this.lastSourcePortDirection
            },
            lastDirectionAgainstFlow:
                this.lastDirectionAgainstFlow,
            lastRetrogradeAlignment:
                this.lastRetrogradeAlignment,
            lastRetrogradeMomentumFluxRatio:
                this.lastRetrogradeMomentumFluxRatio,
            lastPhysicalJetSpeedMmPerSec:
                this.lastPhysicalJetSpeedMmPerSec,
            lastResolvedJetSpeedMmPerSec:
                this.lastResolvedJetSpeedMmPerSec,
            lastEquivalentOutletDiameterMm:
                this.lastEquivalentOutletDiameterMm,
            lastJetMixingLengthMm:
                this.lastJetMixingLengthMm,
            lastTargetedCoreFraction:
                this.lastTargetedCoreFraction,
            sourceMappingChangeCount: this.sourceMappingChangeCount,
            sourceMassByEdge: [...this.sourceMassByEdge]
                .sort((a, b) => b[1] - a[1]),
            catheterSourceMappingChangeCount:
                this.sourceMappingChangeCount,
            catheterSourceMassByEdge: [...this.sourceMassByEdge]
                .sort((a, b) => b[1] - a[1]),
            maxWallPenetrationMm: this.maxWallPenetrationMm
        };
    }
}
