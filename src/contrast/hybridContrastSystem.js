import * as THREE from 'three';
import {
    INTRODUCER_SHEATH_INNER_RADIUS_MM,
    PIGTAIL_CATHETER_INNER_RADIUS_MM
} from '../toolDimensions.js';
import { createContactResult } from '../physics/collision/vesselContactField.js';
import { ContrastFlowNetwork, DEFAULT_HEMODYNAMICS } from './flowNetwork.js';
import {
    DEFAULT_CONTRAST_MEDIUM,
    LocalContrastInjectionSolver
} from './localInjectionSolver.js';
import {
    computeInjectionHydraulics,
    mergeDeviceHydraulicProfiles,
    normalizeInjectorHydraulics
} from './injectionHydraulics.js';
import { PressureDrivenRetrogradeColumn } from './pressureDrivenRetrogradeColumn.js';
import { ContinuousInjectionFlowCoupler } from './continuousInjectionFlowCoupler.js';

export const CONTRAST_SOURCE_SHEATH = 'sheath';
export const CONTRAST_SOURCE_CATHETER = 'catheter';

const SOURCE_LABELS = Object.freeze({
    [CONTRAST_SOURCE_SHEATH]: 'Sheath',
    [CONTRAST_SOURCE_CATHETER]: 'Catheter'
});
const CONTRAST_SOLVER_STEP_SECONDS = 1 / 30;
const MAX_CONTRAST_SUBSTEPS = 4;
const PIGTAIL_SIDE_HOLE_RADIUS_MM = 0.22;
const PIGTAIL_SIDE_HOLE_COUNT = 8;

function nominalPort(radiusMm, direction = { x: 0, y: 1, z: 0 }) {
    const areaMm2 = Math.PI * radiusMm ** 2;
    return Object.freeze({
        radiusMm,
        areaMm2,
        weight: areaMm2,
        direction: Object.freeze({ ...direction })
    });
}

const NOMINAL_HYDRAULIC_PORTS = Object.freeze({
    sheath: Object.freeze([
        nominalPort(INTRODUCER_SHEATH_INNER_RADIUS_MM)
    ]),
    berenstein: Object.freeze([
        nominalPort(PIGTAIL_CATHETER_INNER_RADIUS_MM)
    ]),
    pigtail: Object.freeze(
        Array.from({ length: PIGTAIL_SIDE_HOLE_COUNT }, (_, index) => {
            const angle = index / PIGTAIL_SIDE_HOLE_COUNT * Math.PI * 2;
            return nominalPort(PIGTAIL_SIDE_HOLE_RADIUS_MM, {
                x: Math.cos(angle),
                y: 0,
                z: Math.sin(angle)
            });
        })
    )
});

function cloneVector(point) {
    return point?.clone?.() || new THREE.Vector3(point?.x || 0, point?.y || 0, point?.z || 0);
}

export class HybridContrastSystem {
    constructor({
        centerlineSegments,
        contactField = null,
        sheath = null,
        catheter = null,
        hemodynamics = DEFAULT_HEMODYNAMICS,
        medium = DEFAULT_CONTRAST_MEDIUM,
        injectorHydraulics = {},
        deviceHydraulicProfiles = {},
        flowOptions = {},
        localOptions = {}
    } = {}) {
        this.contactField = contactField;
        this.sheath = sheath;
        this.catheter = catheter;
        this.medium = { ...DEFAULT_CONTRAST_MEDIUM, ...medium };
        this.injectorHydraulics = normalizeInjectorHydraulics(
            injectorHydraulics
        );
        this.deviceHydraulicProfiles = mergeDeviceHydraulicProfiles(
            deviceHydraulicProfiles
        );
        this.flowNetwork = new ContrastFlowNetwork(centerlineSegments, {
            ...hemodynamics,
            ...flowOptions
        });
        this.localSolver = new LocalContrastInjectionSolver({
            flowNetwork: this.flowNetwork,
            contactField,
            ...localOptions
        });
        this.pressureDrivenRetrogradeColumn =
            new PressureDrivenRetrogradeColumn(this.flowNetwork);
        this.continuousInjectionFlowCoupler =
            new ContinuousInjectionFlowCoupler(this.flowNetwork);
        this.injection = null;
        this.totalDeliveredVolumeMl = 0;
        this.totalInjectedIodineMassMg = 0;
        this.lastStopReason = null;
        this.lastInjectionHydraulics = null;
        this._sourcePorts = [];
        this._solverAccumulator = 0;
        this._contactScratch = createContactResult();
        this._sheathStart = sheath?.start ? cloneVector(sheath.start) : null;
        this._sheathEnd = sheath?.end ? cloneVector(sheath.end) : null;
        this._sheathDirection = this._sheathStart && this._sheathEnd
            ? this._sheathEnd.clone().sub(this._sheathStart).normalize()
            : new THREE.Vector3(0, 1, 0);
        this._sheathLength = this._sheathStart && this._sheathEnd
            ? this._sheathStart.distanceTo(this._sheathEnd)
            : 0;
    }

    setCatheter(catheter) {
        this.catheter = catheter;
    }

    setHemodynamics(parameters) {
        return this.flowNetwork.setHemodynamics(parameters);
    }

    setInjectionHydraulicParameters({
        maximumPressurePsi,
        viscosityPaS,
        deviceProfiles
    } = {}) {
        const nextInjector = maximumPressurePsi == null
            ? this.injectorHydraulics
            : normalizeInjectorHydraulics({
                ...this.injectorHydraulics,
                maximumPressurePsi
            });
        const nextViscosity = viscosityPaS == null
            ? this.medium.viscosityPaS
            : Number(viscosityPaS);
        if (!(Number.isFinite(nextViscosity) && nextViscosity > 0)) {
            throw new RangeError(
                'contrast viscosity must be a positive finite number'
            );
        }
        const nextProfiles = deviceProfiles == null
            ? this.deviceHydraulicProfiles
            : mergeDeviceHydraulicProfiles({
                ...this.deviceHydraulicProfiles,
                ...deviceProfiles
            });
        this.injectorHydraulics = nextInjector;
        this.medium.viscosityPaS = nextViscosity;
        this.deviceHydraulicProfiles = nextProfiles;
        return this.getInjectionHydraulicParameters();
    }

    getInjectionHydraulicParameters() {
        return {
            maximumPressurePsi:
                this.injectorHydraulics.maximumPressurePsi,
            viscosityPaS: this.medium.viscosityPaS,
            deviceProfiles: Object.fromEntries(
                Object.entries(this.deviceHydraulicProfiles).map(
                    ([key, profile]) => [key, { ...profile }]
                )
            )
        };
    }

    _deviceKeyForSource(source) {
        if (source === CONTRAST_SOURCE_SHEATH) return 'sheath';
        if (source === CONTRAST_SOURCE_CATHETER) {
            return this.catheter?.type === 'berenstein'
                ? 'berenstein'
                : 'pigtail';
        }
        return null;
    }

    getSourceStatus(source) {
        const ports = this._resolveSourcePorts(source);
        if (!SOURCE_LABELS[source]) {
            return { valid: false, source, label: 'Unknown', reason: 'Unknown injection source', ports };
        }
        if (!ports.length) {
            const deviceKey = this._deviceKeyForSource(source);
            const reason = source === CONTRAST_SOURCE_CATHETER
                ? 'Advance the catheter ports beyond the sheath'
                : 'Sheath outlet is unavailable';
            return {
                valid: false,
                source,
                label: SOURCE_LABELS[source],
                reason,
                deviceKey,
                ports
            };
        }
        return {
            valid: true,
            source,
            label: source === CONTRAST_SOURCE_CATHETER
                ? `${SOURCE_LABELS[source]} · ${this.catheter?.type || 'active'}`
                : SOURCE_LABELS[source],
            reason: '',
            deviceKey: this._deviceKeyForSource(source),
            ports
        };
    }

    getInjectionPreview({
        source = CONTRAST_SOURCE_SHEATH,
        volumeMl = 0,
        rateMlPerSec
    } = {}) {
        const sourceStatus = this.getSourceStatus(source);
        const deviceKey = sourceStatus.deviceKey;
        if (!sourceStatus.valid) {
            const nominalPorts = NOMINAL_HYDRAULIC_PORTS[deviceKey];
            let maximumAchievableRateMlPerSec = null;
            if (nominalPorts?.length && this.deviceHydraulicProfiles[deviceKey]) {
                maximumAchievableRateMlPerSec =
                    computeInjectionHydraulics({
                        requestedRateMlPerSec: 1,
                        ports: nominalPorts,
                        deviceProfile: this.deviceHydraulicProfiles[deviceKey],
                        injector: this.injectorHydraulics,
                        medium: this.medium
                    }).maximumAchievableRateMlPerSec;
            }
            return {
                valid: false,
                source,
                deviceKey,
                reason: sourceStatus.reason,
                maximumAchievableRateMlPerSec
            };
        }
        try {
            const hydraulics = computeInjectionHydraulics({
                requestedRateMlPerSec: rateMlPerSec,
                ports: sourceStatus.ports,
                deviceProfile: this.deviceHydraulicProfiles[deviceKey],
                injector: this.injectorHydraulics,
                medium: this.medium
            });
            return {
                valid: true,
                source,
                deviceKey,
                volumeMl,
                durationSeconds: volumeMl > 0
                    ? volumeMl / hydraulics.actualRateMlPerSec
                    : 0,
                ...hydraulics
            };
        } catch (error) {
            return {
                valid: false,
                source,
                deviceKey,
                reason: error instanceof Error
                    ? error.message
                    : String(error)
            };
        }
    }

    startInjection({
        source = CONTRAST_SOURCE_SHEATH,
        volumeMl,
        rateMlPerSec
    }) {
        if (this.injection) {
            return { ok: false, reason: 'An injection is already active' };
        }
        if (!(volumeMl > 0) || !(rateMlPerSec > 0)) {
            return { ok: false, reason: 'Volume and rate must be greater than zero' };
        }
        const preview = this.getInjectionPreview({
            source,
            volumeMl,
            rateMlPerSec
        });
        if (!preview.valid) return { ok: false, reason: preview.reason };
        const actualRateMlPerSec = preview.actualRateMlPerSec;
        this.injection = {
            source,
            requestedVolumeMl: volumeMl,
            deliveredVolumeMl: 0,
            remainingVolumeMl: volumeMl,
            requestedRateMlPerSec: rateMlPerSec,
            actualRateMlPerSec,
            rateMlPerSec: actualRateMlPerSec,
            elapsedSeconds: 0,
            durationSeconds: volumeMl / actualRateMlPerSec,
            hydraulics: { ...preview }
        };
        this.lastInjectionHydraulics = { ...preview };
        this.lastStopReason = null;
        return {
            ok: true,
            source,
            durationSeconds: this.injection.durationSeconds,
            requestedRateMlPerSec: rateMlPerSec,
            actualRateMlPerSec,
            requiredPressurePsi: preview.requiredPressurePsi,
            appliedPressurePsi: preview.appliedPressurePsi,
            pressureLimited: preview.pressureLimited
        };
    }

    stopInjection(reason = 'Stopped by operator') {
        if (!this.injection) return false;
        this.lastStopReason = reason;
        this.injection = null;
        return true;
    }

    update(dt) {
        if (!(dt > 0)) return;
        this._solverAccumulator += dt;
        let substeps = 0;
        while (
            this._solverAccumulator + 1e-9 >= CONTRAST_SOLVER_STEP_SECONDS &&
            substeps < MAX_CONTRAST_SUBSTEPS
        ) {
            this.flowNetwork.clearFlowOverrides();
            this.pressureDrivenRetrogradeColumn.beginStep();
            this.continuousInjectionFlowCoupler.beginStep();
            if (this.injection) this._updateInjection(CONTRAST_SOLVER_STEP_SECONDS);
            this.localSolver.update(CONTRAST_SOLVER_STEP_SECONDS);
            this.flowNetwork.update(CONTRAST_SOLVER_STEP_SECONDS);
            this.pressureDrivenRetrogradeColumn.updateDiagnostics(
                this.medium.iodineMgPerMl
            );
            this._solverAccumulator -= CONTRAST_SOLVER_STEP_SECONDS;
            substeps++;
        }
        if (this._solverAccumulator >= CONTRAST_SOLVER_STEP_SECONDS) {
            this._solverAccumulator %= CONTRAST_SOLVER_STEP_SECONDS;
        }
    }

    _updateInjection(dt) {
        const injection = this.injection;
        const sourceStatus = this.getSourceStatus(injection.source);
        if (!sourceStatus.valid) {
            this.stopInjection(sourceStatus.reason);
            return;
        }
        const requestedVolume = Math.min(
            injection.remainingVolumeMl,
            injection.actualRateMlPerSec * dt
        );
        // Every device enters one continuous volume-source model. The outlet
        // modifies signed face flows on both sides of its actual control volume;
        // sufficiently strong injection therefore crosses zero into reflux
        // without selecting another transport algorithm.
        const portFlowSplits =
            this.continuousInjectionFlowCoupler.configure({
                ports: sourceStatus.ports,
                rateMlPerSec: injection.actualRateMlPerSec,
                portFlowRatesMlPerSec:
                    injection.hydraulics.portFlowRatesMlPerSec
            });
        const emitted = this.localSolver.emit({
            ports: sourceStatus.ports,
            volumeMl: requestedVolume,
            rateMlPerSec: injection.actualRateMlPerSec,
            emissionWindowSeconds: dt,
            medium: this.medium,
            allowPressureDrivenRetrograde: false,
            portInjectionLocations: portFlowSplits.map(
                split => split.location || null
            ),
            portFlowRatesMlPerSec:
                injection.hydraulics.portFlowRatesMlPerSec,
            portJetVelocitiesMPerSec:
                injection.hydraulics.portJetVelocitiesMPerSec
        });
        const deliveredVolume = emitted.emittedVolumeMl;
        injection.deliveredVolumeMl += deliveredVolume;
        injection.remainingVolumeMl = Math.max(
            0,
            injection.requestedVolumeMl - injection.deliveredVolumeMl
        );
        injection.elapsedSeconds += dt;
        this.totalDeliveredVolumeMl += deliveredVolume;
        this.totalInjectedIodineMassMg += emitted.emittedIodineMassMg;
        if (injection.remainingVolumeMl <= 1e-6) {
            this.lastStopReason = 'Completed';
            this.injection = null;
        }
    }

    _resolveSourcePorts(source) {
        this._sourcePorts.length = 0;
        if (source === CONTRAST_SOURCE_SHEATH) {
            if (!this._sheathEnd) return this._sourcePorts;
            const port = {
                kind: 'sheath-end',
                position: this._sheathEnd,
                direction: this._sheathDirection,
                radiusMm: INTRODUCER_SHEATH_INNER_RADIUS_MM,
                areaMm2: Math.PI * INTRODUCER_SHEATH_INNER_RADIUS_MM ** 2,
                weight: Math.PI * INTRODUCER_SHEATH_INNER_RADIUS_MM ** 2,
                valid: this._isInsideVessel(this._sheathEnd)
            };
            if (port.valid) this._sourcePorts.push(port);
            return this._sourcePorts;
        }
        if (source !== CONTRAST_SOURCE_CATHETER) return this._sourcePorts;
        const catheterPorts = this.catheter?.getInjectionPorts?.(this._sourcePorts) || this._sourcePorts;
        let writeIndex = 0;
        for (let index = 0; index < catheterPorts.length; index++) {
            const port = catheterPorts[index];
            const valid = port?.position &&
                !this._isPortInsideSheath(port.position) &&
                this._isInsideVessel(port.position);
            port.valid = valid;
            if (valid) catheterPorts[writeIndex++] = port;
        }
        catheterPorts.length = writeIndex;
        return catheterPorts;
    }

    _isInsideVessel(point) {
        if (!this.contactField?.querySphere) return true;
        return this.contactField.querySphere(point, 0.02, this._contactScratch).inside;
    }

    _isPortInsideSheath(point) {
        if (!this._sheathStart || !this._sheathEnd || this._sheathLength <= 0) return false;
        const relX = point.x - this._sheathStart.x;
        const relY = point.y - this._sheathStart.y;
        const relZ = point.z - this._sheathStart.z;
        const axial =
            relX * this._sheathDirection.x +
            relY * this._sheathDirection.y +
            relZ * this._sheathDirection.z;
        if (axial < -0.5 || axial > this._sheathLength - 0.5) return false;
        const centerX = this._sheathStart.x + this._sheathDirection.x * axial;
        const centerY = this._sheathStart.y + this._sheathDirection.y * axial;
        const centerZ = this._sheathStart.z + this._sheathDirection.z * axial;
        const radialDistance = Math.hypot(
            point.x - centerX,
            point.y - centerY,
            point.z - centerZ
        );
        return radialDistance <= (this.sheath?.radius || INTRODUCER_SHEATH_INNER_RADIUS_MM) + 0.5;
    }

    hasVisibleContrast(thresholdMg = 0.02) {
        const flowMass = this.flowNetwork.getIodineMassMg();
        const localMass = this.localSolver.getIodineMassMg();
        return flowMass + localMass > thresholdMg;
    }

    get isInjecting() {
        return this.injection !== null;
    }

    getMetrics() {
        const flow = this.flowNetwork.getMassBalanceSnapshot();
        const local = this.localSolver.getDiagnostics();
        const column =
            this.pressureDrivenRetrogradeColumn.getDiagnostics();
        const continuousFlowSplit =
            this.continuousInjectionFlowCoupler.getDiagnostics();
        const accountedMass =
            flow.intravascularIodineMassMg +
            flow.outletIodineMassMg +
            local.localIodineMassMg;
        const balanceErrorMg = this.totalInjectedIodineMassMg - accountedMass;
        return {
            totalDeliveredVolumeMl: this.totalDeliveredVolumeMl,
            totalInjectedIodineMassMg: this.totalInjectedIodineMassMg,
            intravascularIodineMassMg: flow.intravascularIodineMassMg,
            localIodineMassMg: local.localIodineMassMg,
            outletIodineMassMg: flow.outletIodineMassMg,
            balanceErrorMg,
            relativeBalanceError: this.totalInjectedIodineMassMg > 0
                ? balanceErrorMg / this.totalInjectedIodineMassMg
                : 0,
            activeParticleCount: local.particleCount,
            maxWallPenetrationMm: local.maxWallPenetrationMm,
            retrogradeTargetedIodineMassMg: local.retrogradeTargetedIodineMassMg,
            retrogradeHandoffIodineMassMg: local.retrogradeHandoffIodineMassMg,
            retrogradeEntrainedIodineMassMg:
                local.retrogradeEntrainedIodineMassMg,
            sourceHandoffIodineMassMg:
                local.sourceHandoffIodineMassMg,
            catheterSourceHandoffIodineMassMg:
                local.catheterSourceHandoffIodineMassMg,
            lastSourceEdgeIndex: local.lastSourceEdgeIndex,
            lastSourceCellIndex: local.lastSourceCellIndex,
            lastSourceRadiusMm: local.lastSourceRadiusMm,
            lastSourceDistanceMm: local.lastSourceDistanceMm,
            lastSourceSelectionMode: local.lastSourceSelectionMode,
            lastSourceT: local.lastSourceT,
            lastSourcePortPosition: local.lastSourcePortPosition,
            lastSourcePortDirection: local.lastSourcePortDirection,
            lastCatheterSourceEdgeIndex:
                local.lastCatheterSourceEdgeIndex,
            lastCatheterSourceCellIndex:
                local.lastCatheterSourceCellIndex,
            lastCatheterSourceRadiusMm:
                local.lastCatheterSourceRadiusMm,
            lastCatheterSourceDistanceMm:
                local.lastCatheterSourceDistanceMm,
            lastCatheterSourceSelectionMode:
                local.lastCatheterSourceSelectionMode,
            lastCatheterSourceT: local.lastCatheterSourceT,
            lastCatheterPortPosition: local.lastCatheterPortPosition,
            lastCatheterPortDirection: local.lastCatheterPortDirection,
            lastDirectionAgainstFlow:
                local.lastDirectionAgainstFlow,
            lastRetrogradeAlignment:
                local.lastRetrogradeAlignment,
            lastRetrogradeMomentumFluxRatio:
                local.lastRetrogradeMomentumFluxRatio,
            lastPhysicalJetSpeedMmPerSec:
                local.lastPhysicalJetSpeedMmPerSec,
            lastResolvedJetSpeedMmPerSec:
                local.lastResolvedJetSpeedMmPerSec,
            lastEquivalentOutletDiameterMm:
                local.lastEquivalentOutletDiameterMm,
            lastJetMixingLengthMm:
                local.lastJetMixingLengthMm,
            lastTargetedCoreFraction:
                local.lastTargetedCoreFraction,
            catheterSourceMappingChangeCount:
                local.catheterSourceMappingChangeCount,
            catheterSourceMassByEdge:
                local.catheterSourceMassByEdge,
            sourceMappingChangeCount: local.sourceMappingChangeCount,
            sourceMassByEdge: local.sourceMassByEdge,
            maximumRetrogradeProgressMm: local.maximumRetrogradeProgressMm,
            retrogradeColumn: column,
            continuousFlowSplit,
            injectionHydraulics: this.injection?.hydraulics ||
                this.lastInjectionHydraulics,
            injection: this.injection ? { ...this.injection } : null,
            lastStopReason: this.lastStopReason
        };
    }
}
