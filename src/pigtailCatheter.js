import * as THREE from 'three';
import { clamp, smoothstep } from './mathUtils.js';
import {
    PIGTAIL_CATHETER_INNER_RADIUS_MM,
    PIGTAIL_CATHETER_RADIUS_MM,
    PIGTAIL_CATHETER_RENDER_RADIUS_MM
} from './toolDimensions.js';
import { createSmoothTubeGeometry } from './smoothTubeGeometry.js';

const CATHETER_RADIUS = PIGTAIL_CATHETER_RADIUS_MM;
const PIGTAIL_RADIUS = 7.2;
const PIGTAIL_TURNS = 1.05;
const PIGTAIL_ARC_LENGTH = PIGTAIL_RADIUS * PIGTAIL_TURNS * Math.PI * 2;
const CATHETER_TYPE_PIGTAIL = 'pigtail';
const CATHETER_TYPE_BERENSTEIN = 'berenstein';
const BERENSTEIN_BEND_ANGLE = Math.PI / 4;
const BERENSTEIN_STRAIGHT_EXIT_LENGTH = 8;
const BERENSTEIN_BEND_LENGTH = 10;
const BERENSTEIN_TIP_SHAPE_LENGTH =
    BERENSTEIN_STRAIGHT_EXIT_LENGTH + BERENSTEIN_BEND_LENGTH;
const STRAIGHT_EXIT_LENGTH = 16;
const DISTAL_RELEASE_LENGTH = STRAIGHT_EXIT_LENGTH + PIGTAIL_ARC_LENGTH;
const MIN_GUIDE_SUPPORT = 18;
const GUIDE_CAPTURE_TOLERANCE = 4;
const FREE_NODE_SPACING = 3.2;
const FREE_SHAPE_STIFFNESS = 150;
const FREE_SHAPE_POSITION_BLEND = 0.24;
const FREE_ANCHOR_STIFFNESS = 0.96;
const FREE_DAMPING = 0.88;
const FREE_CONSTRAINT_ITERATIONS = 18;
const FREE_XPBD_TARGET_ITERATIONS = 4;
const FREE_BEND_SMOOTHING = 0.04;
const FREE_WALL_FRICTION = 0.08;
const FREE_SHAFT_STRAIGHTENING = 0.22;
const FREE_LONG_SPAN_STRAIGHTENING = 0.075;
const FREE_MAX_BEND_ANGLE = 72 * Math.PI / 180;
const FREE_BEND_LIMIT_STRENGTH = 0.36;
const FREE_SEGMENT_COLLISION_STRENGTH = 0.55;
const FREE_SEGMENT_MAX_CORRECTION = 1.2;
const FREE_SEGMENT_COLLISION_SAMPLES = [0.25, 0.5, 0.75];
const FREE_STRAIGHTENING_SPANS = [2, 4, 7];
const PATH_RELAXATION_PASSES = 3;
const PATH_STRAIGHTENING_SPANS = [2, 4, 8];
const PATH_STRAIGHTENING = 0.24;
const PATH_LONG_SPAN_STRAIGHTENING = 0.085;
const PATH_MAX_BEND_ANGLE = 68 * Math.PI / 180;
const PATH_BEND_LIMIT_STRENGTH = 0.42;
const PATH_MAX_RELAX_STEP = 1.15;
const SOLO_CATHETER_STRAIGHTENING_SCALE = 1.7;
const SOLO_CATHETER_BEND_LIMIT_SCALE = 1.45;
const GUIDEWIRE_RECAPTURE_WINDOW = 7;
const GUIDEWIRE_SUPPORT_BLEND = 0.22;
const GUIDEWIRE_MOVING_SUPPORT_BLEND = 0.42;
const PIGTAIL_RELEASE_CURL_START = 0.42;
const PIGTAIL_RELEASE_CURL_RATE = 2.4;
const SHAPE_RECOVERY_RATE = 2.6;
const SHAPE_RECAPTURE_RATE = 3.2;
const SOLO_XPBD_BEND_COMPLIANCE = 6e-7;
const SOLO_XPBD_SHAFT_MAX_BEND_ANGLE = 10;
const XPBD_SOFT_TIP_BEND_COMPLIANCE = 5e-6;
const BERENSTEIN_XPBD_SOFT_TIP_MAX_BEND_ANGLE = 20;
const XPBD_SHAPE_TARGET_SLEW_LIMIT = 1;
const XPBD_SHAPE_ACTIVATION_LENGTH = 10;
const XPBD_MIN_SHAPE_WEIGHT = 0.025;
const XPBD_SOFT_TIP_LENGTH = PIGTAIL_ARC_LENGTH;
const XPBD_SOFT_TIP_TRANSITION_LENGTH = 8;
const BERENSTEIN_XPBD_SOFT_TIP_LENGTH = 24;
const BERENSTEIN_XPBD_SOFT_TIP_TRANSITION_LENGTH = 12;
const PIGTAIL_XPBD_TARGET_MAX_OFFSET = 0.35;
const BERENSTEIN_XPBD_TARGET_MAX_OFFSET = 0.6;
const BERENSTEIN_XPBD_ROTATION_SHAPE_COMPLIANCE = 5e-6;
const XPBD_FREE_PATH_COMPLIANCE = 4e-6;
const XPBD_CONTACT_DRIVE_COMPLIANCE = 1e-4;
const XPBD_TIP_DRIVE_COMPLIANCE = 7.5e-6;
const XPBD_CENTERLINE_DIRECTION_BLEND = 0.24;
const XPBD_DIRECTION_SPATIAL_BLEND = 0.3;
const XPBD_DIRECTION_TEMPORAL_BLEND = 0.14;
const XPBD_RELEASE_STABILITY_LENGTH = 20;
const PIGTAIL_XPBD_POST_STABILIZATION_PASSES = 8;
const BERENSTEIN_XPBD_POST_STABILIZATION_PASSES = 4;
const GUIDEWIRE_IN_CATHETER_BLEND = 0.78;
const GUIDEWIRE_REACTION_BLEND = 0.16;
const GUIDEWIRE_CATHETER_MAX_CORRECTION = 1.2;
const EXTERNAL_CATHETER_VISIBLE_LENGTH = 90;
const CATHETER_ADVANCE_SPEED = 52;
const CATHETER_WITHDRAW_SPEED = 32;
const ROTATION_SPEED = Math.PI * 0.9;
const CONTACT_CLEARANCE = CATHETER_RADIUS * 0.72;
const TIP_MARKER_LENGTH = 2.4;
const TIP_MARKER_RADIUS = PIGTAIL_CATHETER_RENDER_RADIUS_MM * 1.35;
const PIGTAIL_INJECTION_PORT_RADIUS_MM = 0.22;
const PIGTAIL_INJECTION_PORT_OFFSETS_MM = Object.freeze([3, 6, 9, 12, 15, 18, 21, 24]);
const GOLDEN_ANGLE = Math.PI * (3 - Math.sqrt(5));

class TypedVector3 extends THREE.Vector3 {
    constructor(x = 0, y = 0, z = 0) {
        super(x, y, z);
        this._values = new Float64Array([
            this._initialX ?? x,
            this._initialY ?? y,
            this._initialZ ?? z
        ]);
    }

    get x() { return this._values ? this._values[0] : this._initialX; }
    set x(value) {
        if (this._values) this._values[0] = value;
        else this._initialX = value;
    }

    get y() { return this._values ? this._values[1] : this._initialY; }
    set y(value) {
        if (this._values) this._values[1] = value;
        else this._initialY = value;
    }

    get z() { return this._values ? this._values[2] : this._initialZ; }
    set z(value) {
        if (this._values) this._values[2] = value;
        else this._initialZ = value;
    }
}

function magnitude3(x, y, z) {
    return Math.sqrt(x * x + y * y + z * z);
}

function nodePosition(node) {
    return new TypedVector3(node.x, node.y, node.z);
}

export class PigtailCatheter {
    constructor({ wire, segmentLength, guidewireLength, tailProgressRef, vessel = null, maxLength = 1000 }) {
        this.wire = wire;
        this.segmentLength = segmentLength;
        this.guidewireLength = guidewireLength;
        this.tailProgressRef = tailProgressRef;
        this.vessel = vessel;
        this.vesselColliders = this.#buildVesselColliders(vessel);
        this.collisionMesh = null;
        this.sheathPath = this.#buildSheathPath(vessel?.sheath);
        this.maxLength = maxLength;
        this.progress = 0;
        this.guidewireInserted = 0;
        this.previousGuidewireInserted = 0;
        this.guidewireDelta = 0;
        this.motionCommand = 0;
        this.rotationCommand = 0;
        this.rotation = 0;
        this._pendingXpbdRotation = 0;
        this._xpbdBerensteinTwisted = false;
        this.type = CATHETER_TYPE_PIGTAIL;
        this.pathSpacing = 4;
        this.pathSamples = [];
        this._pathSamplePool = Array.from(
            { length: Math.ceil(maxLength / this.pathSpacing) + 4 },
            () => ({ distance: 0, point: new TypedVector3() })
        );
        this.freeNodes = [];
        this._nextFreeNodes = [];
        this._freeNodePool = [];
        this._freeNodeEpoch = 0;
        this.freeRestDistances = new Float64Array(Math.ceil(maxLength / FREE_NODE_SPACING) + 2);
        this.freeRestDistanceCount = 0;
        this.freeLength = 0;
        this._physicsStepIndex = 0;
        this.material = new THREE.MeshBasicMaterial({
            color: 0xffffff,
            depthTest: false,
            transparent: true,
            opacity: 1
        });
        this.tipMarkerMaterial = new THREE.MeshBasicMaterial({
            color: 0xffffff,
            depthTest: false,
            transparent: true,
            opacity: 1
        });
        this.maxRenderSegments = 320;
        this.shaftMesh = new THREE.Mesh(
            new THREE.BufferGeometry(),
            this.material
        );
        this.tipMarker = new THREE.Mesh(
            new THREE.CylinderGeometry(
                TIP_MARKER_RADIUS,
                TIP_MARKER_RADIUS,
                TIP_MARKER_LENGTH,
                16,
                1,
                false
            ),
            this.tipMarkerMaterial
        );
        this.tipMarker.frustumCulled = false;
        this.tipMarker.renderOrder = 8;
        this.tipMarker.visible = false;
        this.mesh = new THREE.Group();
        this.mesh.add(this.shaftMesh, this.tipMarker);
        this.mesh.frustumCulled = false;
        this.mesh.renderOrder = 7;
        this.mesh.visible = false;
        this.physicsBody = null;
        this.physicsActiveCount = 0;
        this.physicsLumenStartNode = 0;
        this._xpbdLayoutX = null;
        this._xpbdLayoutY = null;
        this._xpbdLayoutZ = null;
        this._xpbdDriveX = null;
        this._xpbdDriveY = null;
        this._xpbdDriveZ = null;
        this._xpbdDriveInitialized = null;
        this._xpbdLayoutCount = 0;
        this._xpbdProgress = 0;
        this._xpbdYieldsToWall = false;
        this._guidewireRelease = 1;
        this.externalCollisionSolver = false;
        this._renderPoints = [];
        this._tipMarkerPosition = new TypedVector3();
        this._tipMarkerTangent = new TypedVector3();
        this._tipMarkerUp = new TypedVector3(0, 1, 0);
        this._injectionPortPool = Array.from(
            { length: PIGTAIL_INJECTION_PORT_OFFSETS_MM.length },
            () => ({
                kind: 'pigtail-side',
                position: new THREE.Vector3(),
                direction: new THREE.Vector3(),
                radiusMm: PIGTAIL_INJECTION_PORT_RADIUS_MM,
                areaMm2: Math.PI * PIGTAIL_INJECTION_PORT_RADIUS_MM ** 2,
                weight: 1,
                valid: true
            })
        );
        this._injectionPosition = new THREE.Vector3();
        this._injectionTangent = new THREE.Vector3();
        this._injectionNormal = new THREE.Vector3();
        this._injectionBinormal = new THREE.Vector3();
        this._injectionHelper = new THREE.Vector3();
        this._shapeNormal = new TypedVector3();
        this._pathTarget = new TypedVector3();
        this._newNodeRest = new TypedVector3();
        this._newNodePath = new TypedVector3();
        this._newNodeGuide = new TypedVector3();
        this._newNodePoint = new TypedVector3();
        this._centerlinePoints = [];
        this._centerlineDistances = [];
        this._centerlinePointCount = 0;
        this._deploymentStateScratch = { pathEnd: 0, supportEnd: 0, freeLength: 0 };
        this._freeFrameScratch = {
            supportTip: new TypedVector3(),
            beforeTip: new TypedVector3(),
            beforePlane: new TypedVector3(),
            tangent: new TypedVector3(),
            normal: new TypedVector3()
        };
        this._guideReleaseFrameScratch = {
            supportTip: new TypedVector3(),
            beforeTip: new TypedVector3(),
            tangent: new TypedVector3(),
            normal: new TypedVector3()
        };
        this._planePreviousTangent = new TypedVector3();
        this._planeCurvature = new TypedVector3();
        this._planeHelper = new TypedVector3();
        this._xpbdDriveDirection = new TypedVector3();
        this._xpbdSoloTipTarget = new TypedVector3();
        this._xpbdSoloTipTargetActive = false;
        this._xpbdSoloTipControlIndex = -1;
    }

    setType(type) {
        const nextType = this.#normalizeType(type);
        if (this.type === nextType) return;
        this.#releaseUnsupportedXpbdTip();
        this.type = nextType;
        this.#clearFreeNodes();
        this.freeRestDistanceCount = 0;
        this.freeLength = 0;
        this._physicsStepIndex = 0;
        this.rotationCommand = 0;
        this._pendingXpbdRotation = 0;
        this._xpbdBerensteinTwisted = false;
        this.physicsLumenStartNode = 0;
        this._xpbdProgress = this.progress;
        this._xpbdYieldsToWall = false;
        this._xpbdDriveInitialized?.fill(0);
        this.updateMesh();
    }

    dispose() {
        this.#releaseUnsupportedXpbdTip();
        this.shaftMesh.geometry?.dispose?.();
        this.tipMarker.geometry?.dispose?.();
        this.material.dispose();
        this.tipMarkerMaterial.dispose();
    }

    setExternalCollisionSolver(enabled = true) {
        if (!enabled) this.#releaseUnsupportedXpbdTip();
        this.externalCollisionSolver = !!enabled;
        return this;
    }

    reset() {
        this.#releaseUnsupportedXpbdTip();
        this.progress = 0;
        this.guidewireInserted = 0;
        this.previousGuidewireInserted = 0;
        this.guidewireDelta = 0;
        this.motionCommand = 0;
        this.rotationCommand = 0;
        this.rotation = 0;
        this._pendingXpbdRotation = 0;
        this._xpbdBerensteinTwisted = false;
        this.pathSamples.length = 0;
        this.#clearFreeNodes();
        this.freeRestDistanceCount = 0;
        this.freeLength = 0;
        this._physicsStepIndex = 0;
        this.physicsActiveCount = 0;
        this._xpbdLayoutCount = 0;
        this._xpbdProgress = 0;
        this._xpbdYieldsToWall = false;
        this._xpbdDriveInitialized?.fill(0);
        this._guidewireRelease = 1;
        this.updateMesh();
        return this;
    }

    syncXpbdBody(body, {
        shapeCompliance = body.shapeCompliance,
        targetSlewLimit = XPBD_SHAPE_TARGET_SLEW_LIMIT,
        restLengthSlewLimit = 0.5,
        bendChordSlewLimit = 1
    } = {}) {
        const points = this.#buildCenterline();
        const count = Math.min(this._centerlinePointCount, body.count);
        let progressDelta = this.progress - this._xpbdProgress;
        if (this.physicsBody !== body || !this._xpbdLayoutX || this._xpbdLayoutX.length !== body.count) {
            if (this.physicsBody && this.physicsBody !== body) {
                this.#releaseUnsupportedXpbdTip(this.physicsBody);
            }
            this._xpbdLayoutX = new Float64Array(body.count);
            this._xpbdLayoutY = new Float64Array(body.count);
            this._xpbdLayoutZ = new Float64Array(body.count);
            this._xpbdDriveX = new Float32Array(body.count);
            this._xpbdDriveY = new Float32Array(body.count);
            this._xpbdDriveZ = new Float32Array(body.count);
            this._xpbdDriveInitialized = new Uint8Array(body.count);
            this._xpbdLayoutCount = 0;
            this.physicsActiveCount = 0;
            this._xpbdProgress = this.progress;
            progressDelta = 0;
            this._xpbdYieldsToWall = false;
        }
        this.physicsBody = body;
        body.postStabilizationPasses =
            this.type === CATHETER_TYPE_PIGTAIL
                ? PIGTAIL_XPBD_POST_STABILIZATION_PASSES
                : BERENSTEIN_XPBD_POST_STABILIZATION_PASSES;
        if (count < 2) {
            for (let index = 0; index < this.physicsActiveCount; index++) body.clearRestShapeTarget(index);
            body.setActiveRange(0, 1);
            body.setCollisionRange(0, -1);
            this.physicsActiveCount = 0;
            this._xpbdLayoutCount = 0;
            this._xpbdProgress = this.progress;
            this._xpbdYieldsToWall = false;
            this._xpbdDriveInitialized?.fill(0);
            this._pendingXpbdRotation = 0;
            this.#releaseUnsupportedXpbdTip(body);
            return 0;
        }

        const previousCount = this.physicsActiveCount;
        const soloXpbd = this.externalCollisionSolver &&
            this.guidewireInserted <= MIN_GUIDE_SUPPORT;
        let insertedIndex = -1;
        let topologyChanged = false;
        if (
            previousCount > 0 &&
            this._xpbdLayoutCount === previousCount &&
            count === previousCount + 1
        ) {
            insertedIndex = soloXpbd && this.freeNodes.length >= 2
                ? this.#xpbdUnsupportedEntryIndex(count)
                : this.freeNodes.length >= 2
                    ? count - 1
                    : this.#xpbdInsertedPointIndex(points, count, previousCount);
            for (let index = count - 1; index > insertedIndex; index--) {
                this.#copyXpbdNodeState(body, index, index - 1);
            }
            this.#initializeInsertedXpbdNode(body, points, insertedIndex, count, shapeCompliance);
            const stabilizeUnsupportedEntry =
                this.type === CATHETER_TYPE_BERENSTEIN ||
                this.progress >= DISTAL_RELEASE_LENGTH;
            if (
                soloXpbd &&
                stabilizeUnsupportedEntry &&
                insertedIndex + 1 < count
            ) {
                this.#initializeUnsupportedEntryPair(
                    body,
                    points,
                    insertedIndex,
                    shapeCompliance
                );
            }
            topologyChanged = true;
        } else if (
            count > 1 &&
            this._xpbdLayoutCount === previousCount &&
            count === previousCount - 1
        ) {
            const removedIndex = soloXpbd && this.freeNodes.length >= 2
                ? this.#xpbdUnsupportedEntryIndex(count)
                : this.freeNodes.length >= 2
                    ? previousCount - 1
                    : this.#xpbdRemovedPointIndex(points, count, previousCount);
            for (let index = removedIndex; index < count; index++) {
                this.#copyXpbdNodeState(body, index, index + 1);
            }
            topologyChanged = true;
        }
        body.setActiveRange(0, count - 1);
        if (
            topologyChanged ||
            Math.abs(this.motionCommand) > 0 ||
            Math.abs(this.guidewireDelta) > 1e-5
        ) {
            body.wake();
        }
        if (topologyChanged) {
            if (soloXpbd) this.#resetXpbdWallContacts(body);
            for (let index = 0; index < count - 1; index++) {
                body.restLength[index] = Math.max(0.5, magnitude3(
                    body.x[index + 1] - body.x[index],
                    body.y[index + 1] - body.y[index],
                    body.z[index + 1] - body.z[index]
                ));
            }
            for (let index = 1; index < count - 1; index++) {
                body.restBendChord[index] = magnitude3(
                    body.x[index + 1] - body.x[index - 1],
                    body.y[index + 1] - body.y[index - 1],
                    body.z[index + 1] - body.z[index - 1]
                );
            }
        }
        let collisionStart = count - 1;
        const sheath = this.vessel?.sheath;
        if (sheath) {
            const axisX = sheath.end.x - sheath.start.x;
            const axisY = sheath.end.y - sheath.start.y;
            const axisZ = sheath.end.z - sheath.start.z;
            const length = magnitude3(axisX, axisY, axisZ) || 1;
            const directionX = axisX / length;
            const directionY = axisY / length;
            const directionZ = axisZ / length;
            for (let index = 0; index < count; index++) {
                const point = points[index];
                const axial =
                    (point.x - sheath.start.x) * directionX +
                    (point.y - sheath.start.y) * directionY +
                    (point.z - sheath.start.z) * directionZ;
                if (axial > length + 0.25) {
                    collisionStart = Math.max(0, index - 1);
                    break;
                }
            }
        }
        this.#applyPendingBerensteinRotation(body, count);
        if (soloXpbd && previousCount > 0 && progressDelta > 0) {
            this.#advanceUnsupportedXpbdBody(
                body,
                collisionStart,
                count,
                Math.min(1, progressDelta)
            );
        }
        let bodyTouchesWall = false;
        for (
            let segment = Math.max(0, collisionStart);
            segment < Math.min(body.segmentCount, count - 1);
            segment++
        ) {
            if (!body.wallActive[segment]) continue;
            bodyTouchesWall = true;
            break;
        }
        const enteredWallYield = bodyTouchesWall && !this._xpbdYieldsToWall;
        if (bodyTouchesWall) this._xpbdYieldsToWall = true;
        for (let index = 0; index < count; index++) {
            const point = points[index];
            const insertedDistance = this._centerlineDistances[index] ?? Infinity;
            const shapeWeight = this.#xpbdShapeMemoryWeight(insertedDistance);
            const softTipWeight = this.#xpbdSoftTipWeight(insertedDistance);
            const guideReleaseStability = this.guidewireInserted > MIN_GUIDE_SUPPORT
                ? 1 - smoothstep(
                    this.guidewireInserted + GUIDE_CAPTURE_TOLERANCE,
                    this.guidewireInserted + XPBD_RELEASE_STABILITY_LENGTH,
                    insertedDistance
                )
                : 0;
            const yieldsToWall = this._xpbdYieldsToWall;
            const stabilizeFixedPath = this.externalCollisionSolver && index <= collisionStart;
            const stabilizeFreeShaft = this.externalCollisionSolver && index > collisionStart;
            const newlyActivated = index === insertedIndex || (
                insertedIndex < 0 && index >= previousCount
            );
            if (newlyActivated && index !== insertedIndex) {
                if (previousCount > 0 && index > 0) {
                    const targetPrevious = points[index - 1];
                    let directionX = point.x - targetPrevious.x;
                    let directionY = point.y - targetPrevious.y;
                    let directionZ = point.z - targetPrevious.z;
                    let targetLength = magnitude3(directionX, directionY, directionZ);
                    if (targetLength < 1e-6 && index > 1) {
                        directionX = body.x[index - 1] - body.x[index - 2];
                        directionY = body.y[index - 1] - body.y[index - 2];
                        directionZ = body.z[index - 1] - body.z[index - 2];
                        targetLength = magnitude3(directionX, directionY, directionZ);
                    }
                    const restLength = Math.max(0.5, point.distanceTo(targetPrevious));
                    const inverseDirectionLength = 1 / Math.max(1e-6, targetLength);
                    body.setNodePosition(
                        index,
                        body.x[index - 1] + directionX * inverseDirectionLength * restLength,
                        body.y[index - 1] + directionY * inverseDirectionLength * restLength,
                        body.z[index - 1] + directionZ * inverseDirectionLength * restLength
                    );
                } else {
                    body.setNodePosition(index, point.x, point.y, point.z);
                }
            }
            const hasShapeMemory = shapeWeight > XPBD_MIN_SHAPE_WEIGHT;
            if (
                stabilizeFixedPath ||
                hasShapeMemory ||
                (
                    stabilizeFreeShaft &&
                    (!yieldsToWall || soloXpbd)
                )
            ) {
                const targetWasEnabled = body.restShapeEnabled[index] === 1;
                const holdBerensteinTwist =
                    this.type === CATHETER_TYPE_BERENSTEIN &&
                    this._xpbdBerensteinTwisted &&
                    hasShapeMemory &&
                    !stabilizeFixedPath &&
                    targetWasEnabled;
                let targetX = (
                    holdBerensteinTwist ||
                    (yieldsToWall && !stabilizeFixedPath && targetWasEnabled)
                )
                    ? body.restShapeX[index]
                    : point.x;
                let targetY = (
                    holdBerensteinTwist ||
                    (yieldsToWall && !stabilizeFixedPath && targetWasEnabled)
                )
                    ? body.restShapeY[index]
                    : point.y;
                let targetZ = (
                    holdBerensteinTwist ||
                    (yieldsToWall && !stabilizeFixedPath && targetWasEnabled)
                )
                    ? body.restShapeZ[index]
                    : point.z;
                if (
                    previousCount > 0 &&
                    !stabilizeFixedPath &&
                    (newlyActivated || !targetWasEnabled || enteredWallYield)
                ) {
                    // Shape memory must engage from the physical pose. Snapping a
                    // newly released node directly to its ideal world-space
                    // target creates the wall-versus-target impulse seen as
                    // chaotic pigtail jumping.
                    targetX = body.x[index];
                    targetY = body.y[index];
                    targetZ = body.z[index];
                } else if (
                    (
                        !holdBerensteinTwist &&
                        (
                            !yieldsToWall ||
                            stabilizeFixedPath ||
                            (
                                hasShapeMemory &&
                                (
                                    soloXpbd ||
                                    (
                                        this.type === CATHETER_TYPE_BERENSTEIN &&
                                        Math.abs(this.rotationCommand) > 0
                                    )
                                ) &&
                                softTipWeight > XPBD_MIN_SHAPE_WEIGHT
                            )
                        )
                    ) &&
                    previousCount > 0 &&
                    targetWasEnabled &&
                    Number.isFinite(targetSlewLimit) &&
                    targetSlewLimit > 0
                ) {
                    const dx = point.x - body.restShapeX[index];
                    const dy = point.y - body.restShapeY[index];
                    const dz = point.z - body.restShapeZ[index];
                    const distance = magnitude3(dx, dy, dz);
                    const weightedSlewLimit = hasShapeMemory
                        ? targetSlewLimit *
                            (0.35 + shapeWeight * 0.65) *
                            (yieldsToWall ? 0.3 : 1)
                        : Math.max(1, targetSlewLimit);
                    if (distance > weightedSlewLimit) {
                        const scale = weightedSlewLimit / distance;
                        targetX = body.restShapeX[index] + dx * scale;
                        targetY = body.restShapeY[index] + dy * scale;
                        targetZ = body.restShapeZ[index] + dz * scale;
                    }
                }
                if (
                    soloXpbd &&
                    yieldsToWall &&
                    !stabilizeFixedPath
                ) {
                    const dx = targetX - body.x[index];
                    const dy = targetY - body.y[index];
                    const dz = targetZ - body.z[index];
                    const distance = magnitude3(dx, dy, dz);
                    const maximumTargetOffset =
                        this.type === CATHETER_TYPE_BERENSTEIN
                            ? BERENSTEIN_XPBD_TARGET_MAX_OFFSET
                            : PIGTAIL_XPBD_TARGET_MAX_OFFSET;
                    if (distance > maximumTargetOffset) {
                        const scale = maximumTargetOffset / distance;
                        targetX = body.x[index] + dx * scale;
                        targetY = body.y[index] + dy * scale;
                        targetZ = body.z[index] + dz * scale;
                    }
                }
                let effectiveShapeCompliance;
                if (!yieldsToWall || stabilizeFixedPath) {
                    effectiveShapeCompliance = XPBD_FREE_PATH_COMPLIANCE;
                } else if (hasShapeMemory) {
                    effectiveShapeCompliance = shapeCompliance / Math.max(0.25, shapeWeight);
                } else {
                    effectiveShapeCompliance = XPBD_CONTACT_DRIVE_COMPLIANCE;
                }
                if (holdBerensteinTwist) {
                    effectiveShapeCompliance = Math.min(
                        effectiveShapeCompliance,
                        BERENSTEIN_XPBD_ROTATION_SHAPE_COMPLIANCE
                    );
                }
                effectiveShapeCompliance = XPBD_FREE_PATH_COMPLIANCE +
                    (effectiveShapeCompliance - XPBD_FREE_PATH_COMPLIANCE) *
                    (1 - guideReleaseStability);
                body.setRestShapeTarget(
                    index,
                    targetX,
                    targetY,
                    targetZ,
                    effectiveShapeCompliance
                );
            } else {
                body.clearRestShapeTarget(index);
            }
            body.nodeRadius[index] = CATHETER_RADIUS;
            const unsupportedStiffness = 1 - smoothstep(
                0,
                MIN_GUIDE_SUPPORT,
                this.guidewireInserted
            );
            const shaftBendCompliance = body.bendCompliance +
                (
                    Math.min(body.bendCompliance, SOLO_XPBD_BEND_COMPLIANCE) -
                    body.bendCompliance
                ) * unsupportedStiffness;
            body.bendComplianceByNode[index] = shaftBendCompliance +
                (XPBD_SOFT_TIP_BEND_COMPLIANCE - shaftBendCompliance) *
                softTipWeight;
            const shaftMaxBendAngle = body.maxBendAngle +
                (
                    Math.min(body.maxBendAngle, SOLO_XPBD_SHAFT_MAX_BEND_ANGLE) -
                    body.maxBendAngle
                ) * unsupportedStiffness;
            const softTipMaxBendAngle = this.type === CATHETER_TYPE_BERENSTEIN
                ? BERENSTEIN_XPBD_SOFT_TIP_MAX_BEND_ANGLE
                : body.maxBendAngle;
            body.maxBendAngleByNode[index] = shaftMaxBendAngle +
                (softTipMaxBendAngle - shaftMaxBendAngle) * softTipWeight;
            if (index > 0) {
                const previous = points[index - 1];
                const desiredLength = Math.max(0.5, point.distanceTo(previous));
                if (previousCount > 0 && restLengthSlewLimit > 0) {
                    body.restLength[index - 1] += clamp(
                        desiredLength - body.restLength[index - 1],
                        -restLengthSlewLimit,
                        restLengthSlewLimit
                    );
                } else {
                    body.restLength[index - 1] = desiredLength;
                }
            }
            if (index > 0 && index < count - 1) {
                const curvedChord = points[index - 1].distanceTo(points[index + 1]);
                const straightChord =
                    points[index - 1].distanceTo(point) + point.distanceTo(points[index + 1]);
                // The unsupported shaft stays straight and stiff, while the
                // distal preformed section retains its natural curvature.
                const settleAgainstWall = yieldsToWall &&
                    previousCount > 0 &&
                    softTipWeight > XPBD_MIN_SHAPE_WEIGHT;
                const desiredChord = settleAgainstWall
                    ? magnitude3(
                        body.x[index + 1] - body.x[index - 1],
                        body.y[index + 1] - body.y[index - 1],
                        body.z[index + 1] - body.z[index - 1]
                    )
                    : soloXpbd
                        ? straightChord + (curvedChord - straightChord) * shapeWeight
                        : curvedChord;
                if (previousCount > 0 && bendChordSlewLimit > 0) {
                    body.restBendChord[index] += clamp(
                        desiredChord - body.restBendChord[index],
                        -bendChordSlewLimit,
                        bendChordSlewLimit
                    );
                } else {
                    body.restBendChord[index] = desiredChord;
                }
            }
        }
        for (let index = count; index < previousCount; index++) body.clearRestShapeTarget(index);
        this.#stabilizeUnsupportedXpbdTip(body, count, soloXpbd, progressDelta);
        body.setCollisionRange(collisionStart, count - 2);
        this.physicsActiveCount = count;
        for (let index = 0; index < count; index++) {
            this._xpbdLayoutX[index] = points[index].x;
            this._xpbdLayoutY[index] = points[index].y;
            this._xpbdLayoutZ[index] = points[index].z;
        }
        this._xpbdLayoutCount = count;
        this._xpbdProgress = this.progress;
        return count;
    }

    #applyPendingBerensteinRotation(body, count) {
        const angle = this._pendingXpbdRotation;
        this._pendingXpbdRotation = 0;
        if (
            this.type !== CATHETER_TYPE_BERENSTEIN ||
            Math.abs(angle) < 1e-6 ||
            count < 4
        ) {
            return;
        }

        const shapeStart = Math.max(
            this.#sheathSupportEnd(),
            this.guidewireInserted,
            this.progress - BERENSTEIN_TIP_SHAPE_LENGTH
        );
        let baseIndex = Math.max(1, count - 1);
        for (let index = 1; index < count; index++) {
            if ((this._centerlineDistances[index] ?? -Infinity) < shapeStart) continue;
            baseIndex = index;
            break;
        }
        if (baseIndex >= count - 1) return;

        const tangentStart = Math.max(0, baseIndex - 2);
        const tangentEnd = Math.min(count - 1, baseIndex + 1);
        let axisX = body.x[tangentEnd] - body.x[tangentStart];
        let axisY = body.y[tangentEnd] - body.y[tangentStart];
        let axisZ = body.z[tangentEnd] - body.z[tangentStart];
        const axisLength = magnitude3(axisX, axisY, axisZ);
        if (axisLength < 1e-6) return;
        axisX /= axisLength;
        axisY /= axisLength;
        axisZ /= axisLength;
        // Switch from the cumulative material-frame angle to physically
        // rotated XPBD state only when a released distal segment really
        // exists. Marking the catheter as twisted while the guidewire still
        // splints the complete Berenstein tip discards Q/E torque before that
        // tip is exposed.
        this._xpbdBerensteinTwisted = true;
        const cosine = Math.cos(angle);
        const sine = Math.sin(angle);
        const oneMinusCosine = 1 - cosine;
        const anchorX = body.x[baseIndex];
        const anchorY = body.y[baseIndex];
        const anchorZ = body.z[baseIndex];
        const previousAnchorX = body.previousX[baseIndex];
        const previousAnchorY = body.previousY[baseIndex];
        const previousAnchorZ = body.previousZ[baseIndex];
        const targetAnchorX = body.restShapeEnabled[baseIndex]
            ? body.restShapeX[baseIndex]
            : anchorX;
        const targetAnchorY = body.restShapeEnabled[baseIndex]
            ? body.restShapeY[baseIndex]
            : anchorY;
        const targetAnchorZ = body.restShapeEnabled[baseIndex]
            ? body.restShapeZ[baseIndex]
            : anchorZ;
        if (this._xpbdSoloTipTargetActive) {
            const controlRelativeX = this._xpbdSoloTipTarget.x - anchorX;
            const controlRelativeY = this._xpbdSoloTipTarget.y - anchorY;
            const controlRelativeZ = this._xpbdSoloTipTarget.z - anchorZ;
            const controlDot =
                controlRelativeX * axisX +
                controlRelativeY * axisY +
                controlRelativeZ * axisZ;
            this._xpbdSoloTipTarget.set(
                anchorX +
                    controlRelativeX * cosine +
                    (axisY * controlRelativeZ - axisZ * controlRelativeY) * sine +
                    axisX * controlDot * oneMinusCosine,
                anchorY +
                    controlRelativeY * cosine +
                    (axisZ * controlRelativeX - axisX * controlRelativeZ) * sine +
                    axisY * controlDot * oneMinusCosine,
                anchorZ +
                    controlRelativeZ * cosine +
                    (axisX * controlRelativeY - axisY * controlRelativeX) * sine +
                    axisZ * controlDot * oneMinusCosine
            );
        }
        for (let index = baseIndex + 1; index < count; index++) {
            const relativeX = body.x[index] - anchorX;
            const relativeY = body.y[index] - anchorY;
            const relativeZ = body.z[index] - anchorZ;
            const dot =
                relativeX * axisX + relativeY * axisY + relativeZ * axisZ;
            body.x[index] =
                anchorX +
                relativeX * cosine +
                (axisY * relativeZ - axisZ * relativeY) * sine +
                axisX * dot * oneMinusCosine;
            body.y[index] =
                anchorY +
                relativeY * cosine +
                (axisZ * relativeX - axisX * relativeZ) * sine +
                axisY * dot * oneMinusCosine;
            body.z[index] =
                anchorZ +
                relativeZ * cosine +
                (axisX * relativeY - axisY * relativeX) * sine +
                axisZ * dot * oneMinusCosine;

            const previousRelativeX = body.previousX[index] - previousAnchorX;
            const previousRelativeY = body.previousY[index] - previousAnchorY;
            const previousRelativeZ = body.previousZ[index] - previousAnchorZ;
            const previousDot =
                previousRelativeX * axisX +
                previousRelativeY * axisY +
                previousRelativeZ * axisZ;
            body.previousX[index] =
                previousAnchorX +
                previousRelativeX * cosine +
                (axisY * previousRelativeZ - axisZ * previousRelativeY) * sine +
                axisX * previousDot * oneMinusCosine;
            body.previousY[index] =
                previousAnchorY +
                previousRelativeY * cosine +
                (axisZ * previousRelativeX - axisX * previousRelativeZ) * sine +
                axisY * previousDot * oneMinusCosine;
            body.previousZ[index] =
                previousAnchorZ +
                previousRelativeZ * cosine +
                (axisX * previousRelativeY - axisY * previousRelativeX) * sine +
                axisZ * previousDot * oneMinusCosine;

            if (body.restShapeEnabled[index]) {
                const targetRelativeX =
                    body.restShapeX[index] - targetAnchorX;
                const targetRelativeY =
                    body.restShapeY[index] - targetAnchorY;
                const targetRelativeZ =
                    body.restShapeZ[index] - targetAnchorZ;
                const targetDot =
                    targetRelativeX * axisX +
                    targetRelativeY * axisY +
                    targetRelativeZ * axisZ;
                body.restShapeX[index] =
                    targetAnchorX +
                    targetRelativeX * cosine +
                    (axisY * targetRelativeZ - axisZ * targetRelativeY) * sine +
                    axisX * targetDot * oneMinusCosine;
                body.restShapeY[index] =
                    targetAnchorY +
                    targetRelativeY * cosine +
                    (axisZ * targetRelativeX - axisX * targetRelativeZ) * sine +
                    axisY * targetDot * oneMinusCosine;
                body.restShapeZ[index] =
                    targetAnchorZ +
                    targetRelativeZ * cosine +
                    (axisX * targetRelativeY - axisY * targetRelativeX) * sine +
                    axisZ * targetDot * oneMinusCosine;
            }

            const velocityX = body.velocityX[index];
            const velocityY = body.velocityY[index];
            const velocityZ = body.velocityZ[index];
            const velocityDot =
                velocityX * axisX + velocityY * axisY + velocityZ * axisZ;
            body.velocityX[index] =
                velocityX * cosine +
                (axisY * velocityZ - axisZ * velocityY) * sine +
                axisX * velocityDot * oneMinusCosine;
            body.velocityY[index] =
                velocityY * cosine +
                (axisZ * velocityX - axisX * velocityZ) * sine +
                axisY * velocityDot * oneMinusCosine;
            body.velocityZ[index] =
                velocityZ * cosine +
                (axisX * velocityY - axisY * velocityX) * sine +
                axisZ * velocityDot * oneMinusCosine;
            body.shapeLambda[index] = 0;
            body.controlLambda[index] = 0;
        }
        body.wake();
    }

    #releaseUnsupportedXpbdTip(body = this.physicsBody) {
        if (body && this._xpbdSoloTipControlIndex >= 0) {
            body.clearControlTarget(this._xpbdSoloTipControlIndex);
            body.setPinned(this._xpbdSoloTipControlIndex, false);
        }
        this._xpbdSoloTipTargetActive = false;
        this._xpbdSoloTipControlIndex = -1;
    }

    #stabilizeUnsupportedXpbdTip(body, count, soloXpbd, progressDelta) {
        const tipIndex = count - 1;
        const controlIndex = tipIndex;
        if (
            this._xpbdSoloTipControlIndex >= 0 &&
            this._xpbdSoloTipControlIndex !== controlIndex
        ) {
            body.clearControlTarget(this._xpbdSoloTipControlIndex);
            body.setPinned(this._xpbdSoloTipControlIndex, false);
        }
        if (!soloXpbd) {
            if (this._xpbdSoloTipControlIndex >= 0) {
                body.clearControlTarget(this._xpbdSoloTipControlIndex);
                body.setPinned(this._xpbdSoloTipControlIndex, false);
            }
            this._xpbdSoloTipTargetActive = false;
            this._xpbdSoloTipControlIndex = -1;
            return;
        }
        if (!this._xpbdSoloTipTargetActive) {
            this._xpbdSoloTipTarget.set(
                body.x[controlIndex],
                body.y[controlIndex],
                body.z[controlIndex]
            );
            this._xpbdSoloTipTargetActive = true;
        }
        if (progressDelta > 0 && controlIndex > 0) {
            const direction = this.#xpbdInsertionDirection(
                body,
                controlIndex,
                body.x[controlIndex] - body.x[controlIndex - 1],
                body.y[controlIndex] - body.y[controlIndex - 1],
                body.z[controlIndex] - body.z[controlIndex - 1]
            );
            const directionLength = direction.length() || 1;
            this._xpbdSoloTipTarget.addScaledVector(
                direction,
                Math.min(0.75, progressDelta) / directionLength
            );
        }
        const dx = this._xpbdSoloTipTarget.x - body.x[controlIndex];
        const dy = this._xpbdSoloTipTarget.y - body.y[controlIndex];
        const dz = this._xpbdSoloTipTarget.z - body.z[controlIndex];
        const distance = magnitude3(dx, dy, dz);
        if (distance > 0.6) {
            const scale = 0.6 / distance;
            this._xpbdSoloTipTarget.set(
                body.x[controlIndex] + dx * scale,
                body.y[controlIndex] + dy * scale,
                body.z[controlIndex] + dz * scale
            );
        }
        // The bend constraints on the preceding soft section already recover
        // the distal profile. Applying a second world-space shape spring to
        // the same endpoint makes it fight the insertion control and flip
        // across the lumen.
        body.clearRestShapeTarget(controlIndex);
        body.setPinned(controlIndex, false);
        body.setControlTarget(
            controlIndex,
            this._xpbdSoloTipTarget.x,
            this._xpbdSoloTipTarget.y,
            this._xpbdSoloTipTarget.z,
            XPBD_TIP_DRIVE_COMPLIANCE
        );
        this._xpbdSoloTipControlIndex = controlIndex;
    }

    #resetXpbdWallContacts(body) {
        body.wallLambda.fill(0);
        body.wallActive.fill(0);
        body.wallT.fill(0);
        body.wallX.fill(0);
        body.wallY.fill(0);
        body.wallZ.fill(0);
        body.wallNormalX.fill(0);
        body.wallNormalY.fill(0);
        body.wallNormalZ.fill(0);
        body.wallBranchId.fill(-1);
        body.wallGap.fill(Infinity);
        body.wallQueryStartX.fill(0);
        body.wallQueryStartY.fill(0);
        body.wallQueryStartZ.fill(0);
        body.wallQueryEndX.fill(0);
        body.wallQueryEndY.fill(0);
        body.wallQueryEndZ.fill(0);
    }

    #xpbdUnsupportedEntryIndex(count) {
        const supportEnd = this.#sheathSupportEnd();
        for (let index = 1; index < count; index++) {
            if ((this._centerlineDistances[index] ?? -Infinity) >= supportEnd - 0.25) {
                return index;
            }
        }
        return Math.max(1, count - 1);
    }

    #initializeUnsupportedEntryPair(body, points, insertedIndex, shapeCompliance) {
        const entry = points[insertedIndex];
        const distal = points[insertedIndex + 1];
        body.setNodePosition(insertedIndex, entry.x, entry.y, entry.z);
        body.setNodePosition(insertedIndex + 1, distal.x, distal.y, distal.z);
        body.clearRestShapeTarget(insertedIndex);
        body.setRestShapeTarget(
            insertedIndex + 1,
            distal.x,
            distal.y,
            distal.z,
            shapeCompliance
        );
        this._xpbdDriveInitialized[insertedIndex] = 0;
        this._xpbdDriveInitialized[insertedIndex + 1] = 0;
    }

    #advanceUnsupportedXpbdBody(body, collisionStart, count, distance) {
        const start = Math.max(1, collisionStart + 1);
        const curvedTipLength = this.type === CATHETER_TYPE_BERENSTEIN
            ? BERENSTEIN_STRAIGHT_EXIT_LENGTH + BERENSTEIN_BEND_LENGTH
            : PIGTAIL_ARC_LENGTH;
        const tipShapeStart = Math.max(
            this.#sheathSupportEnd(),
            this.progress - curvedTipLength
        );
        let tipBaseIndex = count;
        for (let index = start; index < count; index++) {
            if ((this._centerlineDistances[index] ?? Infinity) < tipShapeStart) continue;
            tipBaseIndex = index;
            break;
        }
        const tipTangentStart = Math.max(start, tipBaseIndex - 2);
        const tipTangentEnd = Math.min(count - 1, tipBaseIndex + 1);
        let tipDirectionX = body.x[tipTangentEnd] - body.x[tipTangentStart];
        let tipDirectionY = body.y[tipTangentEnd] - body.y[tipTangentStart];
        let tipDirectionZ = body.z[tipTangentEnd] - body.z[tipTangentStart];
        const centerlineTipDirection = this.#xpbdInsertionDirection(
            body,
            tipBaseIndex,
            tipDirectionX,
            tipDirectionY,
            tipDirectionZ
        );
        tipDirectionX = centerlineTipDirection.x;
        tipDirectionY = centerlineTipDirection.y;
        tipDirectionZ = centerlineTipDirection.z;
        const tipDirectionLength = magnitude3(
            tipDirectionX,
            tipDirectionY,
            tipDirectionZ
        ) || 1;
        for (let index = count - 1; index >= start; index--) {
            const previous = index - 1;
            let directionX;
            let directionY;
            let directionZ;
            let directionLength;
            if (index >= tipBaseIndex) {
                directionX = tipDirectionX;
                directionY = tipDirectionY;
                directionZ = tipDirectionZ;
                directionLength = tipDirectionLength;
            } else if (index + 1 < count) {
                directionX = body.x[index + 1] - body.x[previous];
                directionY = body.y[index + 1] - body.y[previous];
                directionZ = body.z[index + 1] - body.z[previous];
                const centerlineDirection = this.#xpbdInsertionDirection(
                    body,
                    index,
                    directionX,
                    directionY,
                    directionZ
                );
                directionX = centerlineDirection.x;
                directionY = centerlineDirection.y;
                directionZ = centerlineDirection.z;
                directionLength = magnitude3(directionX, directionY, directionZ);
            } else {
                directionX = body.x[index] - body.x[previous];
                directionY = body.y[index] - body.y[previous];
                directionZ = body.z[index] - body.z[previous];
                directionLength = magnitude3(directionX, directionY, directionZ);
            }
            if (directionLength < 1e-6) continue;
            const scale = distance / directionLength;
            const dx = directionX * scale;
            const dy = directionY * scale;
            const dz = directionZ * scale;
            if (!body.restShapeEnabled[index]) continue;
            body.restShapeX[index] += dx;
            body.restShapeY[index] += dy;
            body.restShapeZ[index] += dz;
        }
    }

    #xpbdInsertionDirection(body, index, fallbackX, fallbackY, fallbackZ) {
        const direction = this._xpbdDriveDirection;
        const driveIndex = Math.max(0, Math.min(body.count - 1, index));
        let fallbackLength = magnitude3(fallbackX, fallbackY, fallbackZ);
        if (fallbackLength < 1e-6) {
            if (this._xpbdDriveInitialized?.[driveIndex]) {
                fallbackX = this._xpbdDriveX[driveIndex];
                fallbackY = this._xpbdDriveY[driveIndex];
                fallbackZ = this._xpbdDriveZ[driveIndex];
                fallbackLength = 1;
            } else {
                fallbackX = 1;
                fallbackY = 0;
                fallbackZ = 0;
                fallbackLength = 1;
            }
        }
        fallbackX /= fallbackLength;
        fallbackY /= fallbackLength;
        fallbackZ /= fallbackLength;
        direction.set(fallbackX, fallbackY, fallbackZ);
        let hasCenterlineDirection = false;
        const field = body.contactField;
        if (typeof field?.getCenterlineTangent === 'function') {
            const tangent = field.getCenterlineTangent(
                body.x[driveIndex],
                body.y[driveIndex],
                body.z[driveIndex]
            );
            if (
                tangent &&
                Number.isFinite(tangent.x) &&
                Number.isFinite(tangent.y) &&
                Number.isFinite(tangent.z)
            ) {
                direction.set(tangent.x, tangent.y, tangent.z);
                hasCenterlineDirection = direction.lengthSq() >= 1e-8;
            }
        } else if (field?.centerline && field.centerlineStride >= 6) {
            const centerline = field.centerline;
            const stride = field.centerlineStride;
            const segmentCount = Math.floor(centerline.length / stride);
            let branchId = -1;
            for (let offset = 0; offset <= 2 && branchId < 0; offset++) {
                const right = Math.min(body.segmentCount - 1, driveIndex + offset);
                const left = Math.max(0, driveIndex - 1 - offset);
                if (body.wallActive[right] && body.wallBranchId[right] >= 0) {
                    branchId = body.wallBranchId[right];
                } else if (body.wallActive[left] && body.wallBranchId[left] >= 0) {
                    branchId = body.wallBranchId[left];
                }
            }
            if (branchId < 0) {
                const segment = Math.max(0, Math.min(body.segmentCount - 1, driveIndex - 1));
                branchId = body.wallBranchId[segment];
            }
            if (branchId >= 0 && branchId < segmentCount) {
                const centerlineOffset = branchId * stride;
                direction.set(
                    centerline[centerlineOffset + 3] - centerline[centerlineOffset],
                    centerline[centerlineOffset + 4] - centerline[centerlineOffset + 1],
                    centerline[centerlineOffset + 5] - centerline[centerlineOffset + 2]
                );
                hasCenterlineDirection = direction.lengthSq() >= 1e-8;
            }
        }
        if (hasCenterlineDirection) {
            direction.normalize();
            if (
                direction.x * fallbackX +
                direction.y * fallbackY +
                direction.z * fallbackZ < 0
            ) {
                direction.multiplyScalar(-1);
            }
            direction.set(
                fallbackX + (direction.x - fallbackX) * XPBD_CENTERLINE_DIRECTION_BLEND,
                fallbackY + (direction.y - fallbackY) * XPBD_CENTERLINE_DIRECTION_BLEND,
                fallbackZ + (direction.z - fallbackZ) * XPBD_CENTERLINE_DIRECTION_BLEND
            ).normalize();
        } else {
            direction.set(fallbackX, fallbackY, fallbackZ);
        }
        const previousIndex = driveIndex - 1;
        if (previousIndex >= 0 && this._xpbdDriveInitialized?.[previousIndex]) {
            let previousX = this._xpbdDriveX[previousIndex];
            let previousY = this._xpbdDriveY[previousIndex];
            let previousZ = this._xpbdDriveZ[previousIndex];
            if (
                direction.x * previousX +
                direction.y * previousY +
                direction.z * previousZ < 0
            ) {
                previousX *= -1;
                previousY *= -1;
                previousZ *= -1;
            }
            direction.set(
                direction.x + (previousX - direction.x) * XPBD_DIRECTION_SPATIAL_BLEND,
                direction.y + (previousY - direction.y) * XPBD_DIRECTION_SPATIAL_BLEND,
                direction.z + (previousZ - direction.z) * XPBD_DIRECTION_SPATIAL_BLEND
            ).normalize();
        }
        if (this._xpbdDriveInitialized?.[driveIndex]) {
            let storedX = this._xpbdDriveX[driveIndex];
            let storedY = this._xpbdDriveY[driveIndex];
            let storedZ = this._xpbdDriveZ[driveIndex];
            if (
                direction.x * storedX +
                direction.y * storedY +
                direction.z * storedZ < 0
            ) {
                storedX *= -1;
                storedY *= -1;
                storedZ *= -1;
            }
            direction.set(
                storedX + (direction.x - storedX) * XPBD_DIRECTION_TEMPORAL_BLEND,
                storedY + (direction.y - storedY) * XPBD_DIRECTION_TEMPORAL_BLEND,
                storedZ + (direction.z - storedZ) * XPBD_DIRECTION_TEMPORAL_BLEND
            ).normalize();
        }
        this._xpbdDriveX[driveIndex] = direction.x;
        this._xpbdDriveY[driveIndex] = direction.y;
        this._xpbdDriveZ[driveIndex] = direction.z;
        this._xpbdDriveInitialized[driveIndex] = 1;
        return direction;
    }

    #xpbdInsertedPointIndex(points, count, previousCount) {
        let bestIndex = count - 1;
        let bestScore = Infinity;
        for (let inserted = 0; inserted < count; inserted++) {
            let score = 0;
            for (let oldIndex = 0; oldIndex < previousCount; oldIndex++) {
                const nextIndex = oldIndex < inserted ? oldIndex : oldIndex + 1;
                const point = points[nextIndex];
                const dx = point.x - this._xpbdLayoutX[oldIndex];
                const dy = point.y - this._xpbdLayoutY[oldIndex];
                const dz = point.z - this._xpbdLayoutZ[oldIndex];
                score += dx * dx + dy * dy + dz * dz;
            }
            if (score < bestScore) {
                bestScore = score;
                bestIndex = inserted;
            }
        }
        return bestIndex;
    }

    #xpbdRemovedPointIndex(points, count, previousCount) {
        let bestIndex = previousCount - 1;
        let bestScore = Infinity;
        for (let removed = 0; removed < previousCount; removed++) {
            let score = 0;
            for (let nextIndex = 0; nextIndex < count; nextIndex++) {
                const oldIndex = nextIndex < removed ? nextIndex : nextIndex + 1;
                const point = points[nextIndex];
                const dx = point.x - this._xpbdLayoutX[oldIndex];
                const dy = point.y - this._xpbdLayoutY[oldIndex];
                const dz = point.z - this._xpbdLayoutZ[oldIndex];
                score += dx * dx + dy * dy + dz * dz;
            }
            if (score < bestScore) {
                bestScore = score;
                bestIndex = removed;
            }
        }
        return bestIndex;
    }

    #copyXpbdNodeState(body, target, source) {
        body.x[target] = body.x[source];
        body.y[target] = body.y[source];
        body.z[target] = body.z[source];
        body.previousX[target] = body.previousX[source];
        body.previousY[target] = body.previousY[source];
        body.previousZ[target] = body.previousZ[source];
        body.velocityX[target] = body.velocityX[source];
        body.velocityY[target] = body.velocityY[source];
        body.velocityZ[target] = body.velocityZ[source];
        body.inverseMass[target] = body.inverseMass[source];
        body.nodeRadius[target] = body.nodeRadius[source];
        body.pinned[target] = body.pinned[source];
        body.bendComplianceByNode[target] = body.bendComplianceByNode[source];
        body.restShapeEnabled[target] = body.restShapeEnabled[source];
        body.restShapeX[target] = body.restShapeX[source];
        body.restShapeY[target] = body.restShapeY[source];
        body.restShapeZ[target] = body.restShapeZ[source];
        body.restShapeCompliance[target] = body.restShapeCompliance[source];
        this._xpbdDriveX[target] = this._xpbdDriveX[source];
        this._xpbdDriveY[target] = this._xpbdDriveY[source];
        this._xpbdDriveZ[target] = this._xpbdDriveZ[source];
        this._xpbdDriveInitialized[target] = this._xpbdDriveInitialized[source];
    }

    #initializeInsertedXpbdNode(body, points, index, count, shapeCompliance) {
        if (index > 0 && index + 1 < count) {
            const point = points[index];
            const leftPoint = points[index - 1];
            const rightPoint = points[index + 1];
            const leftDistance = point.distanceTo(leftPoint);
            const rightDistance = point.distanceTo(rightPoint);
            const t = leftDistance / Math.max(1e-6, leftDistance + rightDistance);
            body.x[index] = body.x[index - 1] + (body.x[index + 1] - body.x[index - 1]) * t;
            body.y[index] = body.y[index - 1] + (body.y[index + 1] - body.y[index - 1]) * t;
            body.z[index] = body.z[index - 1] + (body.z[index + 1] - body.z[index - 1]) * t;
            body.previousX[index] = body.previousX[index - 1] +
                (body.previousX[index + 1] - body.previousX[index - 1]) * t;
            body.previousY[index] = body.previousY[index - 1] +
                (body.previousY[index + 1] - body.previousY[index - 1]) * t;
            body.previousZ[index] = body.previousZ[index - 1] +
                (body.previousZ[index + 1] - body.previousZ[index - 1]) * t;
            body.velocityX[index] = body.velocityX[index - 1] +
                (body.velocityX[index + 1] - body.velocityX[index - 1]) * t;
            body.velocityY[index] = body.velocityY[index - 1] +
                (body.velocityY[index + 1] - body.velocityY[index - 1]) * t;
            body.velocityZ[index] = body.velocityZ[index - 1] +
                (body.velocityZ[index + 1] - body.velocityZ[index - 1]) * t;
        } else if (index > 0) {
            const point = points[index];
            const previous = points[index - 1];
            const extrapolatePhysicalTangent = this.externalCollisionSolver &&
                (
                    this.guidewireInserted <= MIN_GUIDE_SUPPORT ||
                    (this._centerlineDistances[index] ?? Infinity) > this.guidewireInserted
                ) &&
                index > 1;
            let directionX = extrapolatePhysicalTangent
                ? body.x[index - 1] - body.x[index - 2]
                : point.x - previous.x;
            let directionY = extrapolatePhysicalTangent
                ? body.y[index - 1] - body.y[index - 2]
                : point.y - previous.y;
            let directionZ = extrapolatePhysicalTangent
                ? body.z[index - 1] - body.z[index - 2]
                : point.z - previous.z;
            let directionLength = magnitude3(directionX, directionY, directionZ);
            if (directionLength < 1e-6 && index > 1) {
                directionX = body.x[index - 1] - body.x[index - 2];
                directionY = body.y[index - 1] - body.y[index - 2];
                directionZ = body.z[index - 1] - body.z[index - 2];
                directionLength = magnitude3(directionX, directionY, directionZ);
            }
            const restLength = Math.max(0.5, point.distanceTo(previous));
            const scale = restLength / Math.max(1e-6, directionLength);
            body.x[index] = body.x[index - 1] + directionX * scale;
            body.y[index] = body.y[index - 1] + directionY * scale;
            body.z[index] = body.z[index - 1] + directionZ * scale;
            body.previousX[index] = body.x[index];
            body.previousY[index] = body.y[index];
            body.previousZ[index] = body.z[index];
            body.velocityX[index] = 0;
            body.velocityY[index] = 0;
            body.velocityZ[index] = 0;
        } else {
            body.setNodePosition(index, points[index].x, points[index].y, points[index].z);
        }
        body.restShapeEnabled[index] = 0;
        body.restShapeX[index] = body.x[index];
        body.restShapeY[index] = body.y[index];
        body.restShapeZ[index] = body.z[index];
        body.restShapeCompliance[index] = shapeCompliance;
        body.shapeLambda[index] = 0;
        this._xpbdDriveInitialized[index] = 0;
    }

    setCollisionGeometry(collision) {
        const geometry = collision?.geometry || collision;
        if (!geometry?.boundsTree) {
            this.collisionMesh = null;
            return;
        }
        this.collisionMesh = {
            geometry,
            meshCollider: collision?.meshCollider || null,
            clearance: Math.max(CATHETER_RADIUS * 0.7, collision?.clearance || 0),
            interiorDirection: collision?.interiorDirection || collision?.collisionInteriorDirection || null
        };
    }

    advance(command, dt, guidewireInserted) {
        this.motionCommand = command;
        this.previousGuidewireInserted = this.guidewireInserted;
        this.guidewireInserted = Math.max(0, guidewireInserted);
        this.guidewireDelta = this.guidewireInserted - this.previousGuidewireInserted;
        const speed = command > 0 ? CATHETER_ADVANCE_SPEED : CATHETER_WITHDRAW_SPEED;
        const nextProgress = clamp(this.progress + command * speed * dt, 0, this.maxLength);
        if (nextProgress > this.progress) {
            this.#recordGuidewirePath(Math.min(nextProgress, this.guidewireInserted));
        } else if (nextProgress < this.progress) {
            this.#trimPath(nextProgress);
        }
        const supportedEnd = Math.min(nextProgress, this.guidewireInserted);
        if ((command !== 0 || this.guidewireDelta > 0) && supportedEnd > MIN_GUIDE_SUPPORT) {
            this.#refreshGuidewirePath(supportedEnd);
        }
        this.progress = nextProgress;
    }

    rotate(command, dt) {
        this.rotationCommand = command;
        if (!command) return;
        const rotationDelta = command * ROTATION_SPEED * dt;
        this.rotation += rotationDelta;
        if (
            this.type === CATHETER_TYPE_BERENSTEIN &&
            this.externalCollisionSolver
        ) {
            this._pendingXpbdRotation += rotationDelta;
        }
    }

    stepPhysics(dt = 1 / 60, { collisions = true } = {}) {
        const state = this.#deploymentState();
        this.#updateGuidewireRelease(dt);
        const stepIndex = this._physicsStepIndex++;
        if (!this.externalCollisionSolver || (stepIndex & 3) === 0) this.#relaxSupportedPath(state.pathEnd);
        if (this.externalCollisionSolver) {
            this.#updateExternalShapeTargets(state, dt);
            return;
        }
        if (state.freeLength < 2 || state.supportEnd <= 0) {
            this.#clearFreeNodes();
            this.freeRestDistanceCount = 0;
            this.freeLength = 0;
            return;
        }

        const frame = this.#freeFrame(state.supportEnd);
        this.#syncFreeNodes(state, frame);
        if (this.freeNodes.length < 2) return;

        this.#recaptureFreeNodes(state);

        const anchor = frame.supportTip;
        for (let i = 0; i < this.freeNodes.length; i++) {
            const node = this.freeNodes[i];
            node.previousPos ||= new TypedVector3();
            node.shapeTarget ||= new TypedVector3();
            node.guideTarget ||= new TypedVector3();
            node.previousPos.copy(node.pos);
        }
        this.freeNodes[0].pos.copy(anchor);
        this.freeNodes[0].vel.set(0, 0, 0);

        for (let i = 1; i < this.freeNodes.length; i++) {
            const node = this.freeNodes[i];
            node.curl = Math.min(1, (node.curl ?? 1) + PIGTAIL_RELEASE_CURL_RATE * dt);
            const relativeDistance = Math.max(0, (node.distance ?? 0) - (this.freeNodes[0].distance ?? 0));
            const rest = this.#freeShapeTarget(
                relativeDistance,
                frame,
                state.freeLength,
                node.curl,
                node.shapeTarget
            );
            const shapeImpulse = FREE_SHAPE_STIFFNESS * dt;
            node.vel.x += (rest.x - node.pos.x) * shapeImpulse;
            node.vel.y += (rest.y - node.pos.y) * shapeImpulse;
            node.vel.z += (rest.z - node.pos.z) * shapeImpulse;
            node.vel.multiplyScalar(FREE_DAMPING);
            node.pos.addScaledVector(node.vel, dt);
        }

        const constraintIterations = collisions
            ? FREE_CONSTRAINT_ITERATIONS
            : FREE_XPBD_TARGET_ITERATIONS;
        for (let iter = 0; iter < constraintIterations; iter++) {
            this.freeNodes[0].pos.copy(anchor);
            this.#recaptureFreeNodes(state);
            this.#solveFreeLengthConstraints();
            this.#solveFreeShape(frame, state.freeLength);
            this.#solveFreeBending(frame);
            this.#solveFreeStraightening(state.freeLength);
            this.#limitFreeBends(state.freeLength);
            if (collisions) {
                this.#collideFreeNodes();
                this.#collideFreeSegments();
            }
            this.#solveFreeLengthConstraints();
        }

        const invDt = 1 / Math.max(1e-4, dt);
        for (let i = 1; i < this.freeNodes.length; i++) {
            const node = this.freeNodes[i];
            node.vel.subVectors(node.pos, node.previousPos).multiplyScalar(invDt * FREE_DAMPING);
        }
        this.freeNodes[0].vel.set(0, 0, 0);
    }

    #updateExternalShapeTargets(state, dt) {
        if (state.freeLength < 2 || state.supportEnd <= 0) {
            this.#clearFreeNodes();
            this.freeRestDistanceCount = 0;
            this.freeLength = 0;
            return;
        }
        const frame = this.#freeFrame(state.supportEnd);
        this.#syncFreeNodes(state, frame);
        if (this.freeNodes.length < 2) return;
        this.freeNodes[0].pos.copy(frame.supportTip);
        this.freeNodes[0].vel.set(0, 0, 0);
        const baseDistance = this.freeNodes[0].distance ?? state.supportEnd;
        for (let index = 1; index < this.freeNodes.length; index++) {
            const node = this.freeNodes[index];
            node.curl = Math.min(1, (node.curl ?? 1) + PIGTAIL_RELEASE_CURL_RATE * dt);
            const relativeDistance = Math.max(0, (node.distance ?? baseDistance) - baseDistance);
            node.pos.copy(this.#freeShapeTarget(
                relativeDistance,
                frame,
                state.freeLength,
                1,
                node.shapeTarget
            ));
            node.vel.set(0, 0, 0);
        }
    }

    constrainGuidewire(dt = 1 / 60, { reactionScale = 1 } = {}) {
        if (this.progress < 4) return;
        const state = this.#deploymentState();
        if (state.freeLength >= 2 && this.freeNodes.length < 2 && state.supportEnd > 0) {
            this.#syncFreeNodes(state, this.#freeFrame(state.supportEnd));
        }
        const tailProgress = this.tailProgressRef();
        const catheterCenterlineEnd = this.freeNodes.length >= 2
            ? Math.max(state.pathEnd, this.progress)
            : state.pathEnd;
        const constrainedEnd = Math.min(this.progress, this.guidewireInserted, catheterCenterlineEnd);
        if (constrainedEnd <= 0) return;

        for (let i = 0; i < this.wire.nodes.length; i++) {
            const distance = this.#nodeInsertedCoordinate(i, tailProgress);
            if (distance <= 0 || distance > constrainedEnd) continue;
            const node = this.wire.nodes[i];
            if (node.pinned) continue;

            const target = this.#sampleCatheterCenterline(distance, state);
            const oldX = node.x;
            const oldY = node.y;
            const oldZ = node.z;
            const entranceTaper = smoothstep(0, this.segmentLength * 1.5, distance);
            const weight = (0.6 + entranceTaper * 0.4) * GUIDEWIRE_IN_CATHETER_BLEND;
            const correction = target.clone().sub(new TypedVector3(oldX, oldY, oldZ));
            const correctionLength = correction.length();
            if (correctionLength > GUIDEWIRE_CATHETER_MAX_CORRECTION) {
                correction.multiplyScalar(GUIDEWIRE_CATHETER_MAX_CORRECTION / correctionLength);
            }
            const appliedCorrection = correction.multiplyScalar(weight);

            node.x = oldX + appliedCorrection.x;
            node.y = oldY + appliedCorrection.y;
            node.z = oldZ + appliedCorrection.z;

            const invDt = 1 / Math.max(1e-4, dt);
            node.vx = (node.x - oldX) * invDt * 0.25;
            node.vy = (node.y - oldY) * invDt * 0.25;
            node.vz = (node.z - oldZ) * invDt * 0.25;

            this.#applyGuidewireReaction(distance, appliedCorrection, reactionScale);
        }
    }

    updateMesh() {
        const body = this.physicsBody;
        const points = body ? null : this.#buildCenterline();
        const pointCount = body ? this.physicsActiveCount : this._centerlinePointCount;
        if (pointCount < 2) {
            this.mesh.visible = false;
            this.tipMarker.visible = false;
            return;
        }

        const renderPointCount = Math.min(pointCount, this.maxRenderSegments + 1);
        this._renderPoints.length = renderPointCount;
        for (let index = 0; index < renderPointCount; index++) {
            let renderPoint = this._renderPoints[index];
            if (!renderPoint) {
                renderPoint = new THREE.Vector3();
                this._renderPoints[index] = renderPoint;
            }
            renderPoint.set(
                body ? body.x[index] : points[index].x,
                body ? body.y[index] : points[index].y,
                body ? body.z[index] : points[index].z
            );
        }
        const previousGeometry = this.shaftMesh.geometry;
        this.shaftMesh.geometry = createSmoothTubeGeometry(this._renderPoints, {
            radius: PIGTAIL_CATHETER_RENDER_RADIUS_MM,
            samplesPerSegment: 3,
            radialSegments: 14
        });
        previousGeometry.dispose();
        this.#updateTipMarker(renderPointCount);
        this.mesh.visible = true;
    }

    getInjectionPorts(out = []) {
        out.length = 0;
        const body = this.physicsBody;
        const points = body ? null : this.#buildCenterline();
        const pointCount = body ? this.physicsActiveCount : this._centerlinePointCount;
        if (pointCount < 2) return out;

        if (this.type === CATHETER_TYPE_BERENSTEIN) {
            const port = this._injectionPortPool[0];
            if (!this.#sampleDistalCenterline(
                0,
                port.position,
                port.direction,
                body,
                points,
                pointCount
            )) return out;
            port.kind = 'berenstein-end';
            port.radiusMm = PIGTAIL_CATHETER_INNER_RADIUS_MM;
            port.areaMm2 = Math.PI * PIGTAIL_CATHETER_INNER_RADIUS_MM ** 2;
            port.weight = port.areaMm2;
            port.valid = true;
            out.push(port);
            return out;
        }

        for (let index = 0; index < PIGTAIL_INJECTION_PORT_OFFSETS_MM.length; index++) {
            const port = this._injectionPortPool[index];
            const sampled = this.#sampleDistalCenterline(
                PIGTAIL_INJECTION_PORT_OFFSETS_MM[index],
                port.position,
                this._injectionTangent,
                body,
                points,
                pointCount
            );
            if (!sampled) continue;
            const tangent = this._injectionTangent;
            this._injectionHelper.set(
                Math.abs(tangent.y) < 0.86 ? 0 : 1,
                Math.abs(tangent.y) < 0.86 ? 1 : 0,
                0
            );
            this._injectionNormal.crossVectors(tangent, this._injectionHelper).normalize();
            this._injectionBinormal.crossVectors(tangent, this._injectionNormal).normalize();
            const angle = this.rotation + index * GOLDEN_ANGLE;
            port.direction.copy(this._injectionNormal).multiplyScalar(Math.cos(angle))
                .addScaledVector(this._injectionBinormal, Math.sin(angle))
                .normalize();
            port.kind = 'pigtail-side';
            port.radiusMm = PIGTAIL_INJECTION_PORT_RADIUS_MM;
            port.areaMm2 = Math.PI * PIGTAIL_INJECTION_PORT_RADIUS_MM ** 2;
            port.weight = port.areaMm2;
            port.valid = true;
            out.push(port);
        }
        return out;
    }

    #sampleDistalCenterline(distanceFromTip, outPosition, outTangent, body, points, pointCount) {
        let remaining = Math.max(0, distanceFromTip);
        for (let index = pointCount - 1; index > 0; index--) {
            const distalX = body ? body.x[index] : points[index].x;
            const distalY = body ? body.y[index] : points[index].y;
            const distalZ = body ? body.z[index] : points[index].z;
            const proximalX = body ? body.x[index - 1] : points[index - 1].x;
            const proximalY = body ? body.y[index - 1] : points[index - 1].y;
            const proximalZ = body ? body.z[index - 1] : points[index - 1].z;
            const dx = distalX - proximalX;
            const dy = distalY - proximalY;
            const dz = distalZ - proximalZ;
            const segmentLength = Math.hypot(dx, dy, dz);
            if (segmentLength < 1e-6) continue;
            if (remaining > segmentLength) {
                remaining -= segmentLength;
                continue;
            }
            const fromDistal = remaining / segmentLength;
            outPosition.set(
                distalX - dx * fromDistal,
                distalY - dy * fromDistal,
                distalZ - dz * fromDistal
            );
            outTangent.set(dx / segmentLength, dy / segmentLength, dz / segmentLength);
            return true;
        }
        return false;
    }

    #updateTipMarker(pointCount) {
        const markerDistance = this.type === CATHETER_TYPE_BERENSTEIN
            ? BERENSTEIN_TIP_SHAPE_LENGTH
            : DISTAL_RELEASE_LENGTH;
        let traversed = 0;
        for (let index = pointCount - 1; index > 0; index--) {
            const distal = this._renderPoints[index];
            const proximal = this._renderPoints[index - 1];
            const segmentLength = distal.distanceTo(proximal);
            if (segmentLength < 1e-6) continue;
            if (traversed + segmentLength < markerDistance) {
                traversed += segmentLength;
                continue;
            }
            const t = clamp(
                (markerDistance - traversed) / segmentLength,
                0,
                1
            );
            this._tipMarkerPosition.copy(distal).lerp(proximal, t);
            this._tipMarkerTangent.subVectors(distal, proximal).normalize();
            this.tipMarker.position.copy(this._tipMarkerPosition);
            this.tipMarker.quaternion.setFromUnitVectors(
                this._tipMarkerUp,
                this._tipMarkerTangent
            );
            this.tipMarker.userData.tipLengthMm = markerDistance;
            this.tipMarker.userData.catheterType = this.type;
            this.tipMarker.visible = true;
            return;
        }
        this.tipMarker.visible = false;
    }

    #buildCenterline() {
        const state = this.#deploymentState();
        const externalLength = this.sheathPath ? EXTERNAL_CATHETER_VISIBLE_LENGTH : 0;
        this.physicsLumenStartNode = 0;
        this._centerlinePointCount = 0;
        if (state.pathEnd <= 0 && externalLength <= 0) return this._centerlinePoints;
        const shaftEnd = Math.max(0, state.supportEnd);
        const shaftSamples = shaftEnd > 0 ? clamp(Math.ceil(shaftEnd / 5), 1, 90) : 0;
        const points = this._centerlinePoints;

        if (externalLength > 0) {
            const externalSamples = clamp(Math.ceil(externalLength / 6), 2, 24);
            for (let i = 0; i <= externalSamples; i++) {
                const s = -externalLength + externalLength * i / externalSamples;
                const index = this._centerlinePointCount++;
                this.#sampleCatheterPath(s, this.#centerlinePoint(index));
                this._centerlineDistances[index] = s;
            }
            this.physicsLumenStartNode = externalSamples;
        }

        if (state.pathEnd <= 0) return points;

        const shaftStartIndex = this._centerlinePointCount ? 1 : 0;
        for (let i = shaftStartIndex; i <= shaftSamples; i++) {
            const s = shaftSamples > 0 ? shaftEnd * i / shaftSamples : 0;
            const index = this._centerlinePointCount++;
            this.#sampleCatheterPath(s, this.#centerlinePoint(index));
            this._centerlineDistances[index] = s;
        }

        if (state.freeLength < 2) {
            if (state.pathEnd > shaftEnd + 0.5) {
                const index = this._centerlinePointCount++;
                this.#sampleCatheterPath(
                    state.pathEnd,
                    this.#centerlinePoint(index)
                );
                this._centerlineDistances[index] = state.pathEnd;
            }
            return points;
        }

        const frame = this.#freeFrame(state.supportEnd);
        this.#syncFreeNodes(state, frame);
        for (let i = 1; i < this.freeNodes.length; i++) {
            const index = this._centerlinePointCount++;
            this.#centerlinePoint(index).copy(this.freeNodes[i].pos);
            this._centerlineDistances[index] = this.freeNodes[i].distance ?? state.supportEnd;
        }
        return points;
    }

    #centerlinePoint(index) {
        let point = this._centerlinePoints[index];
        if (!point) {
            point = new TypedVector3();
            this._centerlinePoints[index] = point;
        }
        return point;
    }

    #deploymentState() {
        const state = this._deploymentStateScratch;
        if (this.progress < 4) {
            state.pathEnd = 0;
            state.supportEnd = 0;
            state.freeLength = 0;
            return state;
        }

        const sheathSupportEnd = this.#sheathSupportEnd();
        const pathEnd = Math.max(sheathSupportEnd, Math.min(this.progress, this.#pathEndDistance()));
        state.pathEnd = pathEnd;
        state.supportEnd = pathEnd > 0 ? sheathSupportEnd : 0;
        state.freeLength = pathEnd > 0 ? Math.max(0, this.progress - sheathSupportEnd) : 0;
        return state;
    }

    #freeFrame(supportEnd) {
        const frame = this._freeFrameScratch;
        const supportTip = this.#sampleCatheterPath(supportEnd, frame.supportTip);
        const beforeTip = this.#sampleCatheterPath(Math.max(0, supportEnd - 10), frame.beforeTip);
        const beforePlane = this.#sampleCatheterPath(Math.max(0, supportEnd - 28), frame.beforePlane);
        const tangent = frame.tangent.subVectors(supportTip, beforeTip);
        if (tangent.lengthSq() < 1e-5) tangent.set(0, 1, 0);
        tangent.normalize();
        const normal = this.#catheterPlaneNormal(tangent, beforeTip, beforePlane, frame.normal);
        const shapeRotation =
            this.type === CATHETER_TYPE_BERENSTEIN &&
            this.externalCollisionSolver &&
            this._xpbdBerensteinTwisted
                ? 0
                : this.rotation;
        normal.applyAxisAngle(tangent, shapeRotation).normalize();
        return frame;
    }

    #syncFreeNodes(state, frame) {
        const distances = this.freeRestDistances;
        distances[0] = state.supportEnd;
        let distanceCount = 1;
        let d = state.supportEnd;
        while (d + FREE_NODE_SPACING < this.progress - 0.5) {
            d += FREE_NODE_SPACING;
            distances[distanceCount++] = d;
        }
        if (this.progress > distances[distanceCount - 1] + 0.5) {
            distances[distanceCount++] = this.progress;
        }
        this.freeRestDistanceCount = distanceCount;

        const oldNodes = this.freeNodes;
        const nextNodes = this._nextFreeNodes;
        nextNodes.length = 0;
        const epoch = ++this._freeNodeEpoch;
        let oldCursor = 0;

        for (let distanceIndex = 0; distanceIndex < distanceCount; distanceIndex++) {
            const distance = distances[distanceIndex];
            const relativeDistance = distance - state.supportEnd;
            let bestIndex = -1;
            let bestDelta = Infinity;
            while (oldCursor < oldNodes.length) {
                const delta = Math.abs((oldNodes[oldCursor].distance ?? 0) - distance);
                const nextDelta = oldCursor + 1 < oldNodes.length
                    ? Math.abs((oldNodes[oldCursor + 1].distance ?? 0) - distance)
                    : Infinity;
                if (nextDelta >= delta) {
                    bestIndex = oldCursor;
                    bestDelta = delta;
                    break;
                }
                oldCursor++;
            }

            let node;
            if (bestIndex >= 0 && bestDelta <= FREE_NODE_SPACING * 0.7) {
                node = oldNodes[bestIndex];
                oldCursor = bestIndex + 1;
            } else {
                const wasJustReleased = this.guidewireDelta < -1e-4
                    && distance >= this.guidewireInserted - GUIDE_CAPTURE_TOLERANCE
                    && distance <= this.previousGuidewireInserted + GUIDE_CAPTURE_TOLERANCE;
                const restPoint = this.#freeShapeTarget(
                    relativeDistance,
                    frame,
                    state.freeLength,
                    wasJustReleased ? PIGTAIL_RELEASE_CURL_START : 1,
                    this._newNodeRest
                );
                const pathPoint = this.#sampleCatheterPath(
                    Math.min(distance, this.#pathEndDistance()),
                    this._newNodePath
                );
                const guideSupported = this.guidewireInserted > MIN_GUIDE_SUPPORT
                    && distance <= this.guidewireInserted + GUIDE_CAPTURE_TOLERANCE;
                const point = this._newNodePoint;
                if (wasJustReleased) point.copy(pathPoint).lerp(restPoint, PIGTAIL_RELEASE_CURL_START);
                else if (guideSupported) {
                    point.copy(this.#sampleGuidewire(distance, this._newNodeGuide)).lerp(restPoint, 0.28);
                } else point.copy(restPoint);
                const projectedPoint = this.externalCollisionSolver
                    ? point
                    : this.#projectInsideVesselDetailed(point).point;
                node = this.#acquireFreeNode(
                    projectedPoint,
                    distance,
                    wasJustReleased ? PIGTAIL_RELEASE_CURL_START : 1
                );
            }
            node._activeEpoch = epoch;
            node.distance = distance;
            node.curl = node.curl ?? 1;
            node.previousPos ||= new TypedVector3();
            node.shapeTarget ||= new TypedVector3();
            node.guideTarget ||= new TypedVector3();
            nextNodes.push(node);
        }

        for (let index = 0; index < oldNodes.length; index++) {
            const node = oldNodes[index];
            if (node._activeEpoch === epoch || node._pooled) continue;
            node._pooled = true;
            this._freeNodePool.push(node);
        }

        this._nextFreeNodes = oldNodes;
        this.freeNodes = nextNodes;
        this.freeLength = state.freeLength;
        if (this.freeNodes[0]) {
            this.freeNodes[0].pos.copy(frame.supportTip);
            this.freeNodes[0].vel.set(0, 0, 0);
        }
    }

    #acquireFreeNode(point, distance, curl) {
        const node = this._freeNodePool.pop() || {
            pos: new TypedVector3(),
            vel: new TypedVector3(),
            previousPos: new TypedVector3(),
            shapeTarget: new TypedVector3(),
            guideTarget: new TypedVector3(),
            distance: 0,
            curl: 1,
            _activeEpoch: 0,
            _pooled: false
        };
        node._pooled = false;
        node.pos.copy(point);
        node.vel.set(0, 0, 0);
        node.previousPos.copy(point);
        node.shapeTarget.copy(point);
        node.guideTarget.copy(point);
        node.distance = distance;
        node.curl = curl;
        return node;
    }

    #clearFreeNodes() {
        for (let listIndex = 0; listIndex < 2; listIndex++) {
            const nodes = listIndex === 0 ? this.freeNodes : this._nextFreeNodes;
            for (let index = 0; index < nodes.length; index++) {
                const node = nodes[index];
                if (node._pooled) continue;
                node._pooled = true;
                this._freeNodePool.push(node);
            }
            nodes.length = 0;
        }
    }

    #recaptureFreeNodes(state) {
        const supportedEnd = Math.min(this.progress, this.guidewireInserted);
        if (supportedEnd <= state.supportEnd + 0.5 || this.freeNodes.length < 2) return;

        const moving = Math.abs(this.motionCommand) > 0;
        const baseBlend = moving ? GUIDEWIRE_MOVING_SUPPORT_BLEND : GUIDEWIRE_SUPPORT_BLEND;
        for (let i = 1; i < this.freeNodes.length; i++) {
            const pathDistance = this.freeNodes[i].distance ?? state.supportEnd;
            if (pathDistance > supportedEnd + GUIDE_CAPTURE_TOLERANCE) continue;
            const entranceFade = smoothstep(state.supportEnd, state.supportEnd + GUIDEWIRE_RECAPTURE_WINDOW, pathDistance);
            const tipFade = 1 - smoothstep(supportedEnd - GUIDEWIRE_RECAPTURE_WINDOW, supportedEnd + GUIDE_CAPTURE_TOLERANCE, pathDistance);
            const target = this.#sampleGuidewire(pathDistance, this.freeNodes[i].guideTarget);
            const blend = baseBlend * entranceFade * (0.35 + tipFade * 0.65);
            this.freeNodes[i].pos.lerp(target, blend);
            this.freeNodes[i].vel.multiplyScalar(1 - blend);
        }
    }

    #relaxSupportedPath(pathEnd) {
        const sheathLength = this.sheathPath?.length || 0;
        if (this.pathSamples.length < 3 || pathEnd <= sheathLength + this.pathSpacing * 2) return;

        for (let pass = 0; pass < PATH_RELAXATION_PASSES; pass++) {
            this.#straightenPathSamples(pathEnd, sheathLength);
            this.#limitPathBends(pathEnd, sheathLength);
        }
    }

    #straightenPathSamples(pathEnd, sheathLength) {
        const exitStart = sheathLength + this.pathSpacing * 1.5;
        const exitInvSpan = 1 / Math.max(1e-8, this.pathSpacing * 6.5);
        const tipStart = pathEnd - this.pathSpacing * 4;
        const tipInvSpan = 1 / Math.max(1e-8, this.pathSpacing * 4);
        for (let i = 1; i < this.pathSamples.length - 1; i++) {
            const sample = this.pathSamples[i];
            const exitT = Math.max(0, Math.min(1, (sample.distance - exitStart) * exitInvSpan));
            const tipT = Math.max(0, Math.min(1, (sample.distance - tipStart) * tipInvSpan));
            const exitWeight = exitT * exitT * (3 - 2 * exitT);
            const tipWeight = 1 - tipT * tipT * (3 - 2 * tipT);
            const weight = exitWeight * (0.35 + tipWeight * 0.65);
            if (weight <= 0.001) continue;
            const previous = this.pathSamples[i - 1].point._values;
            const next = this.pathSamples[i + 1].point._values;
            this.#movePathSample(
                sample,
                (previous[0] + next[0]) * 0.5,
                (previous[1] + next[1]) * 0.5,
                (previous[2] + next[2]) * 0.5,
                PATH_STRAIGHTENING * weight
            );
        }

        for (let spanIndex = 0; spanIndex < PATH_STRAIGHTENING_SPANS.length; spanIndex++) {
            const span = PATH_STRAIGHTENING_SPANS[spanIndex];
            if (this.pathSamples.length <= span * 2) continue;
            for (let i = span; i < this.pathSamples.length - span; i++) {
                const sample = this.pathSamples[i];
                const exitT = Math.max(
                    0,
                    Math.min(1, (sample.distance - exitStart) * exitInvSpan)
                );
                const tipT = Math.max(
                    0,
                    Math.min(1, (sample.distance - tipStart) * tipInvSpan)
                );
                const exitWeight = exitT * exitT * (3 - 2 * exitT);
                const tipWeight = 1 - tipT * tipT * (3 - 2 * tipT);
                const weight = exitWeight * (0.35 + tipWeight * 0.65);
                if (weight <= 0.001) continue;
                const previous = this.pathSamples[i - span].point._values;
                const next = this.pathSamples[i + span].point._values;
                this.#movePathSample(
                    sample,
                    (previous[0] + next[0]) * 0.5,
                    (previous[1] + next[1]) * 0.5,
                    (previous[2] + next[2]) * 0.5,
                    PATH_LONG_SPAN_STRAIGHTENING * weight / Math.sqrt(span)
                );
            }
        }
    }

    #limitPathBends(pathEnd, sheathLength) {
        const minDot = Math.cos(PATH_MAX_BEND_ANGLE);
        for (let i = 1; i < this.pathSamples.length - 1; i++) {
            const sample = this.pathSamples[i];
            const weight = this.#pathRelaxationWeight(sample.distance, pathEnd, sheathLength);
            if (weight <= 0.001) continue;

            const prev = this.pathSamples[i - 1].point;
            const curr = sample.point;
            const next = this.pathSamples[i + 1].point;
            const inX = curr.x - prev.x;
            const inY = curr.y - prev.y;
            const inZ = curr.z - prev.z;
            const outX = next.x - curr.x;
            const outY = next.y - curr.y;
            const outZ = next.z - curr.z;
            const inLength = magnitude3(inX, inY, inZ);
            const outLength = magnitude3(outX, outY, outZ);
            if (inLength < 1e-5 || outLength < 1e-5) continue;

            const dot = clamp(
                (inX * outX + inY * outY + inZ * outZ) / (inLength * outLength),
                -1,
                1
            );
            if (dot >= minDot) continue;

            const severity = clamp((Math.acos(dot) - PATH_MAX_BEND_ANGLE) / (Math.PI - PATH_MAX_BEND_ANGLE), 0, 1);
            this.#movePathSample(
                sample,
                (prev.x + next.x) * 0.5,
                (prev.y + next.y) * 0.5,
                (prev.z + next.z) * 0.5,
                PATH_BEND_LIMIT_STRENGTH * severity * weight
            );
        }
    }

    #pathRelaxationWeight(distance, pathEnd, sheathLength) {
        if (distance <= sheathLength + this.pathSpacing) return 0;
        const exitWeight = smoothstep(sheathLength + this.pathSpacing * 1.5, sheathLength + this.pathSpacing * 8, distance);
        const tipWeight = 1 - smoothstep(pathEnd - this.pathSpacing * 4, pathEnd, distance);
        return exitWeight * (0.35 + tipWeight * 0.65);
    }

    #movePathSample(sample, targetX, targetY, targetZ, amount) {
        const blend = clamp(amount, 0, 1);
        const pointValues = sample.point._values;
        let dx = (targetX - pointValues[0]) * blend;
        let dy = (targetY - pointValues[1]) * blend;
        let dz = (targetZ - pointValues[2]) * blend;
        const deltaLength = magnitude3(dx, dy, dz);
        if (deltaLength <= 1e-6) return;
        if (deltaLength > PATH_MAX_RELAX_STEP) {
            const scale = PATH_MAX_RELAX_STEP / deltaLength;
            dx *= scale;
            dy *= scale;
            dz *= scale;
        }
        if (this.externalCollisionSolver) {
            pointValues[0] += dx;
            pointValues[1] += dy;
            pointValues[2] += dz;
            return;
        }
        this._pathTarget.set(pointValues[0] + dx, pointValues[1] + dy, pointValues[2] + dz);
        sample.point.copy(this.#projectInsideVesselDetailed(this._pathTarget).point);
    }

    #freeRestPoint(distance, frame, freeLength, curlScale = 1, out = new TypedVector3()) {
        if (this.type === CATHETER_TYPE_BERENSTEIN) {
            return this.#berensteinRestPoint(distance, frame, freeLength, curlScale, out);
        }

        const deployLength = Math.min(freeLength, DISTAL_RELEASE_LENGTH);
        const proximalFreeLength = Math.max(0, freeLength - deployLength);
        if (distance <= proximalFreeLength) {
            return out.copy(frame.supportTip).addScaledVector(frame.tangent, distance);
        }

        const local = distance - proximalFreeLength;
        const leadLength = Math.min(deployLength, STRAIGHT_EXIT_LENGTH);
        const curvatureScale = clamp(curlScale, 0, 1);
        if (local <= leadLength || curvatureScale <= 0.001) {
            return out.copy(frame.supportTip).addScaledVector(frame.tangent, distance);
        }

        const arcDistance = Math.min(local - leadLength, PIGTAIL_ARC_LENGTH);
        const radius = PIGTAIL_RADIUS / curvatureScale;
        const theta = Math.min(PIGTAIL_TURNS * Math.PI * 2, arcDistance / radius);
        return out.copy(frame.supportTip)
            .addScaledVector(frame.tangent, proximalFreeLength + leadLength + Math.sin(theta) * radius)
            .addScaledVector(frame.normal, (Math.cos(theta) - 1) * radius);
    }

    #berensteinRestPoint(distance, frame, freeLength, curlScale = 1, out = new TypedVector3()) {
        const deployLength = Math.min(freeLength, BERENSTEIN_TIP_SHAPE_LENGTH);
        const proximalFreeLength = Math.max(0, freeLength - deployLength);
        if (distance <= proximalFreeLength) {
            return out.copy(frame.supportTip).addScaledVector(frame.tangent, distance);
        }

        const local = distance - proximalFreeLength;
        const leadLength = Math.min(deployLength, BERENSTEIN_STRAIGHT_EXIT_LENGTH);
        if (local <= leadLength) {
            return out.copy(frame.supportTip).addScaledVector(frame.tangent, distance);
        }

        const targetAngle = BERENSTEIN_BEND_ANGLE * clamp(curlScale, 0, 1);
        if (targetAngle <= 0.001) {
            return out.copy(frame.supportTip).addScaledVector(frame.tangent, distance);
        }

        const bendNormal = this.#berensteinBendNormal(frame, this._shapeNormal);
        const availableBendLength = Math.max(0, deployLength - leadLength);
        const bendLength = Math.min(BERENSTEIN_BEND_LENGTH, Math.max(1e-4, availableBendLength));
        const bendDistance = Math.min(local - leadLength, bendLength);
        const angle = targetAngle * clamp(bendDistance / bendLength, 0, 1);
        const radius = bendLength / targetAngle;
        out.copy(frame.supportTip)
            .addScaledVector(frame.tangent, proximalFreeLength + leadLength)
            .addScaledVector(frame.tangent, Math.sin(angle) * radius)
            .addScaledVector(bendNormal, (1 - Math.cos(angle)) * radius);

        const afterBend = local - leadLength - bendLength;
        if (afterBend > 0) {
            out.addScaledVector(frame.tangent, Math.cos(targetAngle) * afterBend)
                .addScaledVector(bendNormal, Math.sin(targetAngle) * afterBend);
        }
        return out;
    }

    #berensteinBendNormal(frame, normal = new TypedVector3()) {
        normal.copy(frame.normal);
        normal.z *= 0.18;
        normal.addScaledVector(frame.tangent, -normal.dot(frame.tangent));
        if (normal.lengthSq() < 1e-6) return normal.copy(frame.normal);
        return normal.normalize();
    }

    #solveFreeLengthConstraints() {
        for (let i = 1; i < this.freeNodes.length; i++) {
            const prev = this.freeNodes[i - 1];
            const node = this.freeNodes[i];
            const desired = Math.max(0.5, (node.distance ?? 0) - (prev.distance ?? 0));
            const dx = node.pos.x - prev.pos.x;
            const dy = node.pos.y - prev.pos.y;
            const dz = node.pos.z - prev.pos.z;
            const dist = magnitude3(dx, dy, dz);
            if (dist < 1e-5) continue;
            const correction = (dist - desired) / dist;
            if (i === 1) {
                node.pos.x -= dx * correction;
                node.pos.y -= dy * correction;
                node.pos.z -= dz * correction;
            } else {
                const halfCorrection = correction * 0.5;
                prev.pos.x += dx * halfCorrection;
                prev.pos.y += dy * halfCorrection;
                prev.pos.z += dz * halfCorrection;
                node.pos.x -= dx * halfCorrection;
                node.pos.y -= dy * halfCorrection;
                node.pos.z -= dz * halfCorrection;
            }
        }
    }

    #solveFreeBending(frame) {
        if (this.freeNodes.length > 1) {
            const firstDistance = Math.max(0.5, (this.freeNodes[1].distance ?? 0) - (this.freeNodes[0].distance ?? 0)) || FREE_NODE_SPACING;
            const node = this.freeNodes[1].pos;
            node.x += (frame.supportTip.x + frame.tangent.x * firstDistance - node.x) * FREE_ANCHOR_STIFFNESS;
            node.y += (frame.supportTip.y + frame.tangent.y * firstDistance - node.y) * FREE_ANCHOR_STIFFNESS;
            node.z += (frame.supportTip.z + frame.tangent.z * firstDistance - node.z) * FREE_ANCHOR_STIFFNESS;
        }

        for (let i = 2; i < this.freeNodes.length - 1; i++) {
            const prev = this.freeNodes[i - 1].pos;
            const next = this.freeNodes[i + 1].pos;
            const node = this.freeNodes[i].pos;
            node.x += ((prev.x + next.x) * 0.5 - node.x) * FREE_BEND_SMOOTHING;
            node.y += ((prev.y + next.y) * 0.5 - node.y) * FREE_BEND_SMOOTHING;
            node.z += ((prev.z + next.z) * 0.5 - node.z) * FREE_BEND_SMOOTHING;
        }
    }

    #solveFreeShape(frame, freeLength) {
        for (let i = 1; i < this.freeNodes.length; i++) {
            const relativeDistance = Math.max(0, (this.freeNodes[i].distance ?? 0) - (this.freeNodes[0].distance ?? 0));
            const target = this.#freeShapeTarget(
                relativeDistance,
                frame,
                freeLength,
                this.freeNodes[i].curl ?? 1,
                this.freeNodes[i].shapeTarget
            );
            const distalWeight = smoothstep(0, Math.max(FREE_NODE_SPACING, freeLength), relativeDistance);
            const shapeWeight = this.#distalTipShapeWeight(relativeDistance, freeLength);
            const shapeBoost = this.type === CATHETER_TYPE_PIGTAIL ? 2.4 : 1.2;
            const blend = FREE_SHAPE_POSITION_BLEND *
                (0.45 + distalWeight * 0.55) *
                (1 + shapeWeight * shapeBoost);
            const amount = clamp(blend, 0, 0.68);
            const positionValues = this.freeNodes[i].pos._values;
            const targetValues = target._values;
            positionValues[0] += (targetValues[0] - positionValues[0]) * amount;
            positionValues[1] += (targetValues[1] - positionValues[1]) * amount;
            positionValues[2] += (targetValues[2] - positionValues[2]) * amount;
        }
    }

    #solveFreeStraightening(freeLength) {
        if (this.freeNodes.length < 4) return;
        const baseDistance = this.freeNodes[0]?.distance ?? 0;
        const shapeStart = Math.max(
            0,
            freeLength - this.#distalShapeLength(freeLength)
        );
        const weightStart = Math.max(0, shapeStart - 10);
        const weightSpan = Math.max(1e-8, shapeStart + 8 - weightStart);

        for (let i = 1; i < this.freeNodes.length - 1; i++) {
            const relativeDistance = Math.max(0, (this.freeNodes[i].distance ?? baseDistance) - baseDistance);
            const weightT = Math.max(0, Math.min(1, (relativeDistance - weightStart) / weightSpan));
            const weight = 1 - weightT * weightT * (3 - 2 * weightT);
            if (weight <= 0.001) continue;
            const prev = this.freeNodes[i - 1].pos._values;
            const next = this.freeNodes[i + 1].pos._values;
            const node = this.freeNodes[i].pos._values;
            const scale = this.#unsupportedCatheterScale(this.freeNodes[i].distance);
            const amount = clamp(FREE_SHAFT_STRAIGHTENING * weight * scale, 0, 1);
            node[0] += ((prev[0] + next[0]) * 0.5 - node[0]) * amount;
            node[1] += ((prev[1] + next[1]) * 0.5 - node[1]) * amount;
            node[2] += ((prev[2] + next[2]) * 0.5 - node[2]) * amount;
        }

        for (let spanIndex = 0; spanIndex < FREE_STRAIGHTENING_SPANS.length; spanIndex++) {
            const span = FREE_STRAIGHTENING_SPANS[spanIndex];
            if (this.freeNodes.length <= span * 2) continue;
            for (let i = span; i < this.freeNodes.length - span; i++) {
                const relativeDistance = Math.max(
                    0,
                    (this.freeNodes[i].distance ?? baseDistance) - baseDistance
                );
                const weightT = Math.max(
                    0,
                    Math.min(1, (relativeDistance - weightStart) / weightSpan)
                );
                const weight = 1 - weightT * weightT * (3 - 2 * weightT);
                if (weight <= 0.001) continue;
                const prev = this.freeNodes[i - span].pos._values;
                const next = this.freeNodes[i + span].pos._values;
                const scale = this.#unsupportedCatheterScale(this.freeNodes[i].distance);
                const amount = FREE_LONG_SPAN_STRAIGHTENING * weight * scale / Math.sqrt(span);
                const blend = clamp(amount, 0, 1);
                const node = this.freeNodes[i].pos._values;
                node[0] += ((prev[0] + next[0]) * 0.5 - node[0]) * blend;
                node[1] += ((prev[1] + next[1]) * 0.5 - node[1]) * blend;
                node[2] += ((prev[2] + next[2]) * 0.5 - node[2]) * blend;
            }
        }
    }

    #limitFreeBends(freeLength) {
        if (this.freeNodes.length < 3) return;
        const minDot = Math.cos(FREE_MAX_BEND_ANGLE);

        for (let i = 1; i < this.freeNodes.length - 1; i++) {
            const prev = this.freeNodes[i - 1].pos;
            const curr = this.freeNodes[i].pos;
            const next = this.freeNodes[i + 1].pos;
            const inX = curr.x - prev.x;
            const inY = curr.y - prev.y;
            const inZ = curr.z - prev.z;
            const outX = next.x - curr.x;
            const outY = next.y - curr.y;
            const outZ = next.z - curr.z;
            const inLength = magnitude3(inX, inY, inZ);
            const outLength = magnitude3(outX, outY, outZ);
            if (inLength < 1e-5 || outLength < 1e-5) continue;

            const dot = clamp(
                (inX * outX + inY * outY + inZ * outZ) / (inLength * outLength),
                -1,
                1
            );
            if (dot >= minDot) continue;

            const severity = clamp((Math.acos(dot) - FREE_MAX_BEND_ANGLE) / (Math.PI - FREE_MAX_BEND_ANGLE), 0, 1);
            const straighteningWeight = this.#shaftStraighteningWeight(this.freeNodes[i], freeLength);
            const straighteningScale = this.#unsupportedCatheterScale(this.freeNodes[i].distance);
            const unsupportedWeight = (straighteningScale - 1) /
                Math.max(1e-6, SOLO_CATHETER_STRAIGHTENING_SCALE - 1);
            const scale = 1 + (SOLO_CATHETER_BEND_LIMIT_SCALE - 1) * unsupportedWeight;
            const strength = FREE_BEND_LIMIT_STRENGTH * scale * severity * (0.28 + straighteningWeight * 0.72);
            const amount = clamp(strength, 0, 1);
            curr.x += ((prev.x + next.x) * 0.5 - curr.x) * amount;
            curr.y += ((prev.y + next.y) * 0.5 - curr.y) * amount;
            curr.z += ((prev.z + next.z) * 0.5 - curr.z) * amount;
        }
    }

    #collideFreeNodes() {
        for (let i = 1; i < this.freeNodes.length; i++) {
            const node = this.freeNodes[i];
            const projected = this.#projectInsideVesselDetailed(node.pos);
            if (!projected.collided) continue;
            node.pos.copy(projected.point);
            const normal = projected.normal;
            const outwardSpeed = node.vel.dot(normal);
            if (outwardSpeed > 0) node.vel.addScaledVector(normal, -outwardSpeed);
            node.vel.multiplyScalar(1 - FREE_WALL_FRICTION);
        }
    }

    #collideFreeSegments() {
        if (this.freeNodes.length < 2) return;

        for (let i = 1; i < this.freeNodes.length; i++) {
            const prev = this.freeNodes[i - 1];
            const node = this.freeNodes[i];
            for (const t of FREE_SEGMENT_COLLISION_SAMPLES) {
                const sample = prev.pos.clone().lerp(node.pos, t);
                const projected = this.#projectInsideVesselDetailed(sample);
                if (!projected.collided) continue;

                const correction = projected.point.sub(sample);
                const correctionLength = correction.length();
                if (correctionLength <= 1e-6) continue;
                if (correctionLength > FREE_SEGMENT_MAX_CORRECTION) {
                    correction.multiplyScalar(FREE_SEGMENT_MAX_CORRECTION / correctionLength);
                }

                const prevWeight = i === 1 ? 0 : 1 - t;
                const nodeWeight = i === 1 ? 1 : t;
                prev.pos.addScaledVector(correction, FREE_SEGMENT_COLLISION_STRENGTH * prevWeight);
                node.pos.addScaledVector(correction, FREE_SEGMENT_COLLISION_STRENGTH * nodeWeight);
                prev.vel.multiplyScalar(1 - FREE_WALL_FRICTION * prevWeight);
                node.vel.multiplyScalar(1 - FREE_WALL_FRICTION * nodeWeight);
            }
        }
    }

    #freeShapeTarget(distance, frame, freeLength, curlScale = 1, out = new TypedVector3()) {
        const absoluteDistance = this.#sheathSupportEnd() + distance;
        if (this.guidewireInserted > MIN_GUIDE_SUPPORT) {
            if (absoluteDistance <= this.guidewireInserted) {
                return out.copy(this.#sampleGuidewire(absoluteDistance, this._shapeNormal));
            }
            const unsupportedLength = Math.max(0, this.progress - this.guidewireInserted);
            const releasedDistance = Math.min(
                unsupportedLength,
                absoluteDistance - this.guidewireInserted
            );
            const releaseFrame = this.#guideReleaseFrame(frame);
            return this.#releasedDistalRestPoint(
                releasedDistance,
                releaseFrame,
                unsupportedLength,
                curlScale,
                out
            );
        }
        const target = this.#freeRestPoint(distance, frame, freeLength, curlScale, out);
        return this.externalCollisionSolver ? target : this.#projectInsideVesselDetailed(target).point;
    }

    #guideReleaseFrame(fallbackFrame) {
        const frame = this._guideReleaseFrameScratch;
        this.#sampleGuidewire(this.guidewireInserted, frame.supportTip);
        this.#sampleGuidewire(
            Math.max(this.#sheathSupportEnd(), this.guidewireInserted - 10),
            frame.beforeTip
        );
        frame.tangent.subVectors(frame.supportTip, frame.beforeTip);
        if (frame.tangent.lengthSq() < 1e-6) frame.tangent.copy(fallbackFrame.tangent);
        frame.tangent.normalize();
        frame.normal.copy(fallbackFrame.normal)
            .addScaledVector(frame.tangent, -fallbackFrame.normal.dot(frame.tangent));
        if (frame.normal.lengthSq() < 1e-6) frame.normal.copy(fallbackFrame.normal);
        frame.normal.normalize();
        return frame;
    }

    #updateGuidewireRelease(dt) {
        if (this.guidewireInserted <= MIN_GUIDE_SUPPORT) {
            this._guidewireRelease = 1;
            return;
        }
        const unsupportedLength = Math.max(0, this.progress - this.guidewireInserted);
        const releaseLength = this.type === CATHETER_TYPE_BERENSTEIN
            ? BERENSTEIN_STRAIGHT_EXIT_LENGTH + BERENSTEIN_BEND_LENGTH
            : PIGTAIL_ARC_LENGTH;
        const target = smoothstep(0, releaseLength, unsupportedLength);
        const rate = target >= this._guidewireRelease
            ? SHAPE_RECOVERY_RATE
            : SHAPE_RECAPTURE_RATE;
        this._guidewireRelease += clamp(
            target - this._guidewireRelease,
            -rate * dt,
            rate * dt
        );
    }

    #releasedDistalRestPoint(distance, frame, unsupportedLength, curlScale, out) {
        if (this.type === CATHETER_TYPE_BERENSTEIN) {
            const bendLength = Math.min(unsupportedLength, BERENSTEIN_BEND_LENGTH);
            const straightLength = Math.max(0, unsupportedLength - bendLength);
            if (distance <= straightLength || bendLength <= 1e-4) {
                return out.copy(frame.supportTip).addScaledVector(frame.tangent, distance);
            }
            const releaseScale = this._guidewireRelease * clamp(curlScale, 0, 1);
            const targetAngle = BERENSTEIN_BEND_ANGLE * releaseScale;
            if (targetAngle <= 1e-4) {
                return out.copy(frame.supportTip).addScaledVector(frame.tangent, distance);
            }
            const radius = BERENSTEIN_BEND_LENGTH / targetAngle;
            const arcDistance = Math.min(distance - straightLength, bendLength);
            const angle = arcDistance / radius;
            const bendNormal = this.#berensteinBendNormal(frame, this._shapeNormal);
            return out.copy(frame.supportTip)
                .addScaledVector(frame.tangent, straightLength + Math.sin(angle) * radius)
                .addScaledVector(bendNormal, (1 - Math.cos(angle)) * radius);
        }

        const arcLength = Math.min(unsupportedLength, PIGTAIL_ARC_LENGTH);
        const straightLength = Math.max(0, unsupportedLength - arcLength);
        if (distance <= straightLength || arcLength <= 1e-4) {
            return out.copy(frame.supportTip).addScaledVector(frame.tangent, distance);
        }
        const releaseScale = this._guidewireRelease * clamp(curlScale, 0, 1);
        if (releaseScale <= 1e-4) {
            return out.copy(frame.supportTip).addScaledVector(frame.tangent, distance);
        }
        const radius = PIGTAIL_RADIUS / releaseScale;
        const arcDistance = Math.min(distance - straightLength, arcLength);
        const theta = Math.min(PIGTAIL_TURNS * Math.PI * 2, arcDistance / radius);
        return out.copy(frame.supportTip)
            .addScaledVector(frame.tangent, straightLength + Math.sin(theta) * radius)
            .addScaledVector(frame.normal, (Math.cos(theta) - 1) * radius);
    }

    #shaftStraighteningWeight(node, freeLength) {
        const baseDistance = this.freeNodes[0]?.distance ?? 0;
        const relativeDistance = Math.max(0, (node.distance ?? baseDistance) - baseDistance);
        const shapeStart = Math.max(0, freeLength - this.#distalShapeLength(freeLength));
        return 1 - smoothstep(Math.max(0, shapeStart - 10), shapeStart + 8, relativeDistance);
    }

    #distalTipShapeWeight(relativeDistance, freeLength) {
        const shapeStart = Math.max(0, freeLength - this.#distalShapeLength(freeLength));
        return smoothstep(shapeStart - 2, shapeStart + 10, relativeDistance);
    }

    #xpbdShapeMemoryWeight(insertedDistance) {
        if (!Number.isFinite(insertedDistance) || insertedDistance <= 0) return 0;
        const curvedTipLength = this.type === CATHETER_TYPE_BERENSTEIN
            ? BERENSTEIN_STRAIGHT_EXIT_LENGTH + BERENSTEIN_BEND_LENGTH
            : PIGTAIL_ARC_LENGTH;
        const distalStart = Math.max(this.#sheathSupportEnd(), this.progress - curvedTipLength);
        const distalWeight = smoothstep(
            distalStart - 2,
            distalStart + XPBD_SHAPE_ACTIVATION_LENGTH,
            insertedDistance
        );
        if (this.guidewireInserted <= MIN_GUIDE_SUPPORT) return distalWeight;
        const releaseWeight = smoothstep(
            this.guidewireInserted + 0.5,
            this.guidewireInserted + XPBD_SHAPE_ACTIVATION_LENGTH,
            insertedDistance
        );
        return distalWeight * releaseWeight;
    }

    #xpbdSoftTipWeight(insertedDistance) {
        if (!Number.isFinite(insertedDistance) || insertedDistance <= 0) return 0;
        const softTipLength = this.type === CATHETER_TYPE_BERENSTEIN
            ? BERENSTEIN_XPBD_SOFT_TIP_LENGTH
            : XPBD_SOFT_TIP_LENGTH;
        const transitionLength = this.type === CATHETER_TYPE_BERENSTEIN
            ? BERENSTEIN_XPBD_SOFT_TIP_TRANSITION_LENGTH
            : XPBD_SOFT_TIP_TRANSITION_LENGTH;
        const softTipStart = Math.max(
            this.#sheathSupportEnd(),
            this.progress - softTipLength
        );
        return smoothstep(
            softTipStart,
            softTipStart + transitionLength,
            insertedDistance
        );
    }

    #distalShapeLength(freeLength) {
        const naturalLength = this.type === CATHETER_TYPE_BERENSTEIN
            ? BERENSTEIN_TIP_SHAPE_LENGTH
            : DISTAL_RELEASE_LENGTH;
        return Math.min(freeLength, naturalLength);
    }

    #unsupportedCatheterScale(distance) {
        if (this.guidewireInserted <= MIN_GUIDE_SUPPORT) return SOLO_CATHETER_STRAIGHTENING_SCALE;
        const supportFade = smoothstep(
            this.guidewireInserted - GUIDEWIRE_RECAPTURE_WINDOW,
            this.guidewireInserted + GUIDE_CAPTURE_TOLERANCE,
            distance ?? this.progress
        );
        return 1 + (SOLO_CATHETER_STRAIGHTENING_SCALE - 1) * supportFade;
    }

    #projectInsideVesselDetailed(point) {
        if (this.collisionMesh) {
            return this.#meshProjection(point, this.collisionMesh);
        }
        let best = null;
        for (const collider of this.vesselColliders) {
            const candidate = collider.type === 'sphere'
                ? this.#sphereProjection(point, collider)
                : this.#segmentProjection(point, collider);
            if (candidate.inside) {
                return {
                    point: point.clone(),
                    normal: candidate.normal,
                    collided: false
                };
            }
            if (!best || candidate.distance < best.distance) best = candidate;
        }
        return {
            point: best?.point || point.clone(),
            normal: best?.normal || new TypedVector3(1, 0, 0),
            collided: !!best
        };
    }

    #meshProjection(point, collision) {
        if (collision.meshCollider?.pointContact) {
            const contact = collision.meshCollider.pointContact(point, collision.clearance);
            return {
                point: contact.violation ? contact.target.clone() : point.clone(),
                normal: contact.normal?.clone?.() || new TypedVector3(1, 0, 0),
                collided: !!contact.violation
            };
        }

        const closest = new TypedVector3();
        const hit = collision.geometry.boundsTree.closestPointToPoint(point, { point: closest });
        const dist = hit?.distance ?? point.distanceTo(closest);
        const contactBand = Math.max(collision.clearance + FREE_NODE_SPACING * 1.5, collision.clearance * 2);
        if (dist > contactBand) {
            return {
                point: point.clone(),
                normal: new TypedVector3(1, 0, 0),
                collided: false
            };
        }

        const interior = typeof collision.interiorDirection === 'function'
            ? collision.interiorDirection(point, closest).clone()
            : point.clone().sub(closest);
        if (interior.lengthSq() < 1e-8) interior.set(1, 0, 0);
        interior.normalize();
        const insideDepth = point.clone().sub(closest).dot(interior);

        if (insideDepth >= collision.clearance) {
            return {
                point: point.clone(),
                normal: interior.clone().multiplyScalar(-1),
                collided: false
            };
        }

        return {
            point: closest.clone().addScaledVector(interior, collision.clearance),
            normal: interior.clone().multiplyScalar(-1),
            collided: true
        };
    }

    #segmentProjection(point, seg) {
        const rel = new TypedVector3().subVectors(point, seg.start);
        const axial = clamp(rel.dot(seg.dir), 0, seg.length);
        const center = seg.start.clone().addScaledVector(seg.dir, axial);
        const radial = new TypedVector3().subVectors(point, center);
        const radialDist = radial.length();
        const radius = Math.max(0.6, seg.radius - CONTACT_CLEARANCE);
        const inside = radialDist <= radius;
        const normal = radialDist > 1e-6 ? radial.multiplyScalar(1 / radialDist) : this.#fallbackNormal(seg.dir);
        if (inside) return { inside, point: point.clone(), distance: 0, normal };
        const projected = center.addScaledVector(normal, radius);
        return { inside: false, point: projected, distance: point.distanceTo(projected), normal };
    }

    #sphereProjection(point, sphere) {
        const radial = new TypedVector3().subVectors(point, sphere.center);
        const dist = radial.length();
        const radius = Math.max(0.6, sphere.radius - CONTACT_CLEARANCE);
        const inside = dist <= radius;
        const normal = dist > 1e-6 ? radial.multiplyScalar(1 / dist) : new TypedVector3(1, 0, 0);
        if (inside) return { inside, point: point.clone(), distance: 0, normal };
        const projected = sphere.center.clone().addScaledVector(normal, radius);
        return { inside: false, point: projected, distance: point.distanceTo(projected), normal };
    }

    #fallbackNormal(dir) {
        const helper = Math.abs(dir.y) < 0.85 ? new TypedVector3(0, 1, 0) : new TypedVector3(1, 0, 0);
        return new TypedVector3().crossVectors(dir, helper).normalize();
    }

    #buildVesselColliders(vessel) {
        if (!vessel?.segments) return [];
        const colliders = [];
        const nodeMap = new Map();
        const nodeKey = p => `${p.x.toFixed(5)},${p.y.toFixed(5)},${p.z.toFixed(5)}`;
        const rememberNode = (point, radius) => {
            const key = nodeKey(point);
            const existing = nodeMap.get(key);
            nodeMap.set(key, {
                point,
                radius: existing ? Math.max(existing.radius, radius) : radius
            });
        };

        for (const source of vessel.segments) {
            const start = nodePosition(source.start);
            const end = nodePosition(source.end);
            const axis = new TypedVector3().subVectors(end, start);
            const length = axis.length();
            if (length < 1e-6) continue;
            const dir = axis.multiplyScalar(1 / length);
            colliders.push({
                type: 'segment',
                start,
                end,
                dir,
                length,
                radius: source.radius || vessel.radius || 10
            });
            rememberNode(source.end, source.radius || vessel.radius || 10);
            if (!source.isSheath) rememberNode(source.start, source.radius || vessel.radius || 10);
        }

        for (const { point, radius } of nodeMap.values()) {
            colliders.push({
                type: 'sphere',
                center: nodePosition(point),
                radius
            });
        }
        return colliders;
    }

    #buildSheathPath(sheath) {
        if (!sheath?.start || !sheath?.end) return null;
        const start = nodePosition(sheath.start);
        const end = nodePosition(sheath.end);
        const dir = new TypedVector3().subVectors(end, start);
        const length = dir.length();
        if (length < 1e-6) return null;
        dir.multiplyScalar(1 / length);
        return { start, end, dir, length };
    }

    #sheathSupportEnd() {
        if (!this.sheathPath) return 0;
        return Math.min(this.progress, this.sheathPath.length);
    }

    #sampleSheathPath(insertedDistance, out = new TypedVector3()) {
        if (!this.sheathPath) return null;
        const d = clamp(insertedDistance, 0, this.sheathPath.length);
        return out.copy(this.sheathPath.start).addScaledVector(this.sheathPath.dir, d);
    }

    #recordGuidewirePath(targetDistance) {
        const sheathLength = this.sheathPath?.length || 0;
        if (targetDistance <= sheathLength + 0.5) return;
        if (!this.pathSamples.length) {
            this.#appendPathSample(sheathLength);
        }
        let distance = this.#pathEndDistance();
        while (distance + this.pathSpacing < targetDistance) {
            distance += this.pathSpacing;
            this.#appendPathSample(distance);
        }
        if (targetDistance > this.#pathEndDistance() + 0.5) {
            this.#appendPathSample(targetDistance);
        }
    }

    #appendPathSample(distance) {
        const index = this.pathSamples.length;
        let sample = this._pathSamplePool[index];
        if (!sample) {
            sample = { distance: 0, point: new TypedVector3() };
            this._pathSamplePool[index] = sample;
        }
        sample.distance = distance;
        this.#sampleGuidewire(distance, sample.point);
        this.pathSamples[index] = sample;
        return sample;
    }

    #refreshGuidewirePath(targetDistance) {
        const sheathLength = this.sheathPath?.length || 0;
        if (targetDistance <= sheathLength + 0.5) return;
        this.#recordGuidewirePath(targetDistance);
        for (let index = 0; index < this.pathSamples.length; index++) {
            const sample = this.pathSamples[index];
            if (sample.distance <= sheathLength + 0.5 || sample.distance > targetDistance + 0.5) continue;
            this.#sampleGuidewire(sample.distance, sample.point);
        }
    }

    #trimPath(maxDistance) {
        const sheathLength = this.sheathPath?.length || 0;
        const keepDistance = Math.max(maxDistance, sheathLength);
        while (this.pathSamples.length > 0 && this.pathSamples[this.pathSamples.length - 1].distance > keepDistance) {
            this.pathSamples.pop();
        }
        const end = this.pathSamples[this.pathSamples.length - 1];
        if (end && end.distance > maxDistance && end.distance > sheathLength) {
            end.distance = maxDistance;
        }
    }

    #pathEndDistance() {
        const last = this.pathSamples[this.pathSamples.length - 1];
        return Math.max(this.sheathPath?.length || 0, last ? last.distance : 0);
    }

    #sampleCatheterPath(insertedDistance, out = new TypedVector3()) {
        const sheathLength = this.sheathPath?.length || 0;
        if (this.sheathPath && insertedDistance < 0) {
            return out.copy(this.sheathPath.start).addScaledVector(this.sheathPath.dir, insertedDistance);
        }
        if (this.sheathPath && insertedDistance <= sheathLength + 0.5) {
            return this.#sampleSheathPath(insertedDistance, out);
        }
        if (!this.pathSamples.length) {
            const sheathTip = this.#sampleSheathPath(sheathLength, out);
            if (sheathTip) return sheathTip;
            return this.#sampleGuidewire(insertedDistance, out);
        }
        const target = clamp(insertedDistance, 0, this.#pathEndDistance());
        let prev = this.pathSamples[0];
        for (let i = 1; i < this.pathSamples.length; i++) {
            const next = this.pathSamples[i];
            if (next.distance >= target) {
                const t = clamp((target - prev.distance) / Math.max(1e-6, next.distance - prev.distance), 0, 1);
                return out.copy(prev.point).lerp(next.point, t);
            }
            prev = next;
        }
        return out.copy(prev.point);
    }

    #sampleCatheterCenterline(insertedDistance, state = this.#deploymentState()) {
        if (!this.freeNodes.length || insertedDistance <= state.supportEnd + 0.5) {
            return this.#sampleCatheterPath(insertedDistance);
        }

        const target = clamp(insertedDistance, this.freeNodes[0].distance ?? state.supportEnd, this.progress);
        let prev = this.freeNodes[0];
        for (let i = 1; i < this.freeNodes.length; i++) {
            const next = this.freeNodes[i];
            if ((next.distance ?? target) >= target) {
                const span = Math.max(1e-6, (next.distance ?? target) - (prev.distance ?? target));
                const t = clamp((target - (prev.distance ?? target)) / span, 0, 1);
                return prev.pos.clone().lerp(next.pos, t);
            }
            prev = next;
        }
        return prev.pos.clone();
    }

    #applyGuidewireReaction(insertedDistance, guideCorrection, reactionScale = 1) {
        if (reactionScale <= 0) return;
        if (!this.freeNodes.length || guideCorrection.lengthSq() < 1e-8) return;
        if (insertedDistance <= (this.freeNodes[0].distance ?? 0)) return;

        let prev = this.freeNodes[0];
        for (let i = 1; i < this.freeNodes.length; i++) {
            const next = this.freeNodes[i];
            const prevDistance = prev.distance ?? 0;
            const nextDistance = next.distance ?? prevDistance;
            if (insertedDistance <= nextDistance + 0.5) {
                const span = Math.max(1e-6, nextDistance - prevDistance);
                const t = clamp((insertedDistance - prevDistance) / span, 0, 1);
                const reaction = guideCorrection.clone().multiplyScalar(-GUIDEWIRE_REACTION_BLEND * reactionScale);
                const prevWeight = i === 1 ? 0 : 1 - t;
                const nextWeight = i === 1 ? 1 : t;
                prev.pos.addScaledVector(reaction, prevWeight);
                next.pos.addScaledVector(reaction, nextWeight);
                prev.vel.addScaledVector(reaction, 0.18 * prevWeight);
                next.vel.addScaledVector(reaction, 0.18 * nextWeight);
                return;
            }
            prev = next;
        }

        const tip = this.freeNodes[this.freeNodes.length - 1];
        tip.pos.addScaledVector(guideCorrection, -GUIDEWIRE_REACTION_BLEND * reactionScale);
        tip.vel.addScaledVector(guideCorrection, -0.18 * reactionScale);
    }

    #catheterPlaneNormal(tangent, beforeTip, beforePlane, out = new TypedVector3()) {
        const previousTangent = this._planePreviousTangent.subVectors(beforeTip, beforePlane);
        if (previousTangent.lengthSq() > 1e-5) {
            previousTangent.normalize();
            const curvature = this._planeCurvature.subVectors(tangent, previousTangent);
            curvature.addScaledVector(tangent, -curvature.dot(tangent));
            if (curvature.lengthSq() > 1e-5) return out.copy(curvature).normalize();
        }

        const useY = Math.abs(tangent.y) < 0.85;
        const helper = this._planeHelper.set(useY ? 0 : 1, useY ? 1 : 0, 0);
        return out
            .crossVectors(tangent, helper)
            .cross(tangent)
            .normalize();
    }

    #sampleGuidewire(insertedDistance, out = new TypedVector3()) {
        const tailProgress = this.tailProgressRef();
        const nodes = this.wire.nodes;
        const continuousIndex = clamp(
            (insertedDistance + this.guidewireLength - tailProgress) / this.segmentLength,
            0,
            nodes.length - 1
        );
        const index = Math.min(nodes.length - 2, Math.floor(continuousIndex));
        const t = continuousIndex - index;
        const a = nodes[index];
        const b = nodes[index + 1];
        return out.set(
            a.x + (b.x - a.x) * t,
            a.y + (b.y - a.y) * t,
            a.z + (b.z - a.z) * t
        );
    }

    #nodeInsertedCoordinate(index, tailProgress) {
        return this.segmentLength * index - this.guidewireLength + tailProgress;
    }

    #normalizeType(type) {
        return type === CATHETER_TYPE_BERENSTEIN || type === 'bernstein'
            ? CATHETER_TYPE_BERENSTEIN
            : CATHETER_TYPE_PIGTAIL;
    }
}
