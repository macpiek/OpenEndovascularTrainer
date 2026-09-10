import { transportCatheterThroughSheath } from './physics/catheterSheathTransport.js';
import * as THREE from 'three';
import { clamp, smoothstep } from './mathUtils.js';
import {
    PIGTAIL_CATHETER_INNER_RADIUS_MM,
    PIGTAIL_CATHETER_RADIUS_MM,
    PIGTAIL_CATHETER_RENDER_RADIUS_MM
} from './toolDimensions.js';
import { catheterMaterialProfile, PIGTAIL_NATURAL_ARC_LENGTH_MM, PIGTAIL_NATURAL_RADIUS_MM, PIGTAIL_NATURAL_TURNS } from './physics/catheterMaterialProfile.js';
import { applyKirchhoffMaterialProfile } from './physics/applyKirchhoffMaterialProfile.js';
import { applyProximalTwistBoundary } from './physics/kirchhoffOrientationBoundary.js';
import { updateSmoothTubeGeometry } from './smoothTubeGeometry.js';
import { getCompositeJointRenderPath } from './compositeJointRenderPath.js';

const CATHETER_RADIUS = PIGTAIL_CATHETER_RADIUS_MM;
const PIGTAIL_RADIUS = PIGTAIL_NATURAL_RADIUS_MM;
const PIGTAIL_TURNS = PIGTAIL_NATURAL_TURNS;
const PIGTAIL_ARC_LENGTH = PIGTAIL_NATURAL_ARC_LENGTH_MM;
const CATHETER_TYPE_PIGTAIL = 'pigtail';
const CATHETER_TYPE_BERENSTEIN = 'berenstein';
const CATHETER_TYPE_SIM1 = 'sim1';

const STRAIGHT_EXIT_LENGTH = 16;
const DISTAL_RELEASE_LENGTH = STRAIGHT_EXIT_LENGTH + PIGTAIL_ARC_LENGTH;
const MIN_GUIDE_SUPPORT = 18;
const GUIDE_CAPTURE_TOLERANCE = 4;
const DEFAULT_PATH_SPACING = 4;
const DEFAULT_FREE_NODE_SPACING = 3.2;

const PATH_RELAXATION_PASSES = 3;
const PATH_STRAIGHTENING_SPANS = [2, 4, 8];
const PATH_STRAIGHTENING = 0.24;
const PATH_LONG_SPAN_STRAIGHTENING = 0.085;
const PATH_MAX_BEND_ANGLE = 68 * Math.PI / 180;
const PATH_BEND_LIMIT_STRENGTH = 0.42;
const PATH_MAX_RELAX_STEP = 1.15;

const XPBD_GUIDEWIRE_PATH_SEED_BLEND = 0.18;
const PIGTAIL_RELEASE_CURL_START = 0.42;
const PIGTAIL_RELEASE_CURL_RATE = 2.4;

// Finite angular compliance represents the catheter's bending stiffness.
// With a hard intrinsic-turn constraint and a hard wall constraint there is
// no static solution at a loaded bifurcation, so the two projections chatter.
// Compliance lets their impulses reach a true force-balanced equilibrium.

const SHAPE_RECOVERY_RATE = 2.6;
const SHAPE_RECAPTURE_RATE = 3.2;

const SOLO_XPBD_SHAFT_MAX_BEND_ANGLE = 34.5;

// The natural 7.2 mm loop turns by about 31.8 degrees per 4 mm segment.
// Keep fold protection above that rest angle so it does not intermittently
// clamp and release the very curvature that shape memory is trying to form.
// The normalized 7.2 mm preform peaks at roughly 33.3 degrees per 4 mm
// Voronoi cell. A small guard above that natural angle prevents a discrete
// hinge from folding more tightly than the manufactured loop.
const PIGTAIL_XPBD_SOFT_TIP_MAX_BEND_ANGLE = 33.7;
const BERENSTEIN_XPBD_SOFT_TIP_MAX_BEND_ANGLE = 24;

const XPBD_SHAPE_ACTIVATION_LENGTH = 10;
const XPBD_MIN_SHAPE_WEIGHT = 0.025;
const XPBD_SOFT_TIP_LENGTH = PIGTAIL_ARC_LENGTH;
const XPBD_SOFT_TIP_TRANSITION_LENGTH = 8;
const BERENSTEIN_XPBD_SOFT_TIP_LENGTH = 24;
const BERENSTEIN_XPBD_SOFT_TIP_TRANSITION_LENGTH = 12;

// This node is an Eulerian sample of the introducer valve, not a material
// point tethered by a spring. It must stay exactly on the valve while material
// flows through it; finite compliance lets the entire loaded shaft retreat,
// stores axial compression and seeds a lateral buckle at the sheath outlet.
const XPBD_PROXIMAL_FEED_COMPLIANCE = 0;
const XPBD_RELEASE_STABILITY_LENGTH = 20;
const XPBD_IDLE_MAX_FRAME_DISPLACEMENT = 0.45;
// The coaxial catheter-wire span is a composite beam: both second moments
// resist the same local curvature. A barely 15% gain left that span visually
// indistinguishable from either tool alone; a twofold local stiffness keeps
// the supported shaft load-bearing while the 6 mm taper avoids a hinge where
// the wire ends.

const PIGTAIL_XPBD_FEED_POST_STABILIZATION_PASSES = 4;
const PIGTAIL_XPBD_SOLO_FEED_POST_STABILIZATION_PASSES = 4;
const PIGTAIL_XPBD_WITHDRAW_POST_STABILIZATION_PASSES = 4;
const PIGTAIL_XPBD_IDLE_SHAPE_STABILIZATION_PASSES = 4;

export const CATHETER_PROXIMAL_LOADING_SUPPORT_LENGTH_MM = 90;
const EXTERNAL_CATHETER_VISIBLE_LENGTH =
    CATHETER_PROXIMAL_LOADING_SUPPORT_LENGTH_MM;
const CATHETER_ADVANCE_SPEED = 52;
const CATHETER_WITHDRAW_SPEED = 32;
const ROTATION_SPEED = Math.PI * 0.9;

const TIP_MARKER_LENGTH = 2.4;
// The radiopaque band is embedded in the catheter wall. It changes X-ray
// attenuation, not the device's outer diameter.
const TIP_MARKER_RADIUS = PIGTAIL_CATHETER_RENDER_RADIUS_MM;
const PIGTAIL_INJECTION_PORT_RADIUS_MM = 0.22;
const PIGTAIL_INJECTION_PORT_OFFSETS_MM = Object.freeze([3, 6, 9, 12, 15, 18, 21, 24]);
const GOLDEN_ANGLE = Math.PI * (3 - Math.sqrt(5));
const XPBD_WALL_CONTACT_FIELDS = Object.freeze([
    'wallLambda',
    'wallActive',
    'wallT',
    'wallX',
    'wallY',
    'wallZ',
    'wallNormalX',
    'wallNormalY',
    'wallNormalZ',
    'wallBranchId',
    'wallFaceIndex',
    'wallInsideClearance',
    'wallCapsuleSampleCount',
    'wallGap',
    'wallQueryStartX',
    'wallQueryStartY',
    'wallQueryStartZ',
    'wallQueryEndX',
    'wallQueryEndY',
    'wallQueryEndZ'
]);

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
    constructor({
        wire,
        segmentLength,
        guidewireLength,
        tailProgressRef,
        vessel = null,
        maxLength = 1000,
        physicsSpacing = DEFAULT_PATH_SPACING,
        retainMaterialTip = false
    }) {
        this.wire = wire;
        this.segmentLength = segmentLength;
        this.guidewireLength = guidewireLength;
        this.tailProgressRef = tailProgressRef;
        this.vessel = vessel;
        // The Joint material chart must include the physical distal label
        // even during the first fractional millimetre of insertion.
        this.retainMaterialTip = retainMaterialTip;
        this.minimumLayoutSegmentLength = retainMaterialTip ? 0 : 0.5;
        this._kirchhoffBoundaryPreferredD1 = new TypedVector3(1, 0, 0);
        this._kirchhoffBoundaryFrame = {};
        this._kirchhoffBoundaryOptions = {
            twist: 0,
            segment: 0,
            preferredD1: this._kirchhoffBoundaryPreferredD1,
            compliance: 0,
            out: this._kirchhoffBoundaryFrame
        };
        if (vessel?.sheath) {
            const axisX = vessel.sheath.end.x - vessel.sheath.start.x;
            const axisY = vessel.sheath.end.y - vessel.sheath.start.y;
            const axisZ = vessel.sheath.end.z - vessel.sheath.start.z;
            const axisLength = magnitude3(axisX, axisY, axisZ) || 1;
            const tangentX = axisX / axisLength;
            const tangentY = axisY / axisLength;
            const tangentZ = axisZ / axisLength;
            // Match the old manufactured bend plane once at the handle. The
            // resulting director is material data; it is not rebuilt from the
            // deformed distal loop.
            let normalX = -tangentX * tangentZ;
            let normalY = -tangentY * tangentZ;
            let normalZ = 1 - tangentZ * tangentZ;
            let normalLength = magnitude3(normalX, normalY, normalZ);
            if (normalLength < 1e-6) {
                normalX = 1 - tangentX * tangentX;
                normalY = -tangentY * tangentX;
                normalZ = -tangentZ * tangentX;
                normalLength = magnitude3(normalX, normalY, normalZ) || 1;
            }
            normalX /= normalLength;
            normalY /= normalLength;
            normalZ /= normalLength;
            this._kirchhoffBoundaryPreferredD1.set(
                tangentY * normalZ - tangentZ * normalY,
                tangentZ * normalX - tangentX * normalZ,
                tangentX * normalY - tangentY * normalX
            ).normalize();
        }

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
        this.type = CATHETER_TYPE_PIGTAIL;
        this.pathSpacing = Math.max(1, physicsSpacing);
        this.freeNodeSpacing = this.pathSpacing *
            (DEFAULT_FREE_NODE_SPACING / DEFAULT_PATH_SPACING);
        this.pathSamples = [];
        this._pathSamplePool = Array.from(
            { length: Math.ceil(maxLength / this.pathSpacing) + 4 },
            () => ({ distance: 0, point: new TypedVector3() })
        );
        this.freeNodes = [];
        this._nextFreeNodes = [];
        this._freeNodePool = [];
        this._freeNodeEpoch = 0;
        this.freeRestDistances = new Float64Array(
            Math.ceil(maxLength / this.freeNodeSpacing) + 2
        );
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
        this.tipMarker.userData.radiopaque = true;
        this.tipMarker.userData.outerRadiusMm = TIP_MARKER_RADIUS;
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
        this._xpbdLayoutCount = 0;
        this._xpbdProgress = 0;
        this._xpbdYieldsToWall = false;
        this._guidewireRelease = 1;
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
        this._xpbdProximalFeedControlIndex = -1;
        this._xpbdProximalFeedTarget = new TypedVector3();


        this.shaftStiffnessScale = 1;
        this.tipStiffnessScale = 1;
        this._kirchhoffDiscretization = {};
        this._kirchhoffMaterialOptions = {
            activeStart: 0,
            activeEnd: 1,
            materialCoordinates: null,
            tipCoordinate: 0,
            shaftStiffnessScale: 1,
            tipStiffnessScale: 1,
            discretizationOut: this._kirchhoffDiscretization
        };
    }

    setStiffnessScales({ shaftStiffnessScale = 1, tipStiffnessScale = 1 } = {}) {
        if (!Number.isFinite(shaftStiffnessScale) || shaftStiffnessScale <= 0) {
            throw new RangeError('Catheter shaft stiffness scale must be finite and positive');
        }
        if (!Number.isFinite(tipStiffnessScale) || tipStiffnessScale <= 0) {
            throw new RangeError('Catheter tip stiffness scale must be finite and positive');
        }
        if (
            this.shaftStiffnessScale === shaftStiffnessScale &&
            this.tipStiffnessScale === tipStiffnessScale
        ) return this;
        this.shaftStiffnessScale = shaftStiffnessScale;
        this.tipStiffnessScale = tipStiffnessScale;
        this._kirchhoffMaterialOptions.shaftStiffnessScale =
            shaftStiffnessScale;
        this._kirchhoffMaterialOptions.tipStiffnessScale =
            tipStiffnessScale;
        this.physicsBody?.wake();
        return this;
    }

    getStiffnessScales() {
        return {
            shaft: this.shaftStiffnessScale,
            tip: this.tipStiffnessScale
        };
    }

    setType(type) {
        const nextType = this.#normalizeType(type);
        if (this.type === nextType) return;
        this.#releaseXpbdProximalFeed();
        this.type = nextType;
        this.#clearFreeNodes();
        this.freeRestDistanceCount = 0;
        this.freeLength = 0;
        this._physicsStepIndex = 0;
        this.rotationCommand = 0;
        this._pendingXpbdRotation = 0;
        this.physicsLumenStartNode = 0;
        this._xpbdProgress = this.progress;
        this._xpbdYieldsToWall = false;

        this.updateMesh();
    }

    dispose() {
        this.#releaseXpbdProximalFeed();
        this.shaftMesh.geometry?.dispose?.();
        this.tipMarker.geometry?.dispose?.();
        this.material.dispose();
        this.tipMarkerMaterial.dispose();
    }

    reset() {
        this.#releaseXpbdProximalFeed();
        this.progress = 0;
        this.guidewireInserted = 0;
        this.previousGuidewireInserted = 0;
        this.guidewireDelta = 0;
        this.motionCommand = 0;
        this.rotationCommand = 0;
        this.rotation = 0;
        this._pendingXpbdRotation = 0;
        this.pathSamples.length = 0;
        this.#clearFreeNodes();
        this.freeRestDistanceCount = 0;
        this.freeLength = 0;
        this._physicsStepIndex = 0;
        this.physicsActiveCount = 0;
        this._xpbdLayoutCount = 0;
        this._xpbdProgress = 0;
        this._xpbdYieldsToWall = false;

        this._guidewireRelease = 1;
        this.updateMesh();
        return this;
    }

    syncXpbdBody(body, options = {}) {
        if (!this.vessel?.sheath) return this.#syncPreviewBody(body, options);
        const reset = this.physicsBody !== body || this.physicsActiveCount < 2;
        this.#releaseXpbdProximalFeed();
        const feed = transportCatheterThroughSheath(body, this.vessel.sheath,
            this.progress, this._feedDt ?? 1 / 120, this._sheathFeed ??= {}, {reset});
        this.physicsBody = body;
        this.physicsActiveCount = body.count;
        this.physicsLumenStartNode = feed.lumenStart;
        this.physicsLumenOrigin = feed.lumenOrigin;
        body.nodeRadius.fill(CATHETER_RADIUS);
        const material = this._kirchhoffMaterialOptions;
        material.activeStart = body.activeStart;
        material.activeEnd = body.activeEnd;
        material.materialCoordinates = body.materialCoordinate;
        material.tipCoordinate = this.progress;
        applyKirchhoffMaterialProfile(body, this.type, material);
        this._kirchhoffBoundaryOptions.twist = this.rotation;
        this._kirchhoffBoundaryOptions.segment = body.activeStart;
        applyProximalTwistBoundary(body, this._kirchhoffBoundaryOptions);
        this.#applyStandaloneKirchhoffRuntime(body);
        if (this.guidewireInserted > MIN_GUIDE_SUPPORT) {
            const moving = Math.abs(this.motionCommand) > 1e-6 ||
                Math.abs(this.guidewireDelta) > 1e-5 || Math.abs(this.rotationCommand) > 1e-6;
            body.projectionVelocityRetention = moving ? 1 : 0.005;
            body.toolProjectionVelocityRetention = 0;
        }
        this._pendingXpbdRotation = 0;
        this._xpbdProgress = this.progress;
        return body.count;
    }

    #syncPreviewBody(body, {

        restLengthSlewLimit = 0.5,
        bendChordSlewLimit = 1
    } = {}) {
        const points = this.#buildCenterline();
        const count = Math.min(this._centerlinePointCount, body.count);
        const initializeKirchhoffFrames = this.physicsBody !== body || this.physicsActiveCount < 2;
        let progressDelta = this.progress - this._xpbdProgress;
        if (this.physicsBody !== body || !this._xpbdLayoutX || this._xpbdLayoutX.length !== body.count) {
            if (this.physicsBody && this.physicsBody !== body) {
                this.#releaseXpbdProximalFeed(this.physicsBody);
            }
            this._xpbdBaseWallProjectionVelocityRetention =
                body.wallProjectionVelocityRetention;
            this._xpbdBaseSweptContactPreserveTangentialMotion =
                body.sweptContactPreserveTangentialMotion;
            this._xpbdLayoutX = new Float64Array(body.count);
            this._xpbdLayoutY = new Float64Array(body.count);
            this._xpbdLayoutZ = new Float64Array(body.count);

            this._xpbdLayoutCount = 0;
            this.physicsActiveCount = 0;
            this._xpbdProgress = this.progress;
            progressDelta = 0;
            this._xpbdYieldsToWall = false;

        }
        this.physicsBody = body;
        body.postStabilizationPasses =
            this.motionCommand > 1e-6 || Math.abs(this.guidewireDelta) > 1e-5
                ? this.guidewireInserted <= MIN_GUIDE_SUPPORT
                    ? PIGTAIL_XPBD_SOLO_FEED_POST_STABILIZATION_PASSES
                    : PIGTAIL_XPBD_FEED_POST_STABILIZATION_PASSES
                : this.motionCommand < -1e-6
                ? PIGTAIL_XPBD_WITHDRAW_POST_STABILIZATION_PASSES
                : PIGTAIL_XPBD_IDLE_SHAPE_STABILIZATION_PASSES;
        body.postStabilizationMinPasses = 2;
        body.postStabilizationTolerance = 0.01;
        body.postStabilizationSettledPasses = 2;
        {
            const idle = Math.abs(this.motionCommand) <= 1e-6 &&
                Math.abs(this.guidewireDelta) <= 1e-5;
            body.finalStructuralClosurePasses = idle ? 16 : 8;

        }
        // Pigtail shape memory is represented by material curvature below.
        // Re-solving an additional world-space positional shape in the idle
        // passes creates a second elastic potential with a moving reference.

        // Rigid distal length transport is a topology/feed repair. Applying
        // it while the operator is idle turns elastic shape recovery into an
        // artificial axial pull of the complete catheter.
        body.distalLengthTransportMaxCorrection =
            Math.abs(this.motionCommand) > 1e-6 &&
            !(
                this.type === CATHETER_TYPE_PIGTAIL &&
                this.progress > this.guidewireInserted + 0.5
            )
                ? 1.25
                : this.motionCommand < -1e-6
                ? 0.2
                : 0.2;
        if (count < 2) {
            body.setActiveRange(0, 1);
            body.setCollisionRange(0, -1);
            this.physicsActiveCount = 0;
            this._xpbdLayoutCount = 0;
            this._xpbdProgress = this.progress;
            this._xpbdYieldsToWall = false;
            this._pendingXpbdRotation = 0;

            {
                body.clearProximalOrientationControl?.();
            }
            this.#releaseXpbdProximalFeed(body);
            return 0;
        }

        const previousCount = this.physicsActiveCount;
        const soloXpbd = this.guidewireInserted <= MIN_GUIDE_SUPPORT;
        const unsupportedShapeLength = this.#naturalShapeLength();
        const hasLocallyUnsupportedShaft =
            this.progress > this.guidewireInserted +
                GUIDE_CAPTURE_TOLERANCE + unsupportedShapeLength;
        const hasReleasedPreform =
            this.progress > this.guidewireInserted + 0.5;
        const preserveUnsupportedTopology = soloXpbd || hasReleasedPreform;
        body.postStabilizeBending =
            soloXpbd || hasLocallyUnsupportedShaft || hasReleasedPreform;
        // Contact and length projections should not become a fresh inertial
        // kick on the next frame. A catheter in blood and against a vessel is
        // strongly overdamped, especially after the operator releases feed.
        // Iterative Kirchhoff/length/contact corrections move the rod toward
        // its constrained equilibrium, but they are not inertial momentum.
        // This was already handled for a released/unsupported catheter. Apply
        // the same quasi-static reconstruction whenever the operator is idle
        // while a guidewire supports the catheter; otherwise an equilibrated
        // lumen pair slowly accumulates projection energy until it waves.
        body.projectionVelocityRetention = soloXpbd
            ? (Math.abs(this.motionCommand) > 0 ? 1 : 0.005)
            : (
                Math.abs(this.motionCommand) > 1e-6 ||
                Math.abs(this.guidewireDelta) > 1e-5 ||
                Math.abs(this.rotationCommand) > 1e-6 ||
                Math.abs(this._pendingXpbdRotation) > 1e-6
                    ? 1
                    : 0.005
            );
        let insertedIndex = -1;
        let topologyChanged = false;
        let topologyDelta = 0;
        let topologyIndex = -1;
        if (
            previousCount > 0 &&
            this._xpbdLayoutCount === previousCount &&
            count === previousCount + 1
        ) {
            // New catheter material enters at the sheath outlet. Appending a
            // node at the distal tip changes material identity and makes later
            // withdrawal delete the physical tip from a recorded route.
            insertedIndex = this.freeNodes.length >= 2
                ? this.#xpbdUnsupportedEntryIndex(count, soloXpbd)
                : this.#xpbdInsertedPointIndex(points, count, previousCount);
            if (
                typeof process !== 'undefined' &&
                process.env?.OET_TRACE_AORTA_FOLD === '1' &&
                this.progress >= 99.9 && this.progress <= 100.2
            ) {
                console.log('catheter topology before insert', {
                    progress: this.progress,
                    count,
                    previousCount,
                    insertedIndex,
                    physicsLumenStartNode: this.physicsLumenStartNode,
                    freeNodes: this.freeNodes.map(node => ({
                        distance: node.distance,
                        xpbdIndex: node._xpbdIndex,
                        position: [node.pos.x, node.pos.y, node.pos.z]
                    })),
                    body: Array.from(
                        { length: Math.min(previousCount, 8) },
                        (_, offset) => {
                            const index = previousCount - Math.min(previousCount, 8) + offset;
                            return {
                                index,
                                position: [body.x[index], body.y[index], body.z[index]],
                                velocity: [
                                    body.velocityX[index],
                                    body.velocityY[index],
                                    body.velocityZ[index]
                                ]
                            };
                        }
                    )
                });
            }
            for (let index = count - 1; index > insertedIndex; index--) {
                this.#copyXpbdNodeState(body, index, index - 1);
            }
            this.#initializeInsertedXpbdNode(body, points, insertedIndex, count);
            topologyChanged = true;
            topologyDelta = 1;
            topologyIndex = insertedIndex;
        } else if (
            count > 1 &&
            this._xpbdLayoutCount === previousCount &&
            count === previousCount - 1
        ) {
            // Retraction removes material at the sheath outlet and preserves
            // the distal state. Removing the last node is equivalent to
            // replaying and trimming the insertion path.
            const removedIndex = this.freeNodes.length >= 2
                ? this.#xpbdUnsupportedEntryIndex(count, soloXpbd)
                : this.#xpbdRemovedPointIndex(points, count, previousCount);
            for (let index = removedIndex; index < count; index++) {
                this.#copyXpbdNodeState(body, index, index + 1);
            }
            topologyChanged = true;
            topologyDelta = -1;
            topologyIndex = removedIndex;
        }
        const activelyFeeding = Math.abs(this.motionCommand) > 1e-6;
        // During manipulation, kinetic Coulomb friction follows the normal
        // load generated in this step: a tangent Pigtail can slide and open
        // against the bifurcation instead of inheriting an old contact load.
        // Once the operator releases it, retain the decaying contact multiplier
        // as a small static-friction/damping term so two competing elastic and
        // wall projections converge to a quiet equilibrium.
        body.wallFrictionUsesCurrentLoad =
            this.type === CATHETER_TYPE_PIGTAIL || soloXpbd;
        body.wallFrictionUsesSmoothedLoad = false;
        body.setActiveRange(0, count - 1);
        if (
            topologyChanged ||
            Math.abs(this.motionCommand) > 0 ||
            Math.abs(this.guidewireDelta) > 1e-5
        ) {
            body.wake();
        }
        if (topologyChanged) {
            if (
                this.type === CATHETER_TYPE_PIGTAIL &&
                this.motionCommand > 1e-6 &&
                soloXpbd
            ) {
                // A newly inserted material node starts from a continuous
                // interpolated pose. Do not apply the full steady-feed polish
                // in that same frame: repeated whole-rod length projections
                // otherwise turn the harmless local split into a distal jump.
                body.postStabilizationPasses =
                    PIGTAIL_XPBD_SOLO_FEED_POST_STABILIZATION_PASSES;
            }

            // Positions and material frames form one Lagrangian state. Every
            // insertion/removal remaps both, also while the guidewire supports
            // the catheter. Constitutive targets may be rebuilt below, but a
            // live frame may never stay attached to an index whose node just
            // moved to a different material interval.
            this.#remapXpbdStructuralState(
                body,
                points,
                topologyIndex,
                topologyDelta,
                count
            );
            // Preserve warm-started contacts for material that kept its
            // physical identity. Only the segment split or merged at the
            // sheath outlet needs to lose its cached contact.
            if (preserveUnsupportedTopology) {
                this.#remapXpbdWallContacts(
                    body,
                    topologyIndex,
                    topologyDelta,
                    count
                );
            } else {
                this.#resetXpbdWallContacts(body);
            }
            if (!preserveUnsupportedTopology) {
                for (let index = 0; index < count - 1; index++) {
                    const materialLength = Math.abs(
                        (this._centerlineDistances[index + 1] ?? 0) -
                        (this._centerlineDistances[index] ?? 0)
                    );
                    body.restLength[index] = Math.max(this.minimumLayoutSegmentLength, materialLength);
                }
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
        this.#applyPendingXpbdRotation(body, count);
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

        if (bodyTouchesWall) this._xpbdYieldsToWall = true;
        const releasedPigtail =
            this.type === CATHETER_TYPE_PIGTAIL &&
            this.progress > this.guidewireInserted + 0.5;
        const pigtailIdle =
            Math.abs(this.motionCommand) <= 1e-6 &&
            Math.abs(this.guidewireDelta) <= 1e-5;

        const localPigtailContactOwner =
            this.type === CATHETER_TYPE_PIGTAIL &&
            (
                releasedPigtail ||
                (soloXpbd && pigtailIdle)
            );
        const localPigtailShapeStart = localPigtailContactOwner
            ? Math.max(
                this.#sheathSupportEnd(),
                releasedPigtail ? this.guidewireInserted : 0,
                this.progress - PIGTAIL_ARC_LENGTH
            )
            : Infinity;
        for (let index = 0; index < count; index++) {
            const point = points[index];
            const insertedDistance = this._centerlineDistances[index] ?? Infinity;
            const shapeWeight = this.#xpbdShapeMemoryWeight(insertedDistance);
            const softTipWeight = this.#xpbdSoftTipWeight(insertedDistance);
            let idealShapePoint = point;
            if (shapeWeight > XPBD_MIN_SHAPE_WEIGHT) {
                for (let freeIndex = 1; freeIndex < this.freeNodes.length; freeIndex++) {
                    const freeNode = this.freeNodes[freeIndex];
                    if (freeNode._xpbdIndex !== index) continue;
                    idealShapePoint = freeNode.shapeTarget;
                    break;
                }
            }

            const newlyActivated = index === insertedIndex || (
                insertedIndex < 0 && index >= previousCount
            );
            if (newlyActivated && index !== insertedIndex) {
                if (previousCount > 0 && index > 0) {
                    const targetPrevious = points[index - 1];
                    let directionX;
                    let directionY;
                    let directionZ;
                    let targetLength;
                    if (index > 1) {
                        // A new distal material sample must be C1-continuous
                        // with the live rod. Seeding it from an analytical
                        // world-space shape makes a wall-deflected shaft meet
                        // a different tangent in one frame; the fold guard
                        // then launches the entire tip. Natural κ0 bends this
                        // continuation toward the preform after activation.
                        directionX = body.x[index - 1] - body.x[index - 2];
                        directionY = body.y[index - 1] - body.y[index - 2];
                        directionZ = body.z[index - 1] - body.z[index - 2];
                        targetLength = magnitude3(directionX, directionY, directionZ);
                    } else {
                        directionX = point.x - targetPrevious.x;
                        directionY = point.y - targetPrevious.y;
                        directionZ = point.z - targetPrevious.z;
                        targetLength = magnitude3(directionX, directionY, directionZ);
                    }
                    const restLength = Math.max(this.minimumLayoutSegmentLength, point.distanceTo(targetPrevious));
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
            body.nodeRadius[index] = CATHETER_RADIUS;
            const globalUnsupportedStiffness = 1 - smoothstep(
                0,
                MIN_GUIDE_SUPPORT,
                this.guidewireInserted
            );
            const localUnsupportedStiffness =
                smoothstep(
                    this.guidewireInserted + GUIDE_CAPTURE_TOLERANCE,
                    this.guidewireInserted + XPBD_RELEASE_STABILITY_LENGTH,
                    insertedDistance
                );
            const unsupportedStiffness = Math.max(
                globalUnsupportedStiffness,
                localUnsupportedStiffness
            );

            const materialShaftFoldLimit =
                catheterMaterialProfile(this.type).shaftFoldLimitDegrees ??
                body.maxBendAngle;
            const shaftMaxBendAngle = materialShaftFoldLimit +
                (
                    Math.min(
                        materialShaftFoldLimit,
                        SOLO_XPBD_SHAFT_MAX_BEND_ANGLE
                    ) - materialShaftFoldLimit
                ) * unsupportedStiffness;
            const softTipMaxBendAngle =
                catheterMaterialProfile(this.type).softTipMaxBendAngleDegrees ??
                (this.type === CATHETER_TYPE_BERENSTEIN
                    ? BERENSTEIN_XPBD_SOFT_TIP_MAX_BEND_ANGLE
                    : PIGTAIL_XPBD_SOFT_TIP_MAX_BEND_ANGLE);
            const naturalMaxBendAngle = shaftMaxBendAngle +
                (softTipMaxBendAngle - shaftMaxBendAngle) * softTipWeight;
            body.maxBendAngleByNode[index] = naturalMaxBendAngle;
            if (index > 0) {
                const desiredLength = Math.max(this.minimumLayoutSegmentLength, Math.abs(
                    (this._centerlineDistances[index] ?? 0) -
                    (this._centerlineDistances[index - 1] ?? 0)
                ));
                if (!this.retainMaterialTip && previousCount > 0 && restLengthSlewLimit > 0) {
                    body.restLength[index - 1] += clamp(
                        desiredLength - body.restLength[index - 1],
                        -restLengthSlewLimit,
                        restLengthSlewLimit
                    );
                } else {
                    body.restLength[index - 1] = desiredLength;
                }
            }

        }
        if (initializeKirchhoffFrames) {
            // The body was constructed before its live centerline existed.
            // Initialize only the current frames after the first sync; the
            // manufactured rest rotation still comes exclusively from the
            // material profile below.
            body.captureKirchhoffRestConfiguration({ captureRestRotation: false });
        }
        this.#applyKirchhoffMaterialShape(body, count);
        // The fractional material segment and a compliant outlet control
        // provide the actual proximal push. This is required both with and
        // without a guidewire; omitting it in the supported case lets distal
        // coupling pull the catheter backwards through the sheath.
        this.#stabilizeUnsupportedXpbdEntry(
            body,
            count,
            soloXpbd,
            collisionStart
        );
        body.setCollisionRange(collisionStart, count - 2);
        body.setSheathMaterialEndNode(collisionStart);
        // Keep the complete XPBD correction budget below a visible jump. This
        // is a timestep/CFL guard, not an extra force: insertion, rotation,
        // intrinsic curvature and wall contact still converge to the same
        // equilibrium over subsequent steps.
        const operatorIdle =
            Math.abs(this.motionCommand) <= 1e-6 &&
            Math.abs(this.guidewireDelta) <= 1e-5 &&
            Math.abs(this.rotationCommand) <= 1e-6 &&
            Math.abs(this._pendingXpbdRotation) <= 1e-6;
        body.maxFrameDisplacement = operatorIdle
            ? XPBD_IDLE_MAX_FRAME_DISPLACEMENT
            : Infinity;
        body.frameDisplacementStartNode = Math.max(
            body.activeStart,
            collisionStart
        );
        if (soloXpbd) {
            this.#applyStandaloneKirchhoffRuntime(body);
        } else {
            // Vessel contact is the same unilateral, zero-restitution
            // boundary with and without lumen support. Restoring the raw body
            // default here used to make adding a guidewire turn wall
            // projections into rebound velocity.
            body.wallProjectionVelocityRetention = 0;
            body.toolProjectionVelocityRetention = 0;
            body.sweptContactPreserveTangentialMotion =
                this._xpbdBaseSweptContactPreserveTangentialMotion ?? false;
        }
        this.physicsActiveCount = count;
        for (let index = 0; index < count; index++) {
            this._xpbdLayoutX[index] = points[index].x;
            this._xpbdLayoutY[index] = points[index].y;
            this._xpbdLayoutZ[index] = points[index].z;
        }
        this._xpbdLayoutCount = count;
        this._xpbdProgress = this.progress;
        if (
            typeof process !== 'undefined' &&
            process.env?.OET_TRACE_AORTA_FOLD === '1' &&
            this.progress >= 99.9 && this.progress <= 100.2
        ) {
            console.log('catheter topology after sync', {
                progress: this.progress,
                count,
                insertedIndex,
                collisionStart,
                body: Array.from(
                    { length: Math.min(count, 8) },
                    (_, offset) => {
                        const index = count - Math.min(count, 8) + offset;
                        return {
                            index,
                            material: body.materialCoordinate[index],
                            position: [body.x[index], body.y[index], body.z[index]],
                            previous: [
                                body.previousX[index],
                                body.previousY[index],
                                body.previousZ[index]
                            ],
                            velocity: [
                                body.velocityX[index],
                                body.velocityY[index],
                                body.velocityZ[index]
                            ]
                        };
                    }
                )
            });
        }
        return count;
    }

    #applyPendingXpbdRotation(body, count) {
        {
            // Kirchhoff rotation is a torsional boundary condition applied in
            // sync. Rotating x/previous/velocity would inject rigid motion and
            // rotate the constitutive kappa_0 axis in world space.
            this._pendingXpbdRotation = 0;
            return;
        }

    }

    #releaseXpbdProximalFeed(body = this.physicsBody) {
        if (body && this._xpbdProximalFeedControlIndex >= 0) {
            body.clearControlTarget(this._xpbdProximalFeedControlIndex);
            {
                body.setPinned(this._xpbdProximalFeedControlIndex, false);
            }
        }
        this._xpbdProximalFeedControlIndex = -1;
    }

    #applyStandaloneKirchhoffRuntime(body) {

        // A catheter without guidewire support is the same kind of free,
        // boundary-driven Kirchhoff rod as the guidewire. Keep its material
        // profile (diameter, mass, stiffness and intrinsic distal curvature),
        // but do not switch to a catheter-only solver schedule while feeding
        // or after the operator releases the control.
        body.postStabilizationPasses = 0;
        body.finalStructuralClosurePasses = 8;

        body.postStabilizeBending = false;
        body.projectionVelocityRetention = 1;
        body.distalProjectionVelocityRetention = 1;
        body.distalProjectionVelocityRetentionStartNode = Infinity;
        body.maxFrameDisplacement = Infinity;

        // Match the guidewire's unilateral wall-contact transport: discard
        // the normal displacement introduced by the projection while keeping
        // physical tangential motion. Friction magnitude remains a catheter
        // material property.
        body.wallProjectionVelocityRetention = 0;
        body.toolProjectionVelocityRetention = 1;
        body.sweptContactPreserveTangentialMotion = true;
        body.wallFrictionUsesCurrentLoad = false;
        body.wallFrictionUsesSmoothedLoad = false;
    }

    #stabilizeUnsupportedXpbdEntry(body, count) {
        // The fixed inlet feeds material through the changing rest-length field.
        const controlIndex = clamp(
            this.physicsLumenStartNode,
            body.activeStart,
            Math.min(body.activeEnd, count - 1)
        );
        if (!Number.isFinite(controlIndex) || controlIndex < 0) {
            this.#releaseXpbdProximalFeed(body);
            return;
        }
        if (
            this._xpbdProximalFeedControlIndex >= 0 &&
            this._xpbdProximalFeedControlIndex !== controlIndex
        ) {
            body.clearControlTarget(this._xpbdProximalFeedControlIndex);
            body.setPinned(this._xpbdProximalFeedControlIndex, false);
        }

        // This is the catheter's physical Eulerian boundary at the introducer
        // valve, not a distal path target. Keeping this one spatial inlet point
        // in the valve lets the changing material length create axial feed;
        // rod energy and contact remain solely responsible for the path beyond
        // the introducer.
        const target = this.#sampleCatheterPath(
            0,
            this._xpbdProximalFeedTarget
        );
        // This is an Eulerian boundary sample: catheter material passes
        // through it, but the valve location itself has zero translational
        // mobility. Treating it as an ordinary massive node lets global
        // inextensibility move the valve and then makes a later control
        // projection compress the adjacent segment again.
        body.setNodePosition(
            controlIndex,
            target.x,
            target.y,
            target.z
        );
        body.setPinned(controlIndex, true);
        body.setControlTarget(
            controlIndex,
            target.x,
            target.y,
            target.z,
            XPBD_PROXIMAL_FEED_COMPLIANCE
        );
        this._xpbdProximalFeedControlIndex = controlIndex;
    }

    #applyKirchhoffMaterialShape(body, count) {
        // Rest strain is a material property independent of support and current pose.

        for (let index = 0; index < count; index++) {
            body.materialCoordinate[index] = this._centerlineDistances[index];
        }

        const materialOptions = this._kirchhoffMaterialOptions;
        materialOptions.activeStart = body.activeStart;
        materialOptions.activeEnd = count - 1;
        materialOptions.materialCoordinates = body.materialCoordinate;
        materialOptions.tipCoordinate = this.progress;
        applyKirchhoffMaterialProfile(body, this.type, materialOptions);
        const boundaryOptions = this._kirchhoffBoundaryOptions;
        boundaryOptions.twist = this.rotation;
        boundaryOptions.segment = body.activeStart;
        applyProximalTwistBoundary(body, boundaryOptions);
    }

    #remapXpbdWallContacts(body, nodeIndex, topologyDelta, activeNodeCount) {
        const lastActiveSegment = Math.min(
            body.segmentCount - 1,
            activeNodeCount - 2
        );
        if (topologyDelta > 0) {
            for (
                let segment = lastActiveSegment;
                segment >= nodeIndex + 1;
                segment--
            ) {
                this.#copyXpbdWallContact(body, segment, segment - 1);
            }
            this.#clearXpbdWallContact(body, nodeIndex - 1);
            this.#clearXpbdWallContact(body, nodeIndex);
            return;
        }
        if (topologyDelta < 0) {
            for (
                let segment = Math.max(0, nodeIndex);
                segment <= lastActiveSegment;
                segment++
            ) {
                this.#copyXpbdWallContact(body, segment, segment + 1);
            }
            this.#clearXpbdWallContact(body, nodeIndex - 1);
            this.#clearXpbdWallContact(body, lastActiveSegment + 1);
        }
    }

    #copyXpbdWallContact(body, target, source) {
        if (
            target < 0 || target >= body.segmentCount ||
            source < 0 || source >= body.segmentCount
        ) return;
        for (const field of XPBD_WALL_CONTACT_FIELDS) {
            body[field][target] = body[field][source];
        }
    }

    #clearXpbdWallContact(body, segment) {
        if (segment < 0 || segment >= body.segmentCount) return;
        for (const field of XPBD_WALL_CONTACT_FIELDS) body[field][segment] = 0;
        body.wallBranchId[segment] = -1;
        body.wallFaceIndex[segment] = -1;
        body.wallGap[segment] = Infinity;
    }

    #resetXpbdWallContacts(body) {
        for (let segment = 0; segment < body.segmentCount; segment++) {
            this.#clearXpbdWallContact(body, segment);
        }
    }

    #xpbdUnsupportedEntryIndex(count, proximalFeed = false) {
        const supportEnd = this.#sheathSupportEnd();
        const entryThreshold = proximalFeed
            ? supportEnd + 0.25
            : supportEnd - 0.25;
        for (let index = 1; index < count; index++) {
            if ((this._centerlineDistances[index] ?? -Infinity) > entryThreshold) {
                return index;
            }
        }
        return Math.max(1, count - 1);
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
        body.maxBendAngleByNode[target] = body.maxBendAngleByNode[source];
        {
            body.materialCoordinate[target] = body.materialCoordinate[source];
        }

    }

    #copyXpbdMaterialFrameState(body, target, source) {
        {
            body.orientationX[target] = body.orientationX[source];
            body.orientationY[target] = body.orientationY[source];
            body.orientationZ[target] = body.orientationZ[source];
            body.orientationW[target] = body.orientationW[source];
            body.previousOrientationX[target] = body.previousOrientationX[source];
            body.previousOrientationY[target] = body.previousOrientationY[source];
            body.previousOrientationZ[target] = body.previousOrientationZ[source];
            body.previousOrientationW[target] = body.previousOrientationW[source];
            body.angularVelocityX[target] = body.angularVelocityX[source];
            body.angularVelocityY[target] = body.angularVelocityY[source];
            body.angularVelocityZ[target] = body.angularVelocityZ[source];
            body.inverseInertia1[target] = body.inverseInertia1[source];
            body.inverseInertia2[target] = body.inverseInertia2[source];
            body.inverseInertia3[target] = body.inverseInertia3[source];
            body.adaptationLambdaX[target] = body.adaptationLambdaX[source];
            body.adaptationLambdaY[target] = body.adaptationLambdaY[source];
            body.adaptationLambdaZ[target] = body.adaptationLambdaZ[source];
            body.restRotation1[target] = body.restRotation1[source];
            body.restRotation2[target] = body.restRotation2[source];
            body.restRotation3[target] = body.restRotation3[source];
            body.kirchhoffBendCompliance1[target] =
                body.kirchhoffBendCompliance1[source];
            body.kirchhoffBendCompliance2[target] =
                body.kirchhoffBendCompliance2[source];
            body.kirchhoffTwistCompliance[target] =
                body.kirchhoffTwistCompliance[source];
            body.bendTwistLambda1[target] = body.bendTwistLambda1[source];
            body.bendTwistLambda2[target] = body.bendTwistLambda2[source];
            body.bendTwistLambda3[target] = body.bendTwistLambda3[source];
        }
    }

    #remapXpbdStructuralState(body, points, nodeIndex, topologyDelta, count) {
        const lastSegment = Math.min(body.segmentCount - 1, count - 2);
        if (topologyDelta > 0) {
            for (let segment = lastSegment; segment >= nodeIndex + 1; segment--) {
                body.restLength[segment] = body.restLength[segment - 1];
                this.#copyXpbdMaterialFrameState(body, segment, segment - 1);
            }
            // The inserted node splits the old segment nodeIndex - 1 in two.
            // Its distal half inherits that segment's current material frame;
            // retaining the frame formerly stored at nodeIndex associates the
            // new edge with the following material interval and injects an
            // artificial hinge on every feed topology change.
            if (
                nodeIndex > 0 &&
                nodeIndex <= lastSegment
            ) {
                this.#copyXpbdMaterialFrameState(
                    body,
                    nodeIndex,
                    nodeIndex - 1
                );
            }
        } else if (topologyDelta < 0) {
            for (
                let segment = Math.max(0, nodeIndex);
                segment <= lastSegment;
                segment++
            ) {
                body.restLength[segment] = body.restLength[segment + 1];
                this.#copyXpbdMaterialFrameState(body, segment, segment + 1);
            }
        }
        const firstChangedSegment = Math.max(0, nodeIndex - 1);
        const lastChangedSegment = Math.min(lastSegment, nodeIndex);
        for (
            let segment = firstChangedSegment;
            segment <= lastChangedSegment;
            segment++
        ) {
            body.restLength[segment] = Math.max(this.minimumLayoutSegmentLength, Math.abs(
                (this._centerlineDistances[segment + 1] ?? 0) -
                (this._centerlineDistances[segment] ?? 0)
            ));
        }
        const firstChangedBend = Math.max(1, nodeIndex - 1);
        const lastChangedBend = Math.min(count - 2, nodeIndex + 1);
        for (let index = firstChangedBend; index <= lastChangedBend; index++) {
            {
                body.bendTwistLambda1[index] = 0;
                body.bendTwistLambda2[index] = 0;
                body.bendTwistLambda3[index] = 0;
                for (
                    let segment = Math.max(0, index - 1);
                    segment <= Math.min(body.segmentCount - 1, index);
                    segment++
                ) {
                    body.adaptationLambdaX[segment] = 0;
                    body.adaptationLambdaY[segment] = 0;
                    body.adaptationLambdaZ[segment] = 0;
                }
            }
        }
    }

    #initializeInsertedXpbdNode(body, points, index, count) {
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
            const extrapolatePhysicalTangent = this.guidewireInserted <= MIN_GUIDE_SUPPORT ||
                    (this._centerlineDistances[index] ?? Infinity) > this.guidewireInserted &&
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
            const restLength = Math.max(this.minimumLayoutSegmentLength, point.distanceTo(previous));
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
    }

    advance(command, dt, guidewireInserted) {
        this.motionCommand = command;
        this._feedDt = dt;
        this.previousGuidewireInserted = this.guidewireInserted;
        this.guidewireInserted = Math.max(0, guidewireInserted);
        this.guidewireDelta = this.guidewireInserted - this.previousGuidewireInserted;
        const speed = command > 0 ? CATHETER_ADVANCE_SPEED : CATHETER_WITHDRAW_SPEED;
        const nextProgress = clamp(this.progress + command * speed * dt, 0, this.maxLength);
        if (!this.vessel?.sheath && nextProgress > this.progress) {
            this.#recordGuidewirePath(Math.min(nextProgress, this.guidewireInserted));
        } else if (!this.vessel?.sheath && nextProgress < this.progress) {
            this.#trimPath(nextProgress);
        }
        // Existing catheter material keeps its recorded route. Re-sampling
        // that route from a guidewire which containment just projected back
        // into the catheter creates a closed feedback loop. New material is
        // still seeded from the guidewire by #recordGuidewirePath above.
        this.progress = nextProgress;
    }

    rotate(command, dt) {
        this.rotationCommand = command;
        if (!command) return;
        const rotationDelta = command * ROTATION_SPEED * dt;
        this.rotation += rotationDelta;
        this._pendingXpbdRotation += rotationDelta;
    }

    stepPhysics(dt = 1 / 60) {
        if (this.vessel?.sheath) return;
        const state = this.#deploymentState();
        this.#updateGuidewireRelease(dt);

        const stepIndex = this._physicsStepIndex++;
        if ((stepIndex & 3) === 0) this.#relaxSupportedPath(state.pathEnd);
        this.#syncExternalFreeNodesFromXpbdBody();
        this.#updateExternalShapeTargets(state, dt);

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
        // A standalone catheter is boundary-driven just like the guidewire:
        // XPBD owns its free material continuously, including while the
        // operator is feeding or withdrawing it. The analytical preform is
        // only a constitutive rest strain/visual target and must never replace
        // the live rod pose during manipulation.
        const xpbdOwnsUnsupportedPose =
            this.guidewireInserted <= MIN_GUIDE_SUPPORT &&
            this.physicsBody && this.physicsActiveCount >= 2;
        for (let index = 1; index < this.freeNodes.length; index++) {
            const node = this.freeNodes[index];
            node.curl = Math.min(1, (node.curl ?? 1) + PIGTAIL_RELEASE_CURL_RATE * dt);
            const relativeDistance = Math.max(0, (node.distance ?? baseDistance) - baseDistance);
            const shapeTarget = this.#freeShapeTarget(
                relativeDistance,
                frame,
                state.freeLength,
                node.curl,
                node.shapeTarget
            );
            if (!xpbdOwnsUnsupportedPose) {
                node.pos.copy(shapeTarget);
                node.vel.set(0, 0, 0);
            }
        }
    }

    #syncExternalFreeNodesFromXpbdBody() {
        const body = this.physicsBody;
        if (!body || this.physicsActiveCount < 2) return;
        for (let index = 1; index < this.freeNodes.length; index++) {
            const node = this.freeNodes[index];
            const bodyIndex = node._xpbdIndex ?? -1;
            if (bodyIndex < body.activeStart || bodyIndex > body.activeEnd) continue;
            node.pos.set(
                body.x[bodyIndex],
                body.y[bodyIndex],
                body.z[bodyIndex]
            );
            node.vel.set(
                body.velocityX[bodyIndex],
                body.velocityY[bodyIndex],
                body.velocityZ[bodyIndex]
            );
        }
    }

    updateMesh() {
        if (this.physicsBody && this.physicsActiveCount < 2) {
            this.mesh.visible = false; this.tipMarker.visible = false; return;
        }
        const body = this.physicsBody;
        const physicalPath = getCompositeJointRenderPath(body?.jointStateView);
        const points = body ? null : this.#buildCenterline();
        // The prescribed proximal reservoir is visible even though it does
        // not participate in the free-rod solve. Its positions already lie on
        // the sheath axis; rendering it must not enlarge the active physics range.
        const renderStart = body && this.vessel?.sheath
            ? Math.max(0, Math.min(body.activeStart, Math.floor(
                body.count - 1 - (this.progress + EXTERNAL_CATHETER_VISIBLE_LENGTH) / body.segmentLength)))
            : body?.activeStart ?? 0;
        const pointCount = physicalPath ? physicalPath.pointCount : body ? body.activeEnd - renderStart + 1 : this._centerlinePointCount;
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
            if (physicalPath) physicalPath.getPointAt(index / (renderPointCount - 1), renderPoint);
            else renderPoint.set(
                body ? body.x[renderStart + index] : points[index].x,
                body ? body.y[renderStart + index] : points[index].y,
                body ? body.z[renderStart + index] : points[index].z
            );
        }
        const previousGeometry = this.shaftMesh.geometry;
        const nextGeometry = updateSmoothTubeGeometry(
            previousGeometry,
            this._renderPoints,
            {
                radius: PIGTAIL_CATHETER_RENDER_RADIUS_MM,
                path: physicalPath,
                samplesPerSegment: 3,
                radialSegments: 14
            }
        );
        if (nextGeometry !== previousGeometry) {
            this.shaftMesh.geometry = nextGeometry;
            previousGeometry.dispose();
        }
        this.#updateTipMarker(renderPointCount);
        this.mesh.visible = true;
    }

    getInjectionPorts(out = []) {
        out.length = 0;
        const body = this.physicsBody;
        const points = body ? null : this.#buildCenterline();
        const pointCount = body ? this.physicsActiveCount : this._centerlinePointCount;
        if (pointCount < 2) return out;

        if (this.type !== CATHETER_TYPE_PIGTAIL) {
            const port = this._injectionPortPool[0];
            if (!this.#sampleDistalCenterline(
                0,
                port.position,
                port.direction,
                body,
                points,
                pointCount
            )) return out;
            port.kind = `${this.type}-end`;
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
        for (let index = body ? body.activeEnd : pointCount - 1; index > (body ? body.activeStart : 0); index--) {
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
        const markerDistance =
            catheterMaterialProfile(this.type).naturalArcLength;
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
        // The legacy layout used 5 mm internal and 6 mm external samples at
        // the default 4 mm physics resolution. Scale both material samplings
        // with the requested resolution so changing node spacing cannot
        // create one oversized outlet segment, while preserving the exact
        // default topology and its stable feed history.
        const resolutionScale = this.pathSpacing / DEFAULT_PATH_SPACING;
        const shaftSampleSpacing = 5 * resolutionScale;
        const externalSampleSpacing = 6 * resolutionScale;
        const shaftSamples = shaftEnd > 0
            ? clamp(Math.ceil(shaftEnd / shaftSampleSpacing), 1, 160)
            : 0;
        const points = this._centerlinePoints;

        if (externalLength > 0) {
            const externalSamples = clamp(
                Math.ceil(externalLength / externalSampleSpacing),
                2,
                96
            );
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
            if (state.pathEnd > shaftEnd + this.minimumLayoutSegmentLength) {
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
            this.freeNodes[i]._xpbdIndex = index;
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
        if (this.progress <= 0 || !this.retainMaterialTip && this.progress < 4) {
            state.pathEnd = 0;
            state.supportEnd = 0;
            state.freeLength = 0;
            return state;
        }

        const sheathSupportEnd = this.#sheathSupportEnd();
        const pathEnd = this.retainMaterialTip ? this.progress : Math.max(sheathSupportEnd, Math.min(this.progress, this.#pathEndDistance()));
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
        const shapeRotation = this.rotation;
        normal.applyAxisAngle(tangent, shapeRotation).normalize();
        return frame;
    }

    #syncFreeNodes(state, frame) {
        const distances = this.freeRestDistances;
        distances[0] = state.supportEnd;
        const freeLength = Math.max(0, this.progress - state.supportEnd);
        const freeSegmentCount = Math.max(
            1,
            Math.ceil(Math.max(0, freeLength - 0.5) / this.freeNodeSpacing)
        );
        let distanceCount = 1;
        // Anchor the sampling lattice at the physical distal tip and place the
        // fractional feed segment at the sheath outlet. Anchoring at the
        // outlet put the fractional segment at the tip, so every count change
        // deleted or created distal material and produced a visible jump.
        for (let segment = 1; segment <= freeSegmentCount; segment++) {
            distances[distanceCount++] = this.progress -
                (freeSegmentCount - segment) * this.freeNodeSpacing;
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
            if (bestIndex >= 0 && bestDelta <= this.freeNodeSpacing * 0.7) {
                node = oldNodes[bestIndex];
                oldCursor = bestIndex + 1;
            } else {
                const wasJustReleased = this.guidewireDelta < -1e-4
                    && distance >= this.guidewireInserted - GUIDE_CAPTURE_TOLERANCE
                    && distance <= this.previousGuidewireInserted + GUIDE_CAPTURE_TOLERANCE;
                const releasedByCatheterFeed = this.motionCommand > 1e-6
                    && distance > this.guidewireInserted + 0.5;
                const initialCurl = releasedByCatheterFeed
                    ? 0
                    : wasJustReleased
                        ? PIGTAIL_RELEASE_CURL_START
                        : 1;
                const restPoint = this.#freeShapeTarget(
                    relativeDistance,
                    frame,
                    state.freeLength,
                    initialCurl,
                    this._newNodeRest
                );
                const pathPoint = this.#sampleCatheterPath(
                    Math.min(distance, this.#pathEndDistance()),
                    this._newNodePath
                );
                const guideSupported = this.guidewireInserted > MIN_GUIDE_SUPPORT
                    && distance <= this.guidewireInserted + GUIDE_CAPTURE_TOLERANCE;
                const point = this._newNodePoint;
                if (wasJustReleased || releasedByCatheterFeed) {
                    point.copy(pathPoint).lerp(restPoint, initialCurl);
                }
                else if (guideSupported) {
                    point.copy(this.#sampleGuidewire(distance, this._newNodeGuide)).lerp(restPoint, 0.28);
                } else point.copy(restPoint);
                const projectedPoint = point;
                node = this.#acquireFreeNode(
                    projectedPoint,
                    distance,
                    initialCurl
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
            _xpbdIndex: -1,
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
        node._xpbdIndex = -1;
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
        {
            pointValues[0] += dx;
            pointValues[1] += dy;
            pointValues[2] += dz;
            return;
        }

    }

    #freeRestPoint(distance, frame, freeLength, curlScale = 1, out = new TypedVector3()) {
        if (this.type !== CATHETER_TYPE_PIGTAIL) {
            return this.#profileRestPoint(
                distance,
                frame,
                freeLength,
                curlScale,
                out
            );
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

    #profileRestPoint(distance, frame, freeLength, curlScale = 1, out = new TypedVector3()) {
        const profile = catheterMaterialProfile(this.type);
        const rest = profile.sampleRestCenterline(
            freeLength,
            distance,
            curlScale,
            this._profileRestSample ??= {}
        );
        const bendNormal = this.#preformBendNormal(frame, this._shapeNormal);
        return out.copy(frame.supportTip)
            .addScaledVector(frame.tangent, rest.tangentDistance)
            .addScaledVector(
                bendNormal,
                rest.normalDistance * profile.frameNormalSign
            );
    }

    #preformBendNormal(frame, normal = new TypedVector3()) {
        normal.copy(frame.normal);
        normal.z *= 0.18;
        normal.addScaledVector(frame.tangent, -normal.dot(frame.tangent));
        if (normal.lengthSq() < 1e-6) return normal.copy(frame.normal);
        return normal.normalize();
    }

    #freeShapeTarget(distance, frame, freeLength, curlScale = 1, out = new TypedVector3()) {
        const absoluteDistance = this.#sheathSupportEnd() + distance;
        if (this.guidewireInserted > MIN_GUIDE_SUPPORT) {
            if (absoluteDistance <= this.guidewireInserted) {
                if (this.pathSamples.length) {
                    return out.copy(this.#sampleCatheterPath(
                        absoluteDistance,
                        this._shapeNormal
                    ));
                }
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
        return target;
    }

    #guideReleaseFrame(fallbackFrame) {
        const frame = this._guideReleaseFrameScratch;
        const useRecordedCatheterAxis =
            Math.abs(this.guidewireDelta) <= 1e-5 &&
            this.progress > this.guidewireInserted;
        const beforeDistance = Math.max(
            this.#sheathSupportEnd(),
            this.guidewireInserted - 10
        );
        if (useRecordedCatheterAxis) {
            this.#sampleCatheterPath(this.guidewireInserted, frame.supportTip);
            this.#sampleCatheterPath(beforeDistance, frame.beforeTip);
        } else {
            this.#sampleGuidewire(this.guidewireInserted, frame.supportTip);
            this.#sampleGuidewire(beforeDistance, frame.beforeTip);
        }
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
        const releaseLength =
            catheterMaterialProfile(this.type).naturalArcLength;
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
        if (this.type !== CATHETER_TYPE_PIGTAIL) {
            const releaseScale = this._guidewireRelease * clamp(curlScale, 0, 1);
            const profile = catheterMaterialProfile(this.type);
            const rest = profile.sampleRestCenterline(
                unsupportedLength,
                distance,
                releaseScale,
                this._profileReleaseSample ??= {}
            );
            const bendNormal = this.#preformBendNormal(frame, this._shapeNormal);
            return out.copy(frame.supportTip)
                .addScaledVector(frame.tangent, rest.tangentDistance)
                .addScaledVector(
                    bendNormal,
                    rest.normalDistance * profile.frameNormalSign
                );
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

    #xpbdShapeMemoryWeight(insertedDistance) {
        if (!Number.isFinite(insertedDistance) || insertedDistance <= 0) return 0;
        const curvedTipLength =
            catheterMaterialProfile(this.type).naturalArcLength;
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
        const profile = catheterMaterialProfile(this.type);
        const softTipLength = this.type === CATHETER_TYPE_PIGTAIL
            ? XPBD_SOFT_TIP_LENGTH
            : Math.max(
                BERENSTEIN_XPBD_SOFT_TIP_LENGTH,
                profile.naturalArcLength
            );
        const transitionLength = this.type === CATHETER_TYPE_PIGTAIL
            ? XPBD_SOFT_TIP_TRANSITION_LENGTH
            : BERENSTEIN_XPBD_SOFT_TIP_TRANSITION_LENGTH;
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

    #naturalShapeLength() {
        return this.type === CATHETER_TYPE_PIGTAIL
            ? DISTAL_RELEASE_LENGTH
            : catheterMaterialProfile(this.type).naturalArcLength;
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
        const guidePoint = this.#sampleGuidewire(distance, this._newNodeGuide);
        const body = this.physicsBody;
        if (
            body &&
            this.physicsActiveCount >= 2 &&
            index > 0 &&
            distance >= this.guidewireInserted - GUIDE_CAPTURE_TOLERANCE
        ) {
            const tip = Math.min(body.activeEnd, this.physicsActiveCount - 1);
            const beforeTip = Math.max(body.activeStart, tip - 1);
            let tangentX = body.x[tip] - body.x[beforeTip];
            let tangentY = body.y[tip] - body.y[beforeTip];
            let tangentZ = body.z[tip] - body.z[beforeTip];
            const tangentLength = magnitude3(tangentX, tangentY, tangentZ);
            if (tangentLength > 1e-6) {
                tangentX /= tangentLength;
                tangentY /= tangentLength;
                tangentZ /= tangentLength;
                const previousDistance = this.pathSamples[index - 1]?.distance ?? distance;
                const extension = Math.max(0, distance - previousDistance);
                sample.point.set(
                    body.x[tip] + tangentX * extension,
                    body.y[tip] + tangentY * extension,
                    body.z[tip] + tangentZ * extension
                ).lerp(guidePoint, XPBD_GUIDEWIRE_PATH_SEED_BLEND);
            } else {
                sample.point.copy(guidePoint);
            }
        } else {
            sample.point.copy(guidePoint);
        }
        this.pathSamples[index] = sample;
        return sample;
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

    #normalizeType(type) {
        if (
            type === CATHETER_TYPE_SIM1 ||
            type === 'sim-1' ||
            type === 'simmons-1'
        ) return CATHETER_TYPE_SIM1;
        return type === CATHETER_TYPE_BERENSTEIN || type === 'bernstein'
            ? CATHETER_TYPE_BERENSTEIN
            : CATHETER_TYPE_PIGTAIL;
    }
}
