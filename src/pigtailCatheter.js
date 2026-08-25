import * as THREE from 'three';
import { clamp, smoothstep } from './mathUtils.js';
import {
    PIGTAIL_CATHETER_INNER_RADIUS_MM,
    PIGTAIL_CATHETER_RADIUS_MM,
    PIGTAIL_CATHETER_RENDER_RADIUS_MM
} from './toolDimensions.js';
import {
    BERENSTEIN_BEND_LENGTH_MM,
    BERENSTEIN_NATURAL_BEND_ANGLE_RAD,
    catheterMaterialProfile,
    PIGTAIL_NATURAL_ARC_LENGTH_MM,
    PIGTAIL_NATURAL_RADIUS_MM,
    PIGTAIL_NATURAL_TURNS
} from './physics/catheterMaterialProfile.js';
import { applyKirchhoffMaterialProfile } from './physics/applyKirchhoffMaterialProfile.js';
import { applyProximalTwistBoundary } from './physics/kirchhoffOrientationBoundary.js';
import { updateSmoothTubeGeometry } from './smoothTubeGeometry.js';

const CATHETER_RADIUS = PIGTAIL_CATHETER_RADIUS_MM;
const PIGTAIL_RADIUS = PIGTAIL_NATURAL_RADIUS_MM;
const PIGTAIL_TURNS = PIGTAIL_NATURAL_TURNS;
const PIGTAIL_ARC_LENGTH = PIGTAIL_NATURAL_ARC_LENGTH_MM;
const CATHETER_TYPE_PIGTAIL = 'pigtail';
const CATHETER_TYPE_BERENSTEIN = 'berenstein';
const CATHETER_TYPE_SIM1 = 'sim1';
const BERENSTEIN_BEND_ANGLE = BERENSTEIN_NATURAL_BEND_ANGLE_RAD;
const BERENSTEIN_BEND_LENGTH = BERENSTEIN_BEND_LENGTH_MM;
const STRAIGHT_EXIT_LENGTH = 16;
const DISTAL_RELEASE_LENGTH = STRAIGHT_EXIT_LENGTH + PIGTAIL_ARC_LENGTH;
const MIN_GUIDE_SUPPORT = 18;
const GUIDE_CAPTURE_TOLERANCE = 4;
const DEFAULT_PATH_SPACING = 4;
const DEFAULT_FREE_NODE_SPACING = 3.2;
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
const XPBD_GUIDEWIRE_PATH_SEED_BLEND = 0.18;
const PIGTAIL_RELEASE_CURL_START = 0.42;
const PIGTAIL_RELEASE_CURL_RATE = 2.4;
const PIGTAIL_XPBD_SHAPE_MAX_CORRECTION = 0.025;
// Finite angular compliance represents the catheter's bending stiffness.
// With a hard intrinsic-turn constraint and a hard wall constraint there is
// no static solution at a loaded bifurcation, so the two projections chatter.
// Compliance lets their impulses reach a true force-balanced equilibrium.
const PREFORM_XPBD_BEND_COMPLIANCE = 2e-5;
const PREFORM_XPBD_DIRECTION_MAX_CORRECTION = 0.012;
const SHAPE_RECOVERY_RATE = 2.6;
const SHAPE_RECAPTURE_RATE = 3.2;
const SOLO_XPBD_BEND_COMPLIANCE = 6e-7;
const SOLO_XPBD_SHAFT_MAX_BEND_ANGLE = 34.5;
const SOLO_BERENSTEIN_CURVATURE_VARIATION_COMPLIANCE = 5e-5;
const SOLO_BERENSTEIN_FEED_CURVATURE_VARIATION_COMPLIANCE = 1e-6;
const SOLO_BERENSTEIN_LONG_STRAIGHT_SPAN = 8;
const SOLO_BERENSTEIN_LONG_STRAIGHT_COMPLIANCE = 5e-7;
const SOLO_BERENSTEIN_FEED_LONG_STRAIGHT_COMPLIANCE = 5e-7;
const XPBD_SOFT_TIP_BEND_COMPLIANCE = 5e-6;
// The natural 7.2 mm loop turns by about 31.8 degrees per 4 mm segment.
// Keep fold protection above that rest angle so it does not intermittently
// clamp and release the very curvature that shape memory is trying to form.
// The normalized 7.2 mm preform peaks at roughly 33.3 degrees per 4 mm
// Voronoi cell. A small guard above that natural angle prevents a discrete
// hinge from folding more tightly than the manufactured loop.
const PIGTAIL_XPBD_SOFT_TIP_MAX_BEND_ANGLE = 33.7;
const BERENSTEIN_XPBD_SOFT_TIP_MAX_BEND_ANGLE = 24;
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
const XPBD_COMPOSITE_BEND_STIFFNESS = 2;
const XPBD_COMPOSITE_MAX_BEND_ANGLE = 34.5;
const XPBD_COMPOSITE_TAPER_LENGTH = 6;
const PIGTAIL_XPBD_FEED_POST_STABILIZATION_PASSES = 4;
const PIGTAIL_XPBD_SOLO_FEED_POST_STABILIZATION_PASSES = 4;
const PIGTAIL_XPBD_WITHDRAW_POST_STABILIZATION_PASSES = 4;
const PIGTAIL_XPBD_IDLE_SHAPE_STABILIZATION_PASSES = 4;
const GUIDEWIRE_IN_CATHETER_BLEND = 0.78;
const GUIDEWIRE_REACTION_BLEND = 0.16;
const GUIDEWIRE_CATHETER_MAX_CORRECTION = 1.2;
export const CATHETER_PROXIMAL_LOADING_SUPPORT_LENGTH_MM = 90;
const EXTERNAL_CATHETER_VISIBLE_LENGTH =
    CATHETER_PROXIMAL_LOADING_SUPPORT_LENGTH_MM;
const CATHETER_ADVANCE_SPEED = 52;
const CATHETER_WITHDRAW_SPEED = 32;
const ROTATION_SPEED = Math.PI * 0.9;
const CONTACT_CLEARANCE = CATHETER_RADIUS * 0.72;
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
        physicsSpacing = DEFAULT_PATH_SPACING
    }) {
        this.wire = wire;
        this.segmentLength = segmentLength;
        this.guidewireLength = guidewireLength;
        this.tailProgressRef = tailProgressRef;
        this.vessel = vessel;
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
        this._xpbdDistalTwisted = false;
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
        this._xpbdProximalFeedControlIndex = -1;
        this._xpbdProximalFeedTarget = new TypedVector3();
        this._xpbdPigtailBaseControlIndex = -1;
        this._xpbdDistalAnchor = new TypedVector3();
        this._xpbdDistalTangent = new TypedVector3();
        this._xpbdDistalNormal = new TypedVector3();
        this._xpbdDistalBinormal = new TypedVector3();
        this._xpbdDistalDirection = new TypedVector3();
        this._xpbdDistalReleaseDirection = new TypedVector3();
        this._xpbdDistalInterpolationAxis = new TypedVector3();
        this._xpbdDistalReferenceNormal = new TypedVector3();
        this._xpbdPigtailReferenceAnchor = new TypedVector3();
        this._xpbdPigtailReferenceTangent = new TypedVector3();
        this._xpbdPigtailReferenceNormal = new TypedVector3();
        this._xpbdPigtailReferenceActive = false;
        this._xpbdPigtailReferenceBaseIndex = -1;
        this._xpbdPigtailReferenceProgress = 0;
        this._xpbdPigtailReferenceGuidewire = 0;
        this._xpbdPigtailReferenceRotation = 0;
        this._xpbdPigtailTargetRotation = 0;
        this._xpbdPigtailReleaseX = null;
        this._xpbdPigtailReleaseY = null;
        this._xpbdPigtailReleaseZ = null;
        this._xpbdPigtailAxisX = null;
        this._xpbdPigtailAxisY = null;
        this._xpbdPigtailAxisZ = null;
        this._xpbdPigtailTransportTangent = new TypedVector3();
        this._xpbdPigtailTransportAxis = new TypedVector3();
        this._xpbdPigtailRecovery = 0;
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
        this._xpbdDistalTwisted = false;
        this.physicsLumenStartNode = 0;
        this._xpbdProgress = this.progress;
        this._xpbdYieldsToWall = false;
        this._xpbdDistalReferenceNormal.set(0, 0, 0);
        this.#clearXpbdPigtailReference();
        this.updateMesh();
    }

    dispose() {
        this.#releaseXpbdProximalFeed();
        this.shaftMesh.geometry?.dispose?.();
        this.tipMarker.geometry?.dispose?.();
        this.material.dispose();
        this.tipMarkerMaterial.dispose();
    }

    setExternalCollisionSolver(enabled = true) {
        if (!enabled) {
            this.#releaseXpbdProximalFeed();
        }
        this.externalCollisionSolver = !!enabled;
        return this;
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
        this._xpbdDistalTwisted = false;
        this.pathSamples.length = 0;
        this.#clearFreeNodes();
        this.freeRestDistanceCount = 0;
        this.freeLength = 0;
        this._physicsStepIndex = 0;
        this.physicsActiveCount = 0;
        this._xpbdLayoutCount = 0;
        this._xpbdProgress = 0;
        this._xpbdYieldsToWall = false;
        this._xpbdDistalReferenceNormal.set(0, 0, 0);
        this.#clearXpbdPigtailReference();
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
        const initializeKirchhoffFrames = body.rodModel === 'kirchhoff' &&
            (this.physicsBody !== body || this.physicsActiveCount < 2);
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
            this._xpbdPigtailReleaseX = new Float32Array(body.count);
            this._xpbdPigtailReleaseY = new Float32Array(body.count);
            this._xpbdPigtailReleaseZ = new Float32Array(body.count);
            this._xpbdPigtailAxisX = new Float32Array(body.segmentCount);
            this._xpbdPigtailAxisY = new Float32Array(body.segmentCount);
            this._xpbdPigtailAxisZ = new Float32Array(body.segmentCount);
            this._xpbdLayoutCount = 0;
            this.physicsActiveCount = 0;
            this._xpbdProgress = this.progress;
            progressDelta = 0;
            this._xpbdYieldsToWall = false;
            this.#clearXpbdPigtailReference();
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
            body.intrinsicClosureCorrectionScale =
                idle ? 0.08 : 0;
        }
        body.restDirectionSubiterations = 1;
        // Pigtail shape memory is represented by material curvature below.
        // Re-solving an additional world-space positional shape in the idle
        // passes creates a second elastic potential with a moving reference.
        body.postStabilizeShape = false;
        body.restDirectionContactPasses = 0;
        body.restDirectionContactCorrectionScale = 0.08;
        body.restTurnPolishMaxAngle =
            Math.abs(this.motionCommand) <= 1e-6 &&
            Math.abs(this.guidewireDelta) <= 1e-5
                ? 0.5 * Math.PI / 180
                : 0;
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
            for (let index = 0; index < this.physicsActiveCount; index++) body.clearRestShapeTarget(index);
            body.setActiveRange(0, 1);
            body.setCollisionRange(0, -1);
            this.physicsActiveCount = 0;
            this._xpbdLayoutCount = 0;
            this._xpbdProgress = this.progress;
            this._xpbdYieldsToWall = false;
            this._pendingXpbdRotation = 0;
            this.#clearXpbdPigtailReference();
            body.curvatureVariationEnabled = false;
            body.longStraightSpan = 0;
            if (body.rodModel === 'kirchhoff') {
                body.clearProximalOrientationControl?.();
            }
            this.#releaseXpbdProximalFeed(body);
            return 0;
        }

        const previousCount = this.physicsActiveCount;
        const soloXpbd = this.externalCollisionSolver &&
            this.guidewireInserted <= MIN_GUIDE_SUPPORT;
        const unsupportedShapeLength = this.#naturalShapeLength();
        const hasLocallyUnsupportedShaft =
            this.externalCollisionSolver &&
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
            this.#initializeInsertedXpbdNode(body, points, insertedIndex, count, shapeCompliance);
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
            if (
                this.type === CATHETER_TYPE_PIGTAIL &&
                this._xpbdPigtailReferenceActive &&
                topologyIndex <= this._xpbdPigtailReferenceBaseIndex
            ) {
                this._xpbdPigtailReferenceBaseIndex = clamp(
                    this._xpbdPigtailReferenceBaseIndex + topologyDelta,
                    1,
                    count - 2
                );
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
                    body.restLength[index] = Math.max(0.5, materialLength);
                }
                for (let index = 1; index < count - 1; index++) {
                    this.#initializeXpbdBendChord(body, points, index);
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
        body.curvatureVariationEnabled = false;
        if (!Number.isFinite(body.shaftFeedStiffnessBlend)) {
            body.shaftFeedStiffnessBlend = activelyFeeding ? 1 : 0;
        }
        body.shaftFeedStiffnessBlend = activelyFeeding ? 1 : 0;
        body.curvatureVariationCompliance =
            SOLO_BERENSTEIN_CURVATURE_VARIATION_COMPLIANCE +
            (
                SOLO_BERENSTEIN_FEED_CURVATURE_VARIATION_COMPLIANCE -
                SOLO_BERENSTEIN_CURVATURE_VARIATION_COMPLIANCE
            ) * body.shaftFeedStiffnessBlend;
        const movingShaftStart = Math.max(
            0,
            collisionStart + SOLO_BERENSTEIN_LONG_STRAIGHT_SPAN
        );
        const settledShaftStart = Math.max(0, collisionStart + 1);
        if (activelyFeeding) {
            body.curvatureVariationStartNode = movingShaftStart;
        } else {
            body.curvatureVariationStartNode = Math.max(
                settledShaftStart,
                Math.min(
                    movingShaftStart,
                    body.curvatureVariationStartNode || movingShaftStart
                ) - 1
            );
        }
        body.curvatureVariationEndNode = Math.max(
            body.curvatureVariationStartNode,
            count - 1 - Math.ceil(
                (
                    catheterMaterialProfile(this.type).naturalArcLength
                ) / body.segmentLength
            )
        );
        body.longStraightSpan = 0;
        body.longStraightCompliance = SOLO_BERENSTEIN_LONG_STRAIGHT_COMPLIANCE +
            (
                SOLO_BERENSTEIN_FEED_LONG_STRAIGHT_COMPLIANCE -
                SOLO_BERENSTEIN_LONG_STRAIGHT_COMPLIANCE
            ) * body.shaftFeedStiffnessBlend;
        body.longStraightStartNode = body.curvatureVariationStartNode;
        body.longStraightEndNode = body.curvatureVariationEndNode;
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
        const enteredWallYield = bodyTouchesWall && !this._xpbdYieldsToWall;
        if (bodyTouchesWall) this._xpbdYieldsToWall = true;
        const releasedPigtail =
            this.type === CATHETER_TYPE_PIGTAIL &&
            this.progress > this.guidewireInserted + 0.5;
        const pigtailIdle =
            Math.abs(this.motionCommand) <= 1e-6 &&
            Math.abs(this.guidewireDelta) <= 1e-5;
        const localPigtailShapeOwner =
            this.type === CATHETER_TYPE_PIGTAIL &&
            (soloXpbd || releasedPigtail) &&
            (
                pigtailIdle ||
                (
                    releasedPigtail &&
                    this.motionCommand > 1e-6 &&
                    Math.abs(this.guidewireDelta) <= 1e-5
                )
            );
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
            if (this.externalCollisionSolver && shapeWeight > XPBD_MIN_SHAPE_WEIGHT) {
                for (let freeIndex = 1; freeIndex < this.freeNodes.length; freeIndex++) {
                    const freeNode = this.freeNodes[freeIndex];
                    if (freeNode._xpbdIndex !== index) continue;
                    idealShapePoint = freeNode.shapeTarget;
                    break;
                }
            }
            const guideReleaseStability = this.guidewireInserted > MIN_GUIDE_SUPPORT
                ? 1 - smoothstep(
                    this.guidewireInserted + GUIDE_CAPTURE_TOLERANCE,
                    this.guidewireInserted + XPBD_RELEASE_STABILITY_LENGTH,
                    insertedDistance
                )
                : 0;
            const localPigtailWallContact =
                this.type === CATHETER_TYPE_PIGTAIL &&
                localPigtailContactOwner &&
                insertedDistance >= localPigtailShapeStart &&
                (
                    (index > 0 && body.wallActive[index - 1]) ||
                    (index < body.segmentCount && body.wallActive[index])
                );
            const yieldsToWall =
                this.type === CATHETER_TYPE_PIGTAIL
                    ? localPigtailContactOwner &&
                        insertedDistance >= localPigtailShapeStart &&
                        localPigtailWallContact
                    : this._xpbdYieldsToWall;
            const stabilizeFixedPath = this.externalCollisionSolver &&
                index <= collisionStart;
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
            const locallyControlledPigtailNode =
                localPigtailShapeOwner &&
                insertedDistance >= localPigtailShapeStart;
            if (locallyControlledPigtailNode) {
                // #stabilizeSoloDistalShape owns these targets. Rewriting or
                // clearing them here would reset shapeLambda twice per step
                // and turn steady elastic recovery into a visible flutter.
            } else if (
                stabilizeFixedPath ||
                (hasShapeMemory && !soloXpbd)
            ) {
                const targetWasEnabled = body.restShapeEnabled[index] === 1;
                const holdBerensteinTwist =
                    this.type !== CATHETER_TYPE_PIGTAIL &&
                    this._xpbdDistalTwisted &&
                    hasShapeMemory &&
                    !stabilizeFixedPath &&
                    targetWasEnabled;
                let targetX = (
                    holdBerensteinTwist ||
                    (yieldsToWall && !stabilizeFixedPath && targetWasEnabled)
                )
                    ? body.restShapeX[index]
                    : idealShapePoint.x;
                let targetY = (
                    holdBerensteinTwist ||
                    (yieldsToWall && !stabilizeFixedPath && targetWasEnabled)
                )
                    ? body.restShapeY[index]
                    : idealShapePoint.y;
                let targetZ = (
                    holdBerensteinTwist ||
                    (yieldsToWall && !stabilizeFixedPath && targetWasEnabled)
                )
                    ? body.restShapeZ[index]
                    : idealShapePoint.z;
                if (
                    stabilizeFixedPath &&
                    index > collisionStart &&
                    this.sheathPath
                ) {
                    const materialDistance = this._centerlineDistances[index] ??
                        this.sheathPath.length;
                    const overhang = Math.max(
                        0,
                        materialDistance - this.sheathPath.length
                    );
                    targetX = this.sheathPath.end.x +
                        this.sheathPath.dir.x * overhang;
                    targetY = this.sheathPath.end.y +
                        this.sheathPath.dir.y * overhang;
                    targetZ = this.sheathPath.end.z +
                        this.sheathPath.dir.z * overhang;
                }
                if (
                    targetWasEnabled &&
                    !topologyChanged &&
                    this._xpbdLayoutCount === count &&
                    progressDelta < -1e-6 &&
                    (
                        holdBerensteinTwist ||
                        (yieldsToWall && !stabilizeFixedPath)
                    )
                ) {
                    // Preserve the wall-conformed offset, not the old absolute
                    // world position. A frozen world-space target makes a
                    // withdrawing catheter spring back toward the location at
                    // which it first touched the wall.
                    let transportX = point.x - this._xpbdLayoutX[index];
                    let transportY = point.y - this._xpbdLayoutY[index];
                    let transportZ = point.z - this._xpbdLayoutZ[index];
                    const transportLength = magnitude3(
                        transportX,
                        transportY,
                        transportZ
                    );
                    const transportLimit = Math.max(
                        0.05,
                        Math.min(0.35, targetSlewLimit)
                    );
                    if (transportLength > transportLimit) {
                        const scale = transportLimit / transportLength;
                        transportX *= scale;
                        transportY *= scale;
                        transportZ *= scale;
                    }
                    targetX += transportX;
                    targetY += transportY;
                    targetZ += transportZ;
                }
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
                                        this.type !== CATHETER_TYPE_PIGTAIL &&
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
                    const dx = idealShapePoint.x - body.restShapeX[index];
                    const dy = idealShapePoint.y - body.restShapeY[index];
                    const dz = idealShapePoint.z - body.restShapeZ[index];
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
                if (yieldsToWall && !stabilizeFixedPath) {
                    const dx = targetX - body.x[index];
                    const dy = targetY - body.y[index];
                    const dz = targetZ - body.z[index];
                    const distance = magnitude3(dx, dy, dz);
                    const maximumTargetOffset =
                        this.type === CATHETER_TYPE_PIGTAIL
                            ? PIGTAIL_XPBD_TARGET_MAX_OFFSET
                            : BERENSTEIN_XPBD_TARGET_MAX_OFFSET;
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
                    effectiveShapeCompliance,
                    this.type === CATHETER_TYPE_PIGTAIL && hasShapeMemory
                        ? PIGTAIL_XPBD_SHAPE_MAX_CORRECTION
                        : Infinity
                );
            } else {
                body.clearRestShapeTarget(index);
            }
            body.nodeRadius[index] = CATHETER_RADIUS;
            const globalUnsupportedStiffness = 1 - smoothstep(
                0,
                MIN_GUIDE_SUPPORT,
                this.guidewireInserted
            );
            const localUnsupportedStiffness =
                this.externalCollisionSolver
                ? smoothstep(
                    this.guidewireInserted + GUIDE_CAPTURE_TOLERANCE,
                    this.guidewireInserted + XPBD_RELEASE_STABILITY_LENGTH,
                    insertedDistance
                )
                : 0;
            const unsupportedStiffness = Math.max(
                globalUnsupportedStiffness,
                localUnsupportedStiffness
            );
            const shaftBendCompliance = body.bendCompliance +
                (
                    Math.min(body.bendCompliance, SOLO_XPBD_BEND_COMPLIANCE) -
                    body.bendCompliance
                ) * unsupportedStiffness;
            const naturalBendCompliance = shaftBendCompliance +
                (XPBD_SOFT_TIP_BEND_COMPLIANCE - shaftBendCompliance) *
                softTipWeight;
            // A Kirchhoff catheter and guidewire remain two material rods.
            // Their combined curvature emerges from their two EI fields and
            // symmetric lumen contact; the legacy single-rod approximation
            // must not pre-stiffen the catheter a second time.
            const compositeSupport = body.rodModel !== 'kirchhoff' &&
                this.guidewireInserted > MIN_GUIDE_SUPPORT
                ? 1 - smoothstep(
                    this.guidewireInserted - XPBD_COMPOSITE_TAPER_LENGTH,
                    this.guidewireInserted,
                    insertedDistance
                )
                : 0;
            const compositeBendCompliance =
                body.bendCompliance / XPBD_COMPOSITE_BEND_STIFFNESS;
            body.bendComplianceByNode[index] = naturalBendCompliance +
                (compositeBendCompliance - naturalBendCompliance) *
                compositeSupport;
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
            body.maxBendAngleByNode[index] = naturalMaxBendAngle +
                (
                    Math.min(
                        body.maxBendAngle,
                        XPBD_COMPOSITE_MAX_BEND_ANGLE
                    ) - naturalMaxBendAngle
                ) * compositeSupport;
            if (index > 0) {
                const desiredLength = Math.max(0.5, Math.abs(
                    (this._centerlineDistances[index] ?? 0) -
                    (this._centerlineDistances[index - 1] ?? 0)
                ));
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
                const structuralStraightChord =
                    body.restLength[index - 1] + body.restLength[index];
                // Material state never comes from the live, wall-deformed
                // centerline. The common shaft has kappa_0 = 0; distal
                // preforms are governed exclusively by their signed intrinsic
                // curvature targets and skip this legacy chord energy.
                body.restBendChord[index] = structuralStraightChord;
            }
        }
        for (let index = count; index < previousCount; index++) body.clearRestShapeTarget(index);
        if (initializeKirchhoffFrames) {
            // The body was constructed before its live centerline existed.
            // Initialize only the current frames after the first sync; the
            // manufactured rest rotation still comes exclusively from the
            // material profile below.
            body.captureKirchhoffRestConfiguration({ captureRestRotation: false });
        }
        this.#stabilizeSoloDistalShape(body, count, soloXpbd);
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
        } else if (body.rodModel === 'kirchhoff') {
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
        if (body.rodModel === 'kirchhoff') {
            // Kirchhoff rotation is a torsional boundary condition applied in
            // sync. Rotating x/previous/velocity would inject rigid motion and
            // rotate the constitutive kappa_0 axis in world space.
            this._pendingXpbdRotation = 0;
            return;
        }
        const angle = this._pendingXpbdRotation;
        this._pendingXpbdRotation = 0;
        if (
            Math.abs(angle) < 1e-6 ||
            count < 4
        ) {
            return;
        }

        const distalShapeLength =
            catheterMaterialProfile(this.type).naturalArcLength;
        const shapeStart = Math.max(
            this.#sheathSupportEnd(),
            this.guidewireInserted,
            this.progress - distalShapeLength
        );
        let baseIndex = Math.max(1, count - 1);
        for (let index = 1; index < count; index++) {
            if ((this._centerlineDistances[index] ?? -Infinity) < shapeStart) continue;
            baseIndex = Math.max(1, index - 1);
            break;
        }
        if (baseIndex >= count - 1) return;

        // Rotate around the same shaft-only material tangent used to define
        // the intrinsic-curvature frame. Including the first curved preform
        // node here rotates geometry about a different axis than its rest
        // director and leaves a persistent elastic mismatch after rotation.
        const tangentEnd = Math.max(body.activeStart + 1, baseIndex - 1);
        const tangentStart = Math.max(body.activeStart, tangentEnd - 2);
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
        // exists. This keeps both preformed tips in their material plane.
        this._xpbdDistalTwisted = true;
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

            if (
                this._xpbdPigtailReferenceActive &&
                this._xpbdPigtailReleaseX
            ) {
                const releaseX = this._xpbdPigtailReleaseX[index];
                const releaseY = this._xpbdPigtailReleaseY[index];
                const releaseZ = this._xpbdPigtailReleaseZ[index];
                const releaseDot =
                    releaseX * axisX + releaseY * axisY + releaseZ * axisZ;
                this._xpbdPigtailReleaseX[index] =
                    releaseX * cosine +
                    (axisY * releaseZ - axisZ * releaseY) * sine +
                    axisX * releaseDot * oneMinusCosine;
                this._xpbdPigtailReleaseY[index] =
                    releaseY * cosine +
                    (axisZ * releaseX - axisX * releaseZ) * sine +
                    axisY * releaseDot * oneMinusCosine;
                this._xpbdPigtailReleaseZ[index] =
                    releaseZ * cosine +
                    (axisX * releaseY - axisY * releaseX) * sine +
                    axisZ * releaseDot * oneMinusCosine;
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

    #releaseXpbdProximalFeed(body = this.physicsBody) {
        if (body && this._xpbdProximalFeedControlIndex >= 0) {
            body.clearControlTarget(this._xpbdProximalFeedControlIndex);
            if (body.rodModel === 'kirchhoff') {
                body.setPinned(this._xpbdProximalFeedControlIndex, false);
            }
        }
        this._xpbdProximalFeedControlIndex = -1;
    }

    #applyStandaloneKirchhoffRuntime(body) {
        if (body.rodModel !== 'kirchhoff') return;

        // A catheter without guidewire support is the same kind of free,
        // boundary-driven Kirchhoff rod as the guidewire. Keep its material
        // profile (diameter, mass, stiffness and intrinsic distal curvature),
        // but do not switch to a catheter-only solver schedule while feeding
        // or after the operator releases the control.
        body.postStabilizationPasses = 0;
        body.finalStructuralClosurePasses = 8;
        body.intrinsicClosureCorrectionScale = 0;
        body.postStabilizeBending = false;
        body.restTurnPolishMaxAngle = 0;
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
        // Legacy catheter bodies still convect their unsupported material.
        // The Eulerian inlet boundary is required by the Kirchhoff path,
        // where material is fed by changing the active rest-length field.
        if (body.rodModel !== 'kirchhoff') {
            this.#releaseXpbdProximalFeed(body);
            return;
        }
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

    #clearXpbdPigtailReference() {
        this._xpbdPigtailReferenceActive = false;
        this._xpbdPigtailReferenceBaseIndex = -1;
        this._xpbdPigtailTargetRotation = this.rotation;
        this._xpbdPigtailRecovery = 0;
    }

    #clearXpbdPigtailDirectionTargets(body, count = body?.count ?? 0) {
        if (!body?.clearRestDirectionTarget) return;
        for (let segment = 0; segment < Math.min(body.segmentCount, count - 1); segment++) {
            body.clearRestDirectionTarget(segment);
        }
        body.clearShapeClosureTarget?.();
        if (this._xpbdPigtailBaseControlIndex >= 0) {
            body.clearControlTarget(this._xpbdPigtailBaseControlIndex);
            this._xpbdPigtailBaseControlIndex = -1;
        }
    }

    #advanceXpbdPigtailRecovery(dt) {
        const releasedPigtail =
            this.progress > this.guidewireInserted + 0.5;
        if (
            this.type !== CATHETER_TYPE_PIGTAIL ||
            !this.externalCollisionSolver ||
            !this._xpbdPigtailReferenceActive
        ) return;
        const locallyReleased =
            this.guidewireInserted <= MIN_GUIDE_SUPPORT ||
            this.progress > this.guidewireInserted + 0.5;
        if (!locallyReleased) return;
        // Rest curvature is a constitutive material property, not an animated
        // rest pose. Its finite compliance controls how quickly the physical
        // rod can move toward equilibrium under guidewire and wall loads.
        this._xpbdPigtailRecovery = 1;
    }

    #stabilizeSoloDistalShape(body, count, soloXpbd) {
        body.restShapeTranslationNeutralStart = -1;
        body.restShapeTranslationNeutralEnd = -1;
        if (body.rodModel === 'kirchhoff') {
            this.#applyKirchhoffMaterialShape(body, count);
            return;
        }
        const releasedPreform =
            this.progress > this.guidewireInserted + 0.5;
        const reshapeReleasedPreformDuringMotion =
            releasedPreform &&
            (
                Math.abs(this.motionCommand) > 1e-6 ||
                Math.abs(this.guidewireDelta) > 1e-5
            );
        if (
            count < 4
        ) {
            this.#clearXpbdPigtailDirectionTargets(body, count);
            this.#clearXpbdPigtailReference();
            return;
        }
        const materialProfile = catheterMaterialProfile(this.type);
        const naturalArcLength = materialProfile.naturalArcLength;
        const bendStartDistance = Math.max(
            0,
            this.progress - naturalArcLength
        );
        let baseIndex = count - 2;
        for (let index = 1; index < count - 1; index++) {
            if ((this._centerlineDistances[index] ?? -Infinity) < bendStartDistance) continue;
            baseIndex = Math.max(1, index - 1);
            break;
        }
        const baseDistance = this._centerlineDistances[baseIndex] ?? bendStartDistance;
        const availableLength = Math.max(
            0,
            Math.min(naturalArcLength, this.progress - baseDistance)
        );
        if (availableLength < 0.5 || baseIndex >= count - 1) return;

        // Build the material frame from shaft nodes immediately proximal to
        // the preform. Including the first loop node lets the forming Pigtail
        // rotate its own reference frame and feeds numerical torsion back into
        // the next step.
        const tangentEnd = Math.max(body.activeStart + 1, baseIndex - 1);
        const tangentStart = Math.max(body.activeStart, tangentEnd - 2);
        const anchor = this._xpbdDistalAnchor.set(
            body.x[baseIndex],
            body.y[baseIndex],
            body.z[baseIndex]
        );
        const tangent = this._xpbdDistalTangent.set(
            body.x[tangentEnd] - body.x[tangentStart],
            body.y[tangentEnd] - body.y[tangentStart],
            body.z[tangentEnd] - body.z[tangentStart]
        );
        if (tangent.lengthSq() < 1e-6) return;
        tangent.normalize();

        const catheterFrame = this.#freeFrame(this.#sheathSupportEnd());
        const normal = this._xpbdDistalNormal.copy(catheterFrame.normal);
        if (
            this._xpbdDistalReferenceNormal.lengthSq() > 1e-6 &&
            normal.dot(this._xpbdDistalReferenceNormal) < 0
        ) {
            normal.multiplyScalar(-1);
        }
        this._xpbdDistalReferenceNormal.copy(normal);
        if (this._xpbdDistalTwisted) {
            normal.applyAxisAngle(tangent, this.rotation);
        }
        normal.addScaledVector(tangent, -normal.dot(tangent));
        if (normal.lengthSq() < 1e-6) {
            normal.set(0, 0, 1).addScaledVector(tangent, -tangent.z);
        }
        if (normal.lengthSq() < 1e-6) {
            normal.set(0, 1, 0).addScaledVector(tangent, -tangent.y);
        }
        normal.normalize();

        {
            // Capture one material frame for every preformed catheter.
            // Subsequent feed/withdraw topology changes remap this stored
            // state by material identity; neither Pigtail nor Berenstein may
            // redefine its rest plane from the deformed current geometry.
            const referenceChanged = !this._xpbdPigtailReferenceActive;
            if (referenceChanged) {
                this._xpbdPigtailReferenceAnchor.copy(anchor);
                this._xpbdPigtailReferenceTangent.copy(tangent);
                this._xpbdPigtailReferenceNormal.copy(normal);
                if (
                    !this._xpbdPigtailReleaseX ||
                    this._xpbdPigtailReleaseX.length !== body.count
                ) {
                    this._xpbdPigtailReleaseX = new Float32Array(body.count);
                    this._xpbdPigtailReleaseY = new Float32Array(body.count);
                    this._xpbdPigtailReleaseZ = new Float32Array(body.count);
                }
                for (let index = baseIndex; index < count; index++) {
                    const previous = Math.max(baseIndex, index - 1);
                    let releaseX = body.x[index] - body.x[previous];
                    let releaseY = body.y[index] - body.y[previous];
                    let releaseZ = body.z[index] - body.z[previous];
                    const releaseLength = magnitude3(releaseX, releaseY, releaseZ);
                    if (releaseLength > 1e-6) {
                        releaseX /= releaseLength;
                        releaseY /= releaseLength;
                        releaseZ /= releaseLength;
                    } else {
                        releaseX = tangent.x;
                        releaseY = tangent.y;
                        releaseZ = tangent.z;
                    }
                    this._xpbdPigtailReleaseX[index] = releaseX;
                    this._xpbdPigtailReleaseY[index] = releaseY;
                    this._xpbdPigtailReleaseZ[index] = releaseZ;
                }
                this._xpbdPigtailRecovery = 1;
                this._xpbdPigtailReferenceActive = true;
                this._xpbdPigtailReferenceBaseIndex = baseIndex;
                this._xpbdPigtailReferenceProgress = this.progress;
                this._xpbdPigtailReferenceGuidewire = this.guidewireInserted;
                this._xpbdPigtailReferenceRotation = this.rotation;
                this._xpbdPigtailTargetRotation = this.rotation;
            } else if (baseIndex < this._xpbdPigtailReferenceBaseIndex) {
                for (
                    let index = baseIndex;
                    index < this._xpbdPigtailReferenceBaseIndex;
                    index++
                ) {
                    const previous = Math.max(baseIndex, index - 1);
                    let releaseX = body.x[index] - body.x[previous];
                    let releaseY = body.y[index] - body.y[previous];
                    let releaseZ = body.z[index] - body.z[previous];
                    const releaseLength = magnitude3(
                        releaseX,
                        releaseY,
                        releaseZ
                    );
                    if (releaseLength > 1e-6) {
                        releaseX /= releaseLength;
                        releaseY /= releaseLength;
                        releaseZ /= releaseLength;
                    } else {
                        releaseX = tangent.x;
                        releaseY = tangent.y;
                        releaseZ = tangent.z;
                    }
                    this._xpbdPigtailReleaseX[index] = releaseX;
                    this._xpbdPigtailReleaseY[index] = releaseY;
                    this._xpbdPigtailReleaseZ[index] = releaseZ;
                }
            }
            this._xpbdPigtailReferenceBaseIndex = baseIndex;
            // Parallel-transport the material plane by the shortest rotation
            // from the previous shaft tangent to the current one. Repeatedly
            // projecting the original release normal creates artificial twist
            // as the loop starts loading its own base.
            const transportAxis = this._xpbdDistalInterpolationAxis
                .crossVectors(this._xpbdPigtailReferenceTangent, tangent);
            const transportSine = transportAxis.length();
            const transportCosine = clamp(
                this._xpbdPigtailReferenceTangent.dot(tangent),
                -1,
                1
            );
            normal.copy(this._xpbdPigtailReferenceNormal);
            if (transportSine > 1e-6) {
                transportAxis.multiplyScalar(1 / transportSine);
                normal.applyAxisAngle(
                    transportAxis,
                    Math.atan2(transportSine, transportCosine)
                );
            }
            normal.addScaledVector(tangent, -normal.dot(tangent));
            if (normal.lengthSq() < 1e-6) {
                normal.set(0, 0, 1).addScaledVector(tangent, -tangent.z);
            }
            normal.normalize();
            this._xpbdPigtailReferenceTangent.copy(tangent);
            this._xpbdPigtailReferenceNormal.copy(normal);
            normal.applyAxisAngle(
                tangent,
                this.rotation - this._xpbdPigtailReferenceRotation
            );
            if (
                Math.abs(
                    this.rotation - this._xpbdPigtailTargetRotation
                ) > 1e-6
            ) {
                // #applyPendingXpbdRotation already rotates the physical loop.
                // Restart target slew from that rotated state so stale target
                // coordinates cannot pull the loop back to its old plane.
                this._xpbdPigtailTargetRotation = this.rotation;
            }
        }

        const normalSign = materialProfile.frameNormalSign;
        // A Pigtail is preformed by distributed intrinsic curvature, not by a
        // tether between its base and tip.  A closure distance constraint can
        // pull those remote points through the vessel and produces a large
        // tip kick during feed or withdrawal. Let the local signed-turn
        // constraints curl the loop while wall contact opens it naturally at
        // the bifurcation.
        body.clearShapeClosureTarget?.();
        const pigtailBinormal = this._xpbdDistalBinormal
            .crossVectors(tangent, normal)
            .multiplyScalar(normalSign)
            .normalize();
        {
            if (
                !this._xpbdPigtailAxisX ||
                this._xpbdPigtailAxisX.length !== body.segmentCount
            ) {
                this._xpbdPigtailAxisX = new Float32Array(body.segmentCount);
                this._xpbdPigtailAxisY = new Float32Array(body.segmentCount);
                this._xpbdPigtailAxisZ = new Float32Array(body.segmentCount);
            }

            // Both supplied preforms are planar manufactured shapes. Their
            // intrinsic curvature vectors therefore share one material
            // binormal transported from the shaft base. Rebuilding a separate
            // Bishop axis from the already deformed centerline made the rest
            // plane follow contact-induced torsion and turned a closed loop
            // into a three-dimensional helix.
            for (let segment = baseIndex; segment < count - 1; segment++) {
                this._xpbdPigtailAxisX[segment] = pigtailBinormal.x;
                this._xpbdPigtailAxisY[segment] = pigtailBinormal.y;
                this._xpbdPigtailAxisZ[segment] = pigtailBinormal.z;
            }
            if (this._xpbdPigtailBaseControlIndex >= 0) {
                body.clearControlTarget(this._xpbdPigtailBaseControlIndex);
                this._xpbdPigtailBaseControlIndex = -1;
            }
            for (let segment = 0; segment < Math.min(body.segmentCount, count - 1); segment++) {
                if (segment >= baseIndex) continue;
                body.clearRestDirectionTarget(segment);
            }
        }
        for (let index = baseIndex + 1; index < count; index++) {
            const previous = index - 1;
            const outgoingRestLength = Math.max(
                0.5,
                body.restLength[Math.min(body.segmentCount - 1, previous)]
            );
            const incomingRestLength = previous > 0
                ? Math.max(0.5, body.restLength[previous - 1])
                : outgoingRestLength;
            const voronoiLength =
                (incomingRestLength + outgoingRestLength) * 0.5;
            const materialDistance =
                this._centerlineDistances[previous] ?? this.progress;
            const distanceFromTip = Math.max(
                0,
                this.progress - materialDistance
            );
            const catheterNaturalTurn = materialProfile.integrateIntrinsicTurn(
                distanceFromTip,
                voronoiLength
            );
            const insertedDistance = materialDistance;
            const compositeSupport = this.guidewireInserted > MIN_GUIDE_SUPPORT
                ? 1 - smoothstep(
                    this.guidewireInserted - XPBD_COMPOSITE_TAPER_LENGTH,
                    this.guidewireInserted,
                    insertedDistance
                )
                : 0;
            // Coaxial catheter and guidewire are one composite beam. Their
            // equilibrium curvature is stiffness weighted; the straight,
            // much stiffer wire body therefore suppresses catheter kappa_0
            // while it occupies the lumen. The material target itself remains
            // continuous and recovers through the distal support taper.
            const effectiveNaturalTurn = catheterNaturalTurn *
                (1 - compositeSupport);
            // Intrinsic curvature and the straight shaft are two rest states
            // of the same rod, so they must use the same material stiffness
            // profile.  The distal construction may be more compliant, but
            // zero-curvature portions (notably the straight Berenstein tip)
            // no longer fall onto a separate, much softer solver constant.
            const intrinsicBendCompliance = Math.min(
                materialProfile.intrinsicBendCompliance ??
                    PREFORM_XPBD_BEND_COMPLIANCE,
                body.bendComplianceByNode[previous] * 100
            );
            body.maxBendAngleByNode[previous] = Math.max(
                body.maxBendAngleByNode[previous],
                Math.abs(effectiveNaturalTurn) * 180 / Math.PI + 0.5
            );
            body.setIntrinsicCurvatureTarget(
                previous,
                effectiveNaturalTurn,
                this._xpbdPigtailAxisX[previous],
                this._xpbdPigtailAxisY[previous],
                this._xpbdPigtailAxisZ[previous],
                intrinsicBendCompliance,
                materialProfile.intrinsicBendMaxCorrection ??
                    PREFORM_XPBD_DIRECTION_MAX_CORRECTION,
                0,
                voronoiLength
            );
            body.clearRestShapeTarget(index);
        }
        {
            // A preformed catheter is governed by intrinsic material
            // curvature. Positional shape matching is deliberately absent:
            // even a centroid-neutral target introduces a second potential
            // whose frame follows the current pose and can pump energy when
            // contact rotates the loop. The signed hinge targets above are
            // sufficient to form the natural arc and to deform under load.
            for (let index = baseIndex; index < count; index++) {
                body.clearRestShapeTarget(index);
            }
            body.restShapeTranslationNeutralStart = -1;
            body.restShapeTranslationNeutralEnd = -1;
        }
        if (
            reshapeReleasedPreformDuringMotion &&
            this._xpbdPigtailReferenceActive
        ) {
            this._xpbdPigtailReferenceProgress = this.progress;
            this._xpbdPigtailReferenceGuidewire = this.guidewireInserted;
        }
    }

    #applyKirchhoffMaterialShape(body, count) {
        // Legacy rest directions and world-space shape targets are a second
        // elastic potential. A Kirchhoff body owns one constitutive rest
        // strain field, independent of guidewire support and current pose.
        this.#clearXpbdPigtailDirectionTargets(body, count);
        for (let index = 0; index < count; index++) {
            body.clearRestShapeTarget(index);
            body.materialCoordinate[index] = this._centerlineDistances[index];
        }
        body.clearShapeClosureTarget?.();
        body.curvatureVariationEnabled = false;
        body.longStraightSpan = 0;
        this.#clearXpbdPigtailReference();

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
        body.bendComplianceByNode[target] = body.bendComplianceByNode[source];
        body.maxBendAngleByNode[target] = body.maxBendAngleByNode[source];
        if (body.rodModel === 'kirchhoff') {
            body.materialCoordinate[target] = body.materialCoordinate[source];
        }
        body.restBendChord[target] = body.restBendChord[source];
        body.bendLambda[target] = 0;
        body.restShapeEnabled[target] = body.restShapeEnabled[source];
        body.restShapeX[target] = body.restShapeX[source];
        body.restShapeY[target] = body.restShapeY[source];
        body.restShapeZ[target] = body.restShapeZ[source];
        body.restShapeCompliance[target] = body.restShapeCompliance[source];
        body.restShapeMaxCorrection[target] =
            body.restShapeMaxCorrection[source];
        if (this._xpbdPigtailReleaseX) {
            this._xpbdPigtailReleaseX[target] = this._xpbdPigtailReleaseX[source];
            this._xpbdPigtailReleaseY[target] = this._xpbdPigtailReleaseY[source];
            this._xpbdPigtailReleaseZ[target] = this._xpbdPigtailReleaseZ[source];
        }
    }

    #copyXpbdRestDirectionState(body, target, source) {
        body.restDirectionEnabled[target] = body.restDirectionEnabled[source];
        body.restDirectionRelative[target] = body.restDirectionRelative[source];
        body.restDirectionX[target] = body.restDirectionX[source];
        body.restDirectionY[target] = body.restDirectionY[source];
        body.restDirectionZ[target] = body.restDirectionZ[source];
        body.restDirectionCompliance[target] =
            body.restDirectionCompliance[source];
        body.restDirectionMaxCorrection[target] =
            body.restDirectionMaxCorrection[source];
        body.restDirectionDistalBias[target] =
            body.restDirectionDistalBias[source];
        body.restDirectionTurnAngle[target] =
            body.restDirectionTurnAngle[source];
        body.restDirectionAxisX[target] = body.restDirectionAxisX[source];
        body.restDirectionAxisY[target] = body.restDirectionAxisY[source];
        body.restDirectionAxisZ[target] = body.restDirectionAxisZ[source];
        body.intrinsicBendEnabled[target] = body.intrinsicBendEnabled[source];
        body.intrinsicCurvature[target] = body.intrinsicCurvature[source];
        body.restDirectionLambdaX[target] = 0;
        body.restDirectionLambdaY[target] = 0;
        body.restDirectionLambdaZ[target] = 0;
        if (body.rodModel === 'kirchhoff') {
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
                this.#copyXpbdRestDirectionState(body, segment, segment - 1);
            }
            // The inserted node splits the old segment nodeIndex - 1 in two.
            // Its distal half inherits that segment's current material frame;
            // retaining the frame formerly stored at nodeIndex associates the
            // new edge with the following material interval and injects an
            // artificial hinge on every feed topology change.
            if (
                body.rodModel === 'kirchhoff' &&
                nodeIndex > 0 &&
                nodeIndex <= lastSegment
            ) {
                this.#copyXpbdRestDirectionState(
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
                this.#copyXpbdRestDirectionState(body, segment, segment + 1);
            }
        }
        const firstChangedSegment = Math.max(0, nodeIndex - 1);
        const lastChangedSegment = Math.min(lastSegment, nodeIndex);
        for (
            let segment = firstChangedSegment;
            segment <= lastChangedSegment;
            segment++
        ) {
            body.restLength[segment] = Math.max(0.5, Math.abs(
                (this._centerlineDistances[segment + 1] ?? 0) -
                (this._centerlineDistances[segment] ?? 0)
            ));
        }
        const firstChangedBend = Math.max(1, nodeIndex - 1);
        const lastChangedBend = Math.min(count - 2, nodeIndex + 1);
        for (let index = firstChangedBend; index <= lastChangedBend; index++) {
            this.#initializeXpbdBendChord(body, points, index);
            body.clearRestDirectionTarget(index);
            if (body.rodModel === 'kirchhoff') {
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

    #initializeXpbdBendChord(body, points, index) {
        body.restBendChord[index] =
            body.restLength[index - 1] + body.restLength[index];
        body.bendLambda[index] = 0;
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
        body.restShapeMaxCorrection[index] = Infinity;
        body.shapeLambda[index] = 0;
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
        if (this.externalCollisionSolver) {
            this._pendingXpbdRotation += rotationDelta;
        }
    }

    stepPhysics(dt = 1 / 60, { collisions = true } = {}) {
        const state = this.#deploymentState();
        this.#updateGuidewireRelease(dt);
        this.#advanceXpbdPigtailRecovery(dt);
        const stepIndex = this._physicsStepIndex++;
        if (!this.externalCollisionSolver || (stepIndex & 3) === 0) this.#relaxSupportedPath(state.pathEnd);
        if (this.externalCollisionSolver) {
            this.#syncExternalFreeNodesFromXpbdBody();
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
        const nextGeometry = updateSmoothTubeGeometry(
            previousGeometry,
            this._renderPoints,
            {
                radius: PIGTAIL_CATHETER_RENDER_RADIUS_MM,
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
        const shapeRotation = this.externalCollisionSolver && this._xpbdDistalTwisted
            ? 0
            : this.rotation;
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
                const projectedPoint = this.externalCollisionSolver
                    ? point
                    : this.#projectInsideVesselDetailed(point).point;
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
            const firstDistance = Math.max(
                0.5,
                (this.freeNodes[1].distance ?? 0) -
                    (this.freeNodes[0].distance ?? 0)
            ) || this.freeNodeSpacing;
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
            const distalWeight = smoothstep(
                0,
                Math.max(this.freeNodeSpacing, freeLength),
                relativeDistance
            );
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
                if (this.externalCollisionSolver && this.pathSamples.length) {
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
        return this.externalCollisionSolver ? target : this.#projectInsideVesselDetailed(target).point;
    }

    #guideReleaseFrame(fallbackFrame) {
        const frame = this._guideReleaseFrameScratch;
        const useRecordedCatheterAxis =
            this.externalCollisionSolver &&
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

    #xpbdNaturalDistalBendChord(index) {
        if (index <= 0 || index >= this._centerlinePointCount - 1) return null;
        const previousDistance = this._centerlineDistances[index - 1];
        const distance = this._centerlineDistances[index];
        const nextDistance = this._centerlineDistances[index + 1];
        if (
            !Number.isFinite(previousDistance) ||
            !Number.isFinite(distance) ||
            !Number.isFinite(nextDistance)
        ) return null;
        const supportEnd = this.#sheathSupportEnd();
        const freeLength = Math.max(0, this.progress - supportEnd);
        const profile = catheterMaterialProfile(this.type);
        const arcLength = this.type === CATHETER_TYPE_BERENSTEIN
            ? Math.min(freeLength, BERENSTEIN_BEND_LENGTH)
            : this.type === CATHETER_TYPE_PIGTAIL
                ? Math.min(freeLength, PIGTAIL_ARC_LENGTH)
                : Math.min(freeLength, profile.naturalArcLength);
        if (arcLength <= 1e-4) return null;
        const arcStart = this.progress - arcLength;
        const overlap = Math.max(
            0,
            Math.min(nextDistance, this.progress) -
                Math.max(previousDistance, arcStart)
        );
        if (overlap <= 1e-4 || distance < arcStart - 0.5) return null;
        let curvature;
        if (this.type === CATHETER_TYPE_BERENSTEIN) {
            curvature = BERENSTEIN_BEND_ANGLE / BERENSTEIN_BEND_LENGTH;
        } else if (this.type === CATHETER_TYPE_PIGTAIL) {
            curvature = PIGTAIL_TURNS * Math.PI * 2 / PIGTAIL_ARC_LENGTH;
        } else {
            const voronoiLength = Math.max(
                0.5,
                nextDistance - previousDistance
            );
            curvature = Math.abs(profile.integrateIntrinsicTurn(
                Math.max(0, this.progress - distance),
                voronoiLength
            )) / voronoiLength;
        }
        const turnAngle = Math.min(
            Math.PI - 1e-3,
            curvature * overlap * 0.5
        );
        const previousLength = Math.max(0.5, distance - previousDistance);
        const nextLength = Math.max(0.5, nextDistance - distance);
        return Math.sqrt(Math.max(0,
            previousLength * previousLength +
            nextLength * nextLength +
            2 * previousLength * nextLength * Math.cos(turnAngle)
        ));
    }

    #distalShapeLength(freeLength) {
        const naturalLength = this.#naturalShapeLength();
        return Math.min(freeLength, naturalLength);
    }

    #naturalShapeLength() {
        return this.type === CATHETER_TYPE_PIGTAIL
            ? DISTAL_RELEASE_LENGTH
            : catheterMaterialProfile(this.type).naturalArcLength;
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
        const contactBand = Math.max(
            collision.clearance + this.freeNodeSpacing * 1.5,
            collision.clearance * 2
        );
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
        const guidePoint = this.#sampleGuidewire(distance, this._newNodeGuide);
        const body = this.physicsBody;
        if (
            this.externalCollisionSolver &&
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
