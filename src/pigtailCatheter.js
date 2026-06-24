import * as THREE from 'three';
import { clamp, smoothstep } from './mathUtils.js';

const CATHETER_RADIUS = 1.2;
const PIGTAIL_RADIUS = 7.2;
const PIGTAIL_TURNS = 1.05;
const DISTAL_RELEASE_LENGTH = 48;
const STRAIGHT_EXIT_LENGTH = 16;
const MIN_GUIDE_SUPPORT = 18;
const GUIDE_CAPTURE_TOLERANCE = 4;
const FREE_NODE_SPACING = 3.2;
const FREE_SHAPE_STIFFNESS = 76;
const FREE_SHAPE_POSITION_BLEND = 0.105;
const FREE_ANCHOR_STIFFNESS = 0.96;
const FREE_DAMPING = 0.88;
const FREE_CONSTRAINT_ITERATIONS = 18;
const FREE_BEND_SMOOTHING = 0.085;
const FREE_WALL_FRICTION = 0.08;
const GUIDEWIRE_RECAPTURE_BLEND = 0.92;
const GUIDEWIRE_RECAPTURE_WINDOW = 7;
const GUIDEWIRE_SUPPORT_ADVANCE_RATE = 42;
const GUIDEWIRE_SUPPORT_WITHDRAW_RATE = 95;
const TIP_FLEX_LENGTH = 46;
const TIP_FLEX_MIN_FRACTION = 0.34;
const TIP_FLEX_OVERHANG_FADE_START = 55;
const TIP_FLEX_OVERHANG_FADE_END = 145;
const PIGTAIL_RELEASE_CURL_START = 0.42;
const PIGTAIL_RELEASE_CURL_RATE = 0.82;
const GUIDEWIRE_IN_CATHETER_BLEND = 0.78;
const GUIDEWIRE_REACTION_BLEND = 0.16;
const EXTERNAL_CATHETER_VISIBLE_LENGTH = 90;
const ROTATION_SPEED = Math.PI * 0.9;
const CONTACT_CLEARANCE = CATHETER_RADIUS * 0.72;

function nodePosition(node) {
    return new THREE.Vector3(node.x, node.y, node.z);
}

export class PigtailCatheter {
    constructor({ wire, segmentLength, guidewireLength, tailProgressRef, vessel = null, maxLength = 360 }) {
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
        this.smoothedGuideSupportEnd = 0;
        this.motionCommand = 0;
        this.rotation = 0;
        this.pathSpacing = 4;
        this.pathSamples = [];
        this.freeNodes = [];
        this.freeRestDistances = [];
        this.freeLength = 0;
        this.speed = 32;
        this.material = new THREE.MeshBasicMaterial({
            color: 0xffffff,
            depthTest: false,
            transparent: true,
            opacity: 1
        });
        this.mesh = new THREE.Mesh(new THREE.BufferGeometry(), this.material);
        this.mesh.frustumCulled = false;
        this.mesh.renderOrder = 2;
        this.mesh.visible = false;
    }

    dispose() {
        this.mesh.geometry?.dispose?.();
        this.material.dispose();
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
        const nextProgress = clamp(this.progress + command * this.speed * dt, 0, this.maxLength);
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
        this.#updateGuideSupportEnd(dt);
    }

    rotate(command, dt) {
        if (!command) return;
        this.rotation += command * ROTATION_SPEED * dt;
    }

    stepPhysics(dt = 1 / 60) {
        const state = this.#deploymentState();
        if (state.freeLength < 2 || state.supportEnd <= 0) {
            this.freeNodes.length = 0;
            this.freeRestDistances.length = 0;
            this.freeLength = 0;
            return;
        }

        const frame = this.#freeFrame(state.supportEnd);
        this.#syncFreeNodes(state, frame);
        if (this.freeNodes.length < 2) return;

        this.#recaptureFreeNodes(state);

        const anchor = frame.supportTip;
        const prevPositions = this.freeNodes.map(node => node.pos.clone());
        this.freeNodes[0].pos.copy(anchor);
        this.freeNodes[0].vel.set(0, 0, 0);

        for (let i = 1; i < this.freeNodes.length; i++) {
            const node = this.freeNodes[i];
            node.curl = Math.min(1, (node.curl ?? 1) + PIGTAIL_RELEASE_CURL_RATE * dt);
            const relativeDistance = Math.max(0, (node.distance ?? 0) - (this.freeNodes[0].distance ?? 0));
            const rest = this.#freeRestPoint(relativeDistance, frame, state.freeLength, node.curl);
            node.vel.addScaledVector(new THREE.Vector3().subVectors(rest, node.pos), FREE_SHAPE_STIFFNESS * dt);
            node.vel.multiplyScalar(FREE_DAMPING);
            node.pos.addScaledVector(node.vel, dt);
        }

        for (let iter = 0; iter < FREE_CONSTRAINT_ITERATIONS; iter++) {
            this.freeNodes[0].pos.copy(anchor);
            this.#recaptureFreeNodes(state);
            this.#solveFreeLengthConstraints();
            this.#solveFreeShape(frame, state.freeLength);
            this.#solveFreeBending(frame);
            this.#collideFreeNodes();
        }

        const invDt = 1 / Math.max(1e-4, dt);
        for (let i = 1; i < this.freeNodes.length; i++) {
            const node = this.freeNodes[i];
            node.vel.subVectors(node.pos, prevPositions[i]).multiplyScalar(invDt * FREE_DAMPING);
        }
        this.freeNodes[0].vel.set(0, 0, 0);
    }

    constrainGuidewire(dt = 1 / 60) {
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
            const correction = target.clone().sub(new THREE.Vector3(oldX, oldY, oldZ));

            node.x = oldX + correction.x * weight;
            node.y = oldY + correction.y * weight;
            node.z = oldZ + correction.z * weight;

            const invDt = 1 / Math.max(1e-4, dt);
            node.vx = (node.x - oldX) * invDt * 0.25;
            node.vy = (node.y - oldY) * invDt * 0.25;
            node.vz = (node.z - oldZ) * invDt * 0.25;

            this.#applyGuidewireReaction(distance, correction.multiplyScalar(weight));
        }
    }

    updateMesh() {
        const points = this.#buildCenterline();
        if (points.length < 2) {
            this.mesh.visible = false;
            return;
        }

        const curve = new THREE.CatmullRomCurve3(points, false, 'centripetal', 0.35);
        const tubularSegments = clamp(Math.round(points.length * 2.6), 24, 180);
        const geometry = new THREE.TubeGeometry(curve, tubularSegments, CATHETER_RADIUS, 10, false);
        this.mesh.geometry.dispose();
        this.mesh.geometry = geometry;
        this.mesh.visible = true;
    }

    #buildCenterline() {
        const state = this.#deploymentState();
        const externalLength = this.sheathPath ? EXTERNAL_CATHETER_VISIBLE_LENGTH : 0;
        if (state.pathEnd <= 0 && externalLength <= 0) return [];
        const shaftEnd = Math.max(0, state.supportEnd);
        const shaftSamples = shaftEnd > 0 ? clamp(Math.ceil(shaftEnd / 5), 1, 90) : 0;
        const points = [];

        if (externalLength > 0) {
            const externalSamples = clamp(Math.ceil(externalLength / 6), 2, 24);
            for (let i = 0; i <= externalSamples; i++) {
                const s = -externalLength + externalLength * i / externalSamples;
                points.push(this.#sampleCatheterPath(s));
            }
        }

        if (state.pathEnd <= 0) return points;

        const shaftStartIndex = points.length ? 1 : 0;
        for (let i = shaftStartIndex; i <= shaftSamples; i++) {
            const s = shaftSamples > 0 ? shaftEnd * i / shaftSamples : 0;
            points.push(this.#sampleCatheterPath(s));
        }

        if (state.freeLength < 2) {
            if (state.pathEnd > shaftEnd + 0.5) {
                points.push(this.#sampleCatheterPath(state.pathEnd));
            }
            return points;
        }

        const frame = this.#freeFrame(state.supportEnd);
        this.#syncFreeNodes(state, frame);
        for (let i = 1; i < this.freeNodes.length; i++) {
            points.push(this.freeNodes[i].pos.clone());
        }
        return points;
    }

    #deploymentState() {
        if (this.progress < 4) {
            return { pathEnd: 0, supportEnd: 0, freeLength: 0 };
        }

        const sheathSupportEnd = this.#sheathSupportEnd();
        const pathEnd = Math.max(sheathSupportEnd, Math.min(this.progress, this.#pathEndDistance()));
        if (pathEnd <= 0) return { pathEnd: 0, supportEnd: 0, freeLength: 0 };
        const guideEnd = Math.min(this.guidewireInserted, this.#pathEndDistance());
        const freeBeyondGuide = Math.max(0, this.progress - guideEnd);
        const tipCapturedByGuidewire = freeBeyondGuide <= GUIDE_CAPTURE_TOLERANCE;
        const elasticTipLength = this.#tipFlexLength(pathEnd, guideEnd);
        const movingOnGuidewire = Math.abs(this.motionCommand) > 0
            && this.guidewireInserted > MIN_GUIDE_SUPPORT
            && tipCapturedByGuidewire;
        const guideSupportEnd = movingOnGuidewire
            ? pathEnd
            : Math.max(sheathSupportEnd, clamp(this.smoothedGuideSupportEnd, 0, Math.min(pathEnd, guideEnd)));
        const supportEnd = elasticTipLength > 0
            ? Math.max(sheathSupportEnd, Math.min(guideSupportEnd, pathEnd - elasticTipLength))
            : guideSupportEnd;
        const freeLength = Math.max(0, this.progress - supportEnd);
        return { pathEnd, supportEnd, freeLength };
    }

    #tipFlexLength(pathEnd, guideEnd) {
        const sheathLength = this.sheathPath?.length || 0;
        if (pathEnd <= sheathLength + 6 || guideEnd < pathEnd - GUIDE_CAPTURE_TOLERANCE) return 0;

        const available = Math.max(0, pathEnd - sheathLength);
        const overhang = Math.max(0, guideEnd - pathEnd);
        const overhangFade = 1 - smoothstep(TIP_FLEX_OVERHANG_FADE_START, TIP_FLEX_OVERHANG_FADE_END, overhang);
        const fraction = TIP_FLEX_MIN_FRACTION + (1 - TIP_FLEX_MIN_FRACTION) * overhangFade;
        return Math.min(TIP_FLEX_LENGTH * fraction, available);
    }

    #updateGuideSupportEnd(dt) {
        const pathEnd = Math.min(this.progress, this.#pathEndDistance());
        if (pathEnd <= 0 || this.guidewireInserted <= MIN_GUIDE_SUPPORT) {
            this.smoothedGuideSupportEnd = 0;
            return;
        }

        const guideEnd = Math.min(this.guidewireInserted, pathEnd);
        const freeBeyondGuide = Math.max(0, this.progress - guideEnd);
        const tipCapturedByGuidewire = freeBeyondGuide <= GUIDE_CAPTURE_TOLERANCE;
        const catheterMovingOnGuidewire = Math.abs(this.motionCommand) > 0
            && this.guidewireInserted > MIN_GUIDE_SUPPORT
            && tipCapturedByGuidewire;

        if (catheterMovingOnGuidewire) {
            this.smoothedGuideSupportEnd = pathEnd;
            return;
        }

        const current = clamp(this.smoothedGuideSupportEnd, 0, pathEnd);
        const target = guideEnd;
        const rate = target > current ? GUIDEWIRE_SUPPORT_ADVANCE_RATE : GUIDEWIRE_SUPPORT_WITHDRAW_RATE;
        const maxStep = rate * Math.max(0, dt);
        if (Math.abs(target - current) <= maxStep) {
            this.smoothedGuideSupportEnd = target;
        } else {
            this.smoothedGuideSupportEnd = current + Math.sign(target - current) * maxStep;
        }
    }

    #freeFrame(supportEnd) {
        const supportTip = this.#sampleCatheterPath(supportEnd);
        const beforeTip = this.#sampleCatheterPath(Math.max(0, supportEnd - 10));
        const beforePlane = this.#sampleCatheterPath(Math.max(0, supportEnd - 28));
        const tangent = new THREE.Vector3().subVectors(supportTip, beforeTip);
        if (tangent.lengthSq() < 1e-5) tangent.set(0, 1, 0);
        tangent.normalize();
        const normal = this.#catheterPlaneNormal(tangent, beforeTip, beforePlane);
        normal.applyAxisAngle(tangent, this.rotation).normalize();
        return { supportTip, tangent, normal };
    }

    #syncFreeNodes(state, frame) {
        const distances = [state.supportEnd];
        let d = state.supportEnd;
        while (d + FREE_NODE_SPACING < this.progress - 0.5) {
            d += FREE_NODE_SPACING;
            distances.push(d);
        }
        if (this.progress > distances[distances.length - 1] + 0.5) {
            distances.push(this.progress);
        }

        const oldNodes = this.freeNodes;
        const used = new Set();
        const nextNodes = [];

        for (const distance of distances) {
            const relativeDistance = distance - state.supportEnd;
            let bestIndex = -1;
            let bestDelta = Infinity;
            for (let i = 0; i < oldNodes.length; i++) {
                if (used.has(i)) continue;
                const delta = Math.abs((oldNodes[i].distance ?? 0) - distance);
                if (delta < bestDelta) {
                    bestDelta = delta;
                    bestIndex = i;
                }
            }

            let node;
            if (bestIndex >= 0 && bestDelta <= FREE_NODE_SPACING * 0.7) {
                node = oldNodes[bestIndex];
                used.add(bestIndex);
            } else {
                const wasJustReleased = this.guidewireDelta < -1e-4
                    && distance >= this.guidewireInserted - GUIDE_CAPTURE_TOLERANCE
                    && distance <= this.previousGuidewireInserted + GUIDE_CAPTURE_TOLERANCE;
                const restPoint = this.#freeRestPoint(relativeDistance, frame, state.freeLength, wasJustReleased ? PIGTAIL_RELEASE_CURL_START : 1);
                const pathPoint = this.#sampleCatheterPath(Math.min(distance, this.#pathEndDistance()));
                const point = wasJustReleased
                    ? pathPoint.lerp(restPoint, PIGTAIL_RELEASE_CURL_START)
                    : restPoint;
                node = {
                    pos: this.#projectInsideVesselDetailed(point).point,
                    vel: new THREE.Vector3(),
                    distance,
                    curl: wasJustReleased ? PIGTAIL_RELEASE_CURL_START : 1
                };
            }
            node.distance = distance;
            node.curl = node.curl ?? 1;
            nextNodes.push(node);
        }

        this.freeNodes = nextNodes;
        this.freeRestDistances = distances;
        this.freeLength = state.freeLength;
        if (this.freeNodes[0]) {
            this.freeNodes[0].pos.copy(frame.supportTip);
            this.freeNodes[0].vel.set(0, 0, 0);
        }
    }

    #recaptureFreeNodes(state) {
        const capturedLength = Math.max(0, this.guidewireInserted - state.supportEnd);
        if (capturedLength <= 0 || this.freeNodes.length < 2) return;
        const captureFront = Math.min(this.guidewireInserted, state.supportEnd + GUIDEWIRE_RECAPTURE_WINDOW);

        for (let i = 1; i < this.freeNodes.length; i++) {
            const pathDistance = this.freeNodes[i].distance ?? state.supportEnd;
            if (pathDistance > captureFront + GUIDE_CAPTURE_TOLERANCE) continue;
            const fade = 1 - smoothstep(captureFront - GUIDE_CAPTURE_TOLERANCE, captureFront + GUIDE_CAPTURE_TOLERANCE, pathDistance);
            const target = this.#sampleGuidewire(pathDistance);
            const blend = GUIDEWIRE_RECAPTURE_BLEND * fade;
            this.freeNodes[i].pos.lerp(target, blend);
            this.freeNodes[i].vel.multiplyScalar(1 - blend);
        }
    }

    #freeRestPoint(distance, frame, freeLength, curlScale = 1) {
        const deployLength = Math.min(freeLength, DISTAL_RELEASE_LENGTH);
        const proximalFreeLength = Math.max(0, freeLength - deployLength);
        if (distance <= proximalFreeLength) {
            return frame.supportTip.clone().addScaledVector(frame.tangent, distance);
        }

        const curlBase = frame.supportTip.clone().addScaledVector(frame.tangent, proximalFreeLength);
        const local = distance - proximalFreeLength;
        const unsupportedCurlLength = Math.max(0, deployLength - STRAIGHT_EXIT_LENGTH);
        const curlProgress = smoothstep(0, DISTAL_RELEASE_LENGTH - STRAIGHT_EXIT_LENGTH, unsupportedCurlLength) * curlScale;
        const leadLength = Math.min(deployLength, STRAIGHT_EXIT_LENGTH + unsupportedCurlLength * 0.18);
        if (local <= leadLength || curlProgress <= 0.001) {
            return curlBase.clone().addScaledVector(frame.tangent, local);
        }

        const loopBase = curlBase.clone().addScaledVector(frame.tangent, leadLength);
        const center = loopBase.clone().addScaledVector(frame.normal, -PIGTAIL_RADIUS * curlProgress);
        const loopLength = Math.max(1e-4, deployLength - leadLength);
        const u = clamp((local - leadLength) / loopLength, 0, 1);
        const theta = PIGTAIL_TURNS * Math.PI * 2 * curlProgress * u;
        const taper = 1 - u;
        const radius = PIGTAIL_RADIUS * curlProgress * (0.72 + taper * 0.28);
        return center
            .addScaledVector(frame.normal, Math.cos(theta) * radius)
            .addScaledVector(frame.tangent, Math.sin(theta) * radius);
    }

    #solveFreeLengthConstraints() {
        for (let i = 1; i < this.freeNodes.length; i++) {
            const prev = this.freeNodes[i - 1];
            const node = this.freeNodes[i];
            const desired = Math.max(0.5, (node.distance ?? 0) - (prev.distance ?? 0));
            const delta = new THREE.Vector3().subVectors(node.pos, prev.pos);
            const dist = delta.length();
            if (dist < 1e-5) continue;
            const correction = (dist - desired) / dist;
            if (i === 1) {
                node.pos.addScaledVector(delta, -correction);
            } else {
                prev.pos.addScaledVector(delta, correction * 0.5);
                node.pos.addScaledVector(delta, -correction * 0.5);
            }
        }
    }

    #solveFreeBending(frame) {
        if (this.freeNodes.length > 1) {
            const firstDistance = Math.max(0.5, (this.freeNodes[1].distance ?? 0) - (this.freeNodes[0].distance ?? 0)) || FREE_NODE_SPACING;
            const firstTarget = frame.supportTip.clone().addScaledVector(frame.tangent, firstDistance);
            this.freeNodes[1].pos.lerp(firstTarget, FREE_ANCHOR_STIFFNESS);
        }

        for (let i = 2; i < this.freeNodes.length - 1; i++) {
            const prev = this.freeNodes[i - 1].pos;
            const next = this.freeNodes[i + 1].pos;
            const midpoint = prev.clone().add(next).multiplyScalar(0.5);
            this.freeNodes[i].pos.lerp(midpoint, FREE_BEND_SMOOTHING);
        }
    }

    #solveFreeShape(frame, freeLength) {
        for (let i = 1; i < this.freeNodes.length; i++) {
            const relativeDistance = Math.max(0, (this.freeNodes[i].distance ?? 0) - (this.freeNodes[0].distance ?? 0));
            const target = this.#freeRestPoint(relativeDistance, frame, freeLength, this.freeNodes[i].curl ?? 1);
            const distalWeight = smoothstep(0, Math.max(FREE_NODE_SPACING, freeLength), relativeDistance);
            this.freeNodes[i].pos.lerp(target, FREE_SHAPE_POSITION_BLEND * (0.45 + distalWeight * 0.55));
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
            normal: best?.normal || new THREE.Vector3(1, 0, 0),
            collided: !!best
        };
    }

    #meshProjection(point, collision) {
        if (collision.meshCollider?.pointContact) {
            const contact = collision.meshCollider.pointContact(point, collision.clearance);
            return {
                point: contact.violation ? contact.target.clone() : point.clone(),
                normal: contact.normal?.clone?.() || new THREE.Vector3(1, 0, 0),
                collided: !!contact.violation
            };
        }

        const closest = new THREE.Vector3();
        const hit = collision.geometry.boundsTree.closestPointToPoint(point, { point: closest });
        const dist = hit?.distance ?? point.distanceTo(closest);
        const contactBand = Math.max(collision.clearance + FREE_NODE_SPACING * 1.5, collision.clearance * 2);
        if (dist > contactBand) {
            return {
                point: point.clone(),
                normal: new THREE.Vector3(1, 0, 0),
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
        const rel = new THREE.Vector3().subVectors(point, seg.start);
        const axial = clamp(rel.dot(seg.dir), 0, seg.length);
        const center = seg.start.clone().addScaledVector(seg.dir, axial);
        const radial = new THREE.Vector3().subVectors(point, center);
        const radialDist = radial.length();
        const radius = Math.max(0.6, seg.radius - CONTACT_CLEARANCE);
        const inside = radialDist <= radius;
        const normal = radialDist > 1e-6 ? radial.multiplyScalar(1 / radialDist) : this.#fallbackNormal(seg.dir);
        if (inside) return { inside, point: point.clone(), distance: 0, normal };
        const projected = center.addScaledVector(normal, radius);
        return { inside: false, point: projected, distance: point.distanceTo(projected), normal };
    }

    #sphereProjection(point, sphere) {
        const radial = new THREE.Vector3().subVectors(point, sphere.center);
        const dist = radial.length();
        const radius = Math.max(0.6, sphere.radius - CONTACT_CLEARANCE);
        const inside = dist <= radius;
        const normal = dist > 1e-6 ? radial.multiplyScalar(1 / dist) : new THREE.Vector3(1, 0, 0);
        if (inside) return { inside, point: point.clone(), distance: 0, normal };
        const projected = sphere.center.clone().addScaledVector(normal, radius);
        return { inside: false, point: projected, distance: point.distanceTo(projected), normal };
    }

    #fallbackNormal(dir) {
        const helper = Math.abs(dir.y) < 0.85 ? new THREE.Vector3(0, 1, 0) : new THREE.Vector3(1, 0, 0);
        return new THREE.Vector3().crossVectors(dir, helper).normalize();
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
            const axis = new THREE.Vector3().subVectors(end, start);
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
        const dir = new THREE.Vector3().subVectors(end, start);
        const length = dir.length();
        if (length < 1e-6) return null;
        dir.multiplyScalar(1 / length);
        return { start, end, dir, length };
    }

    #sheathSupportEnd() {
        if (!this.sheathPath) return 0;
        return Math.min(this.progress, this.sheathPath.length);
    }

    #sampleSheathPath(insertedDistance) {
        if (!this.sheathPath) return null;
        const d = clamp(insertedDistance, 0, this.sheathPath.length);
        return this.sheathPath.start.clone().addScaledVector(this.sheathPath.dir, d);
    }

    #recordGuidewirePath(targetDistance) {
        const sheathLength = this.sheathPath?.length || 0;
        if (targetDistance <= sheathLength + 0.5) return;
        if (!this.pathSamples.length) {
            this.pathSamples.push({ distance: sheathLength, point: this.#sampleGuidewire(sheathLength) });
        }
        let distance = this.#pathEndDistance();
        while (distance + this.pathSpacing < targetDistance) {
            distance += this.pathSpacing;
            this.pathSamples.push({ distance, point: this.#sampleGuidewire(distance) });
        }
        if (targetDistance > this.#pathEndDistance() + 0.5) {
            this.pathSamples.push({ distance: targetDistance, point: this.#sampleGuidewire(targetDistance) });
        }
    }

    #refreshGuidewirePath(targetDistance) {
        const sheathLength = this.sheathPath?.length || 0;
        if (targetDistance <= sheathLength + 0.5) return;
        this.#recordGuidewirePath(targetDistance);
        for (const sample of this.pathSamples) {
            if (sample.distance <= sheathLength + 0.5 || sample.distance > targetDistance + 0.5) continue;
            sample.point.copy(this.#sampleGuidewire(sample.distance));
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

    #sampleCatheterPath(insertedDistance) {
        const sheathLength = this.sheathPath?.length || 0;
        if (this.sheathPath && insertedDistance < 0) {
            return this.sheathPath.start.clone().addScaledVector(this.sheathPath.dir, insertedDistance);
        }
        if (this.sheathPath && insertedDistance <= sheathLength + 0.5) {
            return this.#sampleSheathPath(insertedDistance);
        }
        if (!this.pathSamples.length) {
            const sheathTip = this.#sampleSheathPath(sheathLength);
            if (sheathTip) return sheathTip;
            return this.#sampleGuidewire(insertedDistance);
        }
        const target = clamp(insertedDistance, 0, this.#pathEndDistance());
        let prev = this.pathSamples[0];
        for (let i = 1; i < this.pathSamples.length; i++) {
            const next = this.pathSamples[i];
            if (next.distance >= target) {
                const t = clamp((target - prev.distance) / Math.max(1e-6, next.distance - prev.distance), 0, 1);
                return prev.point.clone().lerp(next.point, t);
            }
            prev = next;
        }
        return prev.point.clone();
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

    #applyGuidewireReaction(insertedDistance, guideCorrection) {
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
                const reaction = guideCorrection.clone().multiplyScalar(-GUIDEWIRE_REACTION_BLEND);
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
        tip.pos.addScaledVector(guideCorrection, -GUIDEWIRE_REACTION_BLEND);
        tip.vel.addScaledVector(guideCorrection, -0.18);
    }

    #catheterPlaneNormal(tangent, beforeTip, beforePlane) {
        const previousTangent = new THREE.Vector3().subVectors(beforeTip, beforePlane);
        if (previousTangent.lengthSq() > 1e-5) {
            previousTangent.normalize();
            const curvature = new THREE.Vector3().subVectors(tangent, previousTangent);
            curvature.addScaledVector(tangent, -curvature.dot(tangent));
            if (curvature.lengthSq() > 1e-5) return curvature.normalize();
        }

        const helper = Math.abs(tangent.y) < 0.85
            ? new THREE.Vector3(0, 1, 0)
            : new THREE.Vector3(1, 0, 0);
        return new THREE.Vector3()
            .crossVectors(tangent, helper)
            .cross(tangent)
            .normalize();
    }

    #sampleGuidewire(insertedDistance) {
        const tailProgress = this.tailProgressRef();
        const targetCoord = insertedDistance;
        const nodes = this.wire.nodes;
        let prevIndex = 0;
        let prevCoord = this.#nodeInsertedCoordinate(0, tailProgress);

        for (let i = 1; i < nodes.length; i++) {
            const coord = this.#nodeInsertedCoordinate(i, tailProgress);
            if (coord >= targetCoord) {
                const a = nodePosition(nodes[prevIndex]);
                const b = nodePosition(nodes[i]);
                const t = clamp((targetCoord - prevCoord) / Math.max(1e-6, coord - prevCoord), 0, 1);
                return a.lerp(b, t);
            }
            prevIndex = i;
            prevCoord = coord;
        }

        return nodePosition(nodes[nodes.length - 1]);
    }

    #nodeInsertedCoordinate(index, tailProgress) {
        return this.segmentLength * index - this.guidewireLength + tailProgress;
    }
}
