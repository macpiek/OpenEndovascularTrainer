import * as THREE from 'three';
import { Brush, Evaluator, ADDITION } from 'https://unpkg.com/three-bvh-csg@0.0.17/build/index.module.js';

function verifyManifold(geometry) {
    const index = geometry.index;
    if (!index) return 1;
    const count = geometry.attributes.position.count;
    const visited = new Array(count).fill(false);
    const adj = Array.from({length: count}, () => []);
    const arr = index.array;
    for (let i = 0; i < arr.length; i += 3) {
        const a = arr[i], b = arr[i + 1], c = arr[i + 2];
        adj[a].push(b, c);
        adj[b].push(a, c);
        adj[c].push(a, b);
    }
    let components = 0;
    const stack = [];
    for (let i = 0; i < count; i++) {
        if (!visited[i]) {
            components++;
            stack.push(i);
            visited[i] = true;
            while (stack.length) {
                const v = stack.pop();
                for (const n of adj[v]) {
                    if (!visited[n]) {
                        visited[n] = true;
                        stack.push(n);
                    }
                }
            }
        }
    }
    if (components > 1) {
        console.warn(`Geometry has ${components} disconnected components`);
    }
    return components;
}

function createTaperedTube(path, tubularSegments, radialSegments, startRadius, endRadius) {
    const geometry = new THREE.TubeGeometry(path, tubularSegments, 1, radialSegments, false);
    const pos = geometry.attributes.position;
    const normals = geometry.attributes.normal;
    const segments = tubularSegments + 1;
    const radials = radialSegments + 1;
    for (let i = 0; i < segments; i++) {
        const t = i / tubularSegments;
        const r = startRadius + (endRadius - startRadius) * t;
        for (let j = 0; j < radials; j++) {
            const idx = i * radials + j;
            pos.setX(idx, pos.getX(idx) + normals.getX(idx) * (r - 1));
            pos.setY(idx, pos.getY(idx) + normals.getY(idx) * (r - 1));
            pos.setZ(idx, pos.getZ(idx) + normals.getZ(idx) * (r - 1));
        }
    }
    pos.needsUpdate = true;
    geometry.computeVertexNormals();
    return geometry;
}

function createBranchingSegment(mainRadius, branchRadius, branchPointY, branchLength, blend, branchAngleOffset) {
    const trunkHeight = Math.abs(branchPointY);
    const trunkGeom = new THREE.CylinderGeometry(mainRadius, mainRadius, trunkHeight, 16, 1, true);
    trunkGeom.translate(0, branchPointY / 2, 0);

    const angleBase = Math.PI / 6;
    const makeCurve = angle => new THREE.QuadraticBezierCurve3(
        new THREE.Vector3(0, branchPointY, 0),
        new THREE.Vector3(Math.sin(angle) * blend, branchPointY - blend, 0),
        new THREE.Vector3(Math.sin(angle) * (blend + branchLength), branchPointY - (blend + branchLength), 0)
    );

    const rightCurve = makeCurve(angleBase + branchAngleOffset);
    const leftCurve = makeCurve(-angleBase - branchAngleOffset);

    const rightGeom = createTaperedTube(rightCurve, 64, 16, mainRadius, branchRadius);
    const leftGeom = createTaperedTube(leftCurve, 64, 16, mainRadius, branchRadius);

    const trunkBrush = new Brush(trunkGeom);
    const rightBrush = new Brush(rightGeom);
    const leftBrush = new Brush(leftGeom);
    trunkBrush.updateMatrixWorld();
    rightBrush.updateMatrixWorld();
    leftBrush.updateMatrixWorld();

    const evaluator = new Evaluator();
    const result1 = evaluator.evaluate(trunkBrush, rightBrush, ADDITION);
    result1.updateMatrixWorld();
    const result = evaluator.evaluate(result1, leftBrush, ADDITION);
    const geometry = result.geometry;
    geometry.computeVertexNormals();
    verifyManifold(geometry);
    return geometry;
}

/**
 * Generates a branched vessel with deterministic parameters.
 * Defaults produce repeatable geometry; modify arguments to change it explicitly.
 * @param {number} branchLength length of each branch in units (default 140)
 * @param {number} branchAngleOffset angle offset in radians for branches (default 0)
 * @returns {{vessel: object, geometry: THREE.BufferGeometry}}
 */
export function generateVessel(branchLength = 140, branchAngleOffset = 0) {
    const mainRadius = 20;
    const branchRadius = mainRadius / 2;
    const branchPointY = -180;
    const blend = 40;

    const vessel = {
        radius: mainRadius,
        branchRadius,
        branchPoint: {x: 0, y: branchPointY, z: 0},
        segments: []
    };

    const mainStart = {x: 0, y: 0, z: 0};
    const mainEnd = {x: 0, y: branchPointY + blend, z: 0};
    vessel.main = {start: mainStart, end: mainEnd};
    vessel.segments.push({start: mainStart, end: mainEnd, radius: mainRadius});

    function branch(dir) {
        const angle = Math.PI / 6 * dir + branchAngleOffset * dir;
        const curveEnd = {
            x: Math.sin(angle) * blend,
            y: branchPointY - blend,
            z: 0
        };
        const end = {
            x: Math.sin(angle) * (blend + branchLength),
            y: branchPointY - (blend + branchLength),
            z: 0
        };
        const length = branchLength + blend;
        return {angle, curveEnd, end, length};
    }

    vessel.right = branch(1);
    vessel.left = branch(-1);

    function addCurve(p0, p1, p2) {
        const steps = 24;
        let prev = p0;
        for (let i = 1; i <= steps; i++) {
            const t = i / steps;
            const tt = 1 - t;
            const p = {
                x: tt * tt * p0.x + 2 * tt * t * p1.x + t * t * p2.x,
                y: tt * tt * p0.y + 2 * tt * t * p1.y + t * t * p2.y,
                z: tt * tt * p0.z + 2 * tt * t * p1.z + t * t * p2.z
            };
            const r = mainRadius + (branchRadius - mainRadius) * t;
            vessel.segments.push({start: prev, end: p, radius: r});
            prev = p;
        }
    }

    addCurve(mainEnd, vessel.branchPoint, vessel.right.curveEnd);
    vessel.segments.push({start: vessel.right.curveEnd, end: vessel.right.end, radius: branchRadius});
    addCurve(mainEnd, vessel.branchPoint, vessel.left.curveEnd);
    vessel.segments.push({start: vessel.left.curveEnd, end: vessel.left.end, radius: branchRadius});

    const geometry = createBranchingSegment(mainRadius, branchRadius, branchPointY, branchLength, blend, branchAngleOffset);
    return { vessel, geometry };
}

