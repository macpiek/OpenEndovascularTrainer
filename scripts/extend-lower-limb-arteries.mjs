import crypto from 'node:crypto';
import { execFileSync } from 'node:child_process';
import fs from 'node:fs';
import * as THREE from 'three';
import ManifoldModule from 'manifold-3d';
import { STLExporter } from 'three/examples/jsm/exporters/STLExporter.js';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import { mergeVertices } from 'three/examples/jsm/utils/BufferGeometryUtils.js';
import {
    AORTA_MODEL_URL,
    transformAortaGeometry
} from '../src/aortaTransform.js';
import { decodeCollisionAsset } from '../src/physics/collision/collisionAssetFormat.js';
import {
    createLowerLimbArteryPaths,
    findLowerLimbAttachmentRoots
} from '../src/lowerLimbArteries.js';
import {
    createHeadArteryPaths,
    findHeadArteryAttachmentRoots,
    HEAD_ARTERY_SUPERIOR_Y_MM
} from '../src/headArteries.js';
import {
    createUpperLimbArteryPaths,
    findUpperLimbAttachmentRoots,
    UPPER_LIMB_HAND_DISTAL_Y_MM,
    UPPER_LIMB_LATERAL_EXTENT_MM
} from '../src/upperLimbArteries.js';
import { generateVessel } from '../src/vesselGeometry.js';

const SOURCE_PATH = 'res/Aorta_plain.stl';
const COLLISION_PATH = 'res/Aorta_plain.collision.bin';
const TEMP_PATH = 'res/Aorta_plain.extended-anatomy.tmp.stl';
const BASE_STL_SHA256 = '60e84c7cf241948b552d3753818229cf93601f06fc62e6123397f5e7e9dc20da';
const RADIAL_SEGMENTS = 16;
const AXIAL_SAMPLE_SPACING_MM = 6;
const MINIMUM_WALL_THICKNESS_MM = 0.55;
const MAXIMUM_WALL_THICKNESS_MM = 0.95;

function arrayBuffer(bytes) {
    return bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength);
}

function sha256(bytes) {
    return crypto.createHash('sha256').update(bytes).digest('hex');
}

function readOriginDevAsset(path) {
    return execFileSync('git', ['show', `origin/dev:${path}`], {
        encoding: 'buffer',
        maxBuffer: 64 * 1024 * 1024,
        stdio: ['ignore', 'pipe', 'ignore']
    });
}

function loadCenterlineSegments(asset) {
    const data = asset.arrays.centerlineSegments;
    const edges = asset.arrays.centerlineEdges;
    const stride = asset.metadata.centerline.stride;
    return Array.from({ length: data.length / stride }, (_, id) => {
        const offset = id * stride;
        return {
            id,
            start: new THREE.Vector3(
                data[offset], data[offset + 1], data[offset + 2]
            ),
            end: new THREE.Vector3(
                data[offset + 3], data[offset + 4], data[offset + 5]
            ),
            radiusStart: data[offset + 6],
            radiusEnd: data[offset + 7],
            nodeStartId: edges[id * 2],
            nodeEndId: edges[id * 2 + 1]
        };
    });
}

function wallThickness(radius) {
    return THREE.MathUtils.clamp(
        radius * 0.28,
        MINIMUM_WALL_THICKNESS_MM,
        MAXIMUM_WALL_THICKNESS_MM
    );
}

function tubeDefinition(pathDefinition, {
    outerWall = false,
    capTerminal = false
} = {}) {
    const sourcePoints = outerWall
        ? pathDefinition.points.slice(pathDefinition.outerStartIndex || 0)
        : pathDefinition.points;
    const points = sourcePoints.map(point => ({
        position: point.position.clone(),
        radius: point.radius + (outerWall ? wallThickness(point.radius) : 0)
    }));
    if (outerWall && capTerminal) {
        const last = points.at(-1);
        const previous = points.at(-2);
        const tangent = last.position.clone()
            .sub(previous.position)
            .normalize();
        points.push({
            position: last.position.clone().addScaledVector(
                tangent,
                wallThickness(pathDefinition.points.at(-1).radius)
            ),
            radius: last.radius
        });
    }
    return { ...pathDefinition, points };
}

function taperedTubeGeometry(pathDefinition) {
    const controlPoints = pathDefinition.points.map(point => point.position);
    const curve = new THREE.CatmullRomCurve3(
        controlPoints,
        false,
        'centripetal',
        0.5
    );
    const controlDistances = [0];
    for (let index = 1; index < controlPoints.length; index++) {
        controlDistances.push(
            controlDistances[index - 1] +
            controlPoints[index].distanceTo(controlPoints[index - 1])
        );
    }
    const controlLength = controlDistances.at(-1);
    const tubularSegments = Math.max(
        12,
        Math.ceil(curve.getLength() / AXIAL_SAMPLE_SPACING_MM)
    );
    const frames = curve.computeFrenetFrames(tubularSegments, false);
    const ringSize = RADIAL_SEGMENTS;
    const ringCount = tubularSegments + 1;
    const capStartIndex = ringCount * ringSize;
    const capEndIndex = capStartIndex + 1;
    const positions = new Float32Array((capEndIndex + 1) * 3);
    const point = new THREE.Vector3();
    let radiusCursor = 0;

    for (let ring = 0; ring < ringCount; ring++) {
        const u = ring / tubularSegments;
        curve.getPointAt(u, point);
        const distance = u * controlLength;
        while (
            radiusCursor + 1 < controlDistances.length - 1 &&
            controlDistances[radiusCursor + 1] < distance
        ) radiusCursor++;
        const fromDistance = controlDistances[radiusCursor];
        const toDistance = controlDistances[radiusCursor + 1];
        const radiusT = THREE.MathUtils.clamp(
            (distance - fromDistance) /
                Math.max(1e-6, toDistance - fromDistance),
            0,
            1
        );
        const radius = THREE.MathUtils.lerp(
            pathDefinition.points[radiusCursor].radius,
            pathDefinition.points[radiusCursor + 1].radius,
            radiusT
        );
        const normal = frames.normals[ring];
        const binormal = frames.binormals[ring];
        for (let radial = 0; radial < RADIAL_SEGMENTS; radial++) {
            const angle = radial / RADIAL_SEGMENTS * Math.PI * 2;
            const cos = Math.cos(angle);
            const sin = Math.sin(angle);
            const vertexIndex = (ring * ringSize + radial) * 3;
            positions[vertexIndex] = point.x + radius * (
                normal.x * cos + binormal.x * sin
            );
            positions[vertexIndex + 1] = point.y + radius * (
                normal.y * cos + binormal.y * sin
            );
            positions[vertexIndex + 2] = point.z + radius * (
                normal.z * cos + binormal.z * sin
            );
        }
    }
    positions.set(controlPoints[0].toArray(), capStartIndex * 3);
    positions.set(controlPoints.at(-1).toArray(), capEndIndex * 3);

    const indices = [];
    for (let ring = 0; ring < tubularSegments; ring++) {
        for (let radial = 0; radial < RADIAL_SEGMENTS; radial++) {
            const next = (radial + 1) % RADIAL_SEGMENTS;
            const a = ring * ringSize + radial;
            const b = (ring + 1) * ringSize + radial;
            const c = (ring + 1) * ringSize + next;
            const d = ring * ringSize + next;
            indices.push(a, b, d, b, c, d);
        }
    }
    for (let radial = 0; radial < RADIAL_SEGMENTS; radial++) {
        const next = (radial + 1) % RADIAL_SEGMENTS;
        indices.push(capStartIndex, radial, next);
        const lastRing = tubularSegments * ringSize;
        indices.push(capEndIndex, lastRing + next, lastRing + radial);
    }
    // The ring construction above follows the curve frame clockwise. Flip the
    // complete shell so Manifold receives outward-facing triangles; an inward
    // shell represents the complement and would subtract lumen during union.
    for (let index = 0; index < indices.length; index += 3) {
        const swap = indices[index + 1];
        indices[index + 1] = indices[index + 2];
        indices[index + 2] = swap;
    }

    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geometry.setIndex(indices);
    geometry.computeVertexNormals();
    geometry.userData.extendedArtery = pathDefinition.name;
    return geometry;
}

function threeGeometryToManifold(module, sourceGeometry, tolerance = 1e-3) {
    const geometry = sourceGeometry.index
        ? sourceGeometry
        : mergeVertices(sourceGeometry, tolerance);
    const mesh = new module.Mesh({
        numProp: 3,
        vertProperties: new Float32Array(
            geometry.getAttribute('position').array
        ),
        triVerts: new Uint32Array(geometry.index.array),
        tolerance
    });
    mesh.merge();
    return module.Manifold.ofMesh(mesh);
}

function manifoldToThreeGeometry(manifold) {
    const mesh = manifold.getMesh();
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute(
        'position',
        new THREE.BufferAttribute(new Float32Array(mesh.vertProperties), 3)
    );
    geometry.setIndex(new THREE.BufferAttribute(
        new Uint32Array(mesh.triVerts),
        1
    ));
    geometry.computeVertexNormals();
    return geometry;
}

function unionArteryPaths(module, paths, {
    outerWall = false
} = {}) {
    const terminalPath = path =>
        path.terminal === true ||
        path.name.endsWith('deep-femoral') ||
        path.name.endsWith('fibular') ||
        path.name.endsWith('medial-plantar');
    const pathSolids = paths.map(path =>
        threeGeometryToManifold(
            module,
            taperedTubeGeometry(tubeDefinition(path, {
                outerWall,
                capTerminal: outerWall && terminalPath(path)
            })),
            1e-5
        )
    );
    const endpointGroups = new Map();
    for (const path of paths) {
        // Include shared interior control points as well as terminal endpoints.
        // Several anatomical branches (profunda femoris, digital arteries,
        // communicating vessels) originate from the middle of a parent path.
        // A fitted junction sphere makes those CSG unions robust enough for
        // medial-axis extraction without changing ordinary curve points.
        for (const controlPoint of path.points) {
            const key = controlPoint.position.toArray()
                .map(value => value.toFixed(6))
                .join(',');
            const group = endpointGroups.get(key) || [];
            group.push(controlPoint);
            endpointGroups.set(key, group);
        }
    }
    const junctionSolids = [...endpointGroups.values()]
        .filter(group => group.length > 1)
        .map(group => {
            const endpoint = group[0];
            const lumenRadius = Math.max(...group.map(item => item.radius));
            const radius = lumenRadius + (
                outerWall ? wallThickness(lumenRadius) : 0
            );
            return module.Manifold.sphere(radius, RADIAL_SEGMENTS).translate(
                endpoint.position.x,
                endpoint.position.y,
                endpoint.position.z
            );
        });
    const solids = [...pathSolids, ...junctionSolids];
    const result = module.Manifold.union(solids);
    for (const solid of solids) solid.delete();
    return result;
}

function unionArteryPathGroups(module, pathGroups, options) {
    const groupSolids = pathGroups.map(paths =>
        unionArteryPaths(module, paths, options)
    );
    const result = module.Manifold.union(groupSolids);
    for (const solid of groupSolids) solid.delete();
    return result;
}

function createArteryPathGroupSolids(module, pathGroups, options) {
    return pathGroups.map(paths => unionArteryPaths(module, paths, options));
}

function inverseAortaTransform(geometry, transform) {
    geometry.translate(
        -transform.targetCenter[0],
        -transform.targetCenter[1],
        -transform.targetCenter[2]
    );
    geometry.scale(1 / transform.scale, 1 / transform.scale, 1 / transform.scale);
    geometry.rotateX(Math.PI / 2);
    geometry.translate(
        transform.sourceCenter[0],
        transform.sourceCenter[1],
        transform.sourceCenter[2]
    );
    geometry.computeVertexNormals();
    return geometry;
}

function removeZeroAreaTriangles(geometry) {
    const position = geometry.getAttribute('position');
    const sourceIndex = geometry.getIndex();
    const filtered = [];
    const a = new THREE.Vector3();
    const b = new THREE.Vector3();
    const c = new THREE.Vector3();
    const ab = new THREE.Vector3();
    const ac = new THREE.Vector3();
    let removedTriangleCount = 0;
    for (let index = 0; index < sourceIndex.count; index += 3) {
        const ia = sourceIndex.getX(index);
        const ib = sourceIndex.getX(index + 1);
        const ic = sourceIndex.getX(index + 2);
        a.fromBufferAttribute(position, ia);
        b.fromBufferAttribute(position, ib);
        c.fromBufferAttribute(position, ic);
        if (ab.subVectors(b, a).cross(ac.subVectors(c, a)).lengthSq() === 0) {
            removedTriangleCount++;
            continue;
        }
        filtered.push(ia, ib, ic);
    }
    geometry.setIndex(filtered);
    return removedTriangleCount;
}

function writeBinaryStl(geometry, path) {
    const mesh = new THREE.Mesh(geometry);
    mesh.updateMatrixWorld();
    const output = new STLExporter().parse(mesh, { binary: true });
    fs.writeFileSync(
        path,
        Buffer.from(output.buffer, output.byteOffset, output.byteLength)
    );
}

let sourceBytes = fs.readFileSync(SOURCE_PATH);
let sourceHash = sha256(sourceBytes);
if (sourceHash !== BASE_STL_SHA256) {
    sourceBytes = readOriginDevAsset(SOURCE_PATH);
    sourceHash = sha256(sourceBytes);
}
if (sourceHash !== BASE_STL_SHA256) {
    throw new Error(
        `Expected untouched base STL ${BASE_STL_SHA256}, received ${sourceHash}`
    );
}

// Keep this import observable in generated documentation and make accidental
// path changes fail before the expensive CSG work starts.
if (!AORTA_MODEL_URL.endsWith('/res/Aorta_plain.stl')) {
    throw new Error(`Unexpected aorta model URL: ${AORTA_MODEL_URL}`);
}

let collisionBytes = fs.readFileSync(COLLISION_PATH);
let collisionAsset = decodeCollisionAsset(arrayBuffer(collisionBytes));
if (collisionAsset.metadata.source.stlSha256 !== sourceHash) {
    collisionBytes = readOriginDevAsset(COLLISION_PATH);
    collisionAsset = decodeCollisionAsset(arrayBuffer(collisionBytes));
}
if (collisionAsset.metadata.source.stlSha256 !== sourceHash) {
    throw new Error('The collision asset does not describe the base STL');
}

const sourceGeometry = new STLLoader().parse(arrayBuffer(sourceBytes));
const { vessel } = generateVessel(140, 0);
const transform = transformAortaGeometry(sourceGeometry, vessel);
const centerline = loadCenterlineSegments(collisionAsset);
const lowerLimbAttachmentRoots = findLowerLimbAttachmentRoots(centerline);
const headAttachmentRoots = findHeadArteryAttachmentRoots(centerline);
const upperLimbAttachmentRoots = findUpperLimbAttachmentRoots(centerline);
const lowerLimbPaths = createLowerLimbArteryPaths(lowerLimbAttachmentRoots);
const headPaths = createHeadArteryPaths(headAttachmentRoots);
const upperLimbPaths = createUpperLimbArteryPaths(upperLimbAttachmentRoots);
const arteryPaths = [...lowerLimbPaths, ...headPaths, ...upperLimbPaths];
const pairedPathGroups = paths => ['right', 'left'].map(side =>
    paths.filter(path => path.anatomicalSide === side)
);
const arteryPathGroups = [
    ...pairedPathGroups(lowerLimbPaths),
    headPaths,
    ...pairedPathGroups(upperLimbPaths)
];

console.log('Lower-limb attachment roots', lowerLimbAttachmentRoots.map(root => ({
    anatomicalSide: root.anatomicalSide,
    point: root.point.toArray(),
    radius: root.radius,
    segmentId: root.segmentId
})));
console.log('Head attachment roots', headAttachmentRoots.map(roots => ({
    anatomicalSide: roots.anatomicalSide,
    carotid: {
        point: roots.carotid.point.toArray(),
        radius: roots.carotid.radius,
        segmentId: roots.carotid.segmentId
    },
    vertebral: {
        point: roots.vertebral.point.toArray(),
        radius: roots.vertebral.radius,
        segmentId: roots.vertebral.segmentId
    }
})));
console.log('Upper-limb attachment roots', upperLimbAttachmentRoots.map(root => ({
    anatomicalSide: root.anatomicalSide,
    point: root.point.toArray(),
    radius: root.radius,
    segmentId: root.segmentId
})));

const manifoldModule = await ManifoldModule();
manifoldModule.setup();
const extensionOuterSolid = unionArteryPathGroups(manifoldModule, arteryPathGroups, {
    outerWall: true
});
const extensionLumenGroupSolids = createArteryPathGroupSolids(
    manifoldModule,
    arteryPathGroups
);
const extensionLumenSolid = manifoldModule.Manifold.union(
    extensionLumenGroupSolids
);
const extensionWallSolid = extensionOuterSolid.subtract(extensionLumenSolid);
const baseSolid = threeGeometryToManifold(manifoldModule, sourceGeometry);
const baseVolume = baseSolid.volume();
const extensionOuterVolume = extensionOuterSolid.volume();
const extensionLumenVolume = extensionLumenSolid.volume();
const extensionWallVolume = extensionWallSolid.volume();
const wallUnionSolid = manifoldModule.Manifold.union([
    baseSolid,
    extensionOuterSolid
]);
// The source STL is the material of the vessel wall, not a solid cast of its
// lumen. Carving the inner tube both opens the old terminal caps and continues
// the existing hollow lumen through the newly added arterial wall.
// Subtract each anatomical lumen group separately. Keeping the right and left
// limbs independent here avoids numerical loss of a narrow subclavian inlet
// when a single Boolean expression contains the complete full-body tree.
let combinedSolid = wallUnionSolid;
for (const lumenGroupSolid of extensionLumenGroupSolids) {
    const nextSolid = combinedSolid.subtract(lumenGroupSolid);
    if (combinedSolid !== wallUnionSolid) combinedSolid.delete();
    combinedSolid = nextSolid;
}
const combinedVolume = combinedSolid.volume();
baseSolid.delete();
extensionOuterSolid.delete();
extensionLumenSolid.delete();
for (const solid of extensionLumenGroupSolids) solid.delete();
extensionWallSolid.delete();
wallUnionSolid.delete();
const combinedGeometry = manifoldToThreeGeometry(combinedSolid);
const manifoldDiagnostics = {
    status: combinedSolid.status(),
    triangleCount: combinedSolid.numTri(),
    vertexCount: combinedSolid.numVert(),
    genus: combinedSolid.genus(),
    baseVolume,
    extensionOuterVolume,
    extensionLumenVolume,
    extensionWallVolume,
    combinedVolume
};
combinedSolid.delete();
combinedGeometry.computeBoundingBox();
const workingBounds = combinedGeometry.boundingBox.clone();
if (
    workingBounds.max.y < HEAD_ARTERY_SUPERIOR_Y_MM - 8 ||
    workingBounds.min.y > -1325 ||
    workingBounds.min.x > -UPPER_LIMB_LATERAL_EXTENT_MM ||
    workingBounds.max.x < UPPER_LIMB_LATERAL_EXTENT_MM ||
    !arteryPaths.some(path =>
        path.points.some(point =>
            point.position.y <= UPPER_LIMB_HAND_DISTAL_Y_MM
        )
    ) ||
    combinedGeometry.index.count / 3 <=
        sourceGeometry.attributes.position.count / 3 ||
    combinedVolume <= baseVolume + extensionWallVolume * 0.65
) {
    throw new Error('Generated extended-anatomy CSG geometry failed its extent checks');
}

inverseAortaTransform(combinedGeometry, transform);
const removedZeroAreaTriangleCount = removeZeroAreaTriangles(combinedGeometry);
writeBinaryStl(combinedGeometry, TEMP_PATH);

const generatedBytes = fs.readFileSync(TEMP_PATH);
const generatedGeometry = new STLLoader().parse(arrayBuffer(generatedBytes));
const generatedTransform = transformAortaGeometry(generatedGeometry, vessel);
generatedGeometry.computeBoundingBox();
if (
    Math.abs(generatedTransform.scale - transform.scale) > 1e-9 ||
    generatedGeometry.boundingBox.min.y > -1325 ||
    generatedGeometry.boundingBox.max.y < HEAD_ARTERY_SUPERIOR_Y_MM - 8 ||
    generatedGeometry.boundingBox.min.x > -UPPER_LIMB_LATERAL_EXTENT_MM ||
    generatedGeometry.boundingBox.max.x < UPPER_LIMB_LATERAL_EXTENT_MM
) {
    fs.unlinkSync(TEMP_PATH);
    throw new Error('Reloaded STL did not preserve the intended anatomical scale');
}

fs.renameSync(TEMP_PATH, SOURCE_PATH);
console.log(JSON.stringify({
    output: SOURCE_PATH,
    sha256: sha256(generatedBytes),
    byteLength: generatedBytes.byteLength,
    triangleCount: generatedGeometry.attributes.position.count / 3,
    removedZeroAreaTriangleCount,
    lowerLimbAttachmentRoots: lowerLimbAttachmentRoots.map(root => ({
        anatomicalSide: root.anatomicalSide,
        point: root.point.toArray(),
        radius: root.radius
    })),
    headAttachmentRoots: headAttachmentRoots.map(roots => ({
        anatomicalSide: roots.anatomicalSide,
        carotid: {
            point: roots.carotid.point.toArray(),
            radius: roots.carotid.radius
        },
        vertebral: {
            point: roots.vertebral.point.toArray(),
            radius: roots.vertebral.radius
        }
    })),
    upperLimbAttachmentRoots: upperLimbAttachmentRoots.map(root => ({
        anatomicalSide: root.anatomicalSide,
        point: root.point.toArray(),
        radius: root.radius
    })),
    paths: arteryPaths.map(path => ({
        name: path.name,
        vesselNames: path.vesselNames
    })),
    manifold: manifoldDiagnostics,
    bounds: {
        min: generatedGeometry.boundingBox.min.toArray(),
        max: generatedGeometry.boundingBox.max.toArray()
    }
}, null, 2));
