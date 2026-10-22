// Audit retained short native subtrees against every other lumen, independently
// of the local connected-neighbourhood cleanup used by the flow network.
import fs from 'node:fs';
import * as THREE from 'three';
import {decodeCollisionAsset} from '../src/physics/collision/collisionAssetFormat.js';
import {ContrastFlowNetwork} from '../src/contrast/flowNetwork.js';
function system(name='Aorta_infrarenal_aneurysm') {
    const bytes=fs.readFileSync(new URL(`../res/${name}.collision.bin`,import.meta.url));
    const asset=decodeCollisionAsset(bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength));
    const data=asset.arrays.centerlineSegments,links=asset.arrays.centerlineEdges;
    const segments=Array.from({length:links.length/2},(_,id)=>({
        id,nodeStartId:links[id*2],nodeEndId:links[id*2+1],
        start:new THREE.Vector3().fromArray(data,id*9),end:new THREE.Vector3().fromArray(data,id*9+3),
        radiusStart:data[id*9+6],radiusEnd:data[id*9+7],safeRadius:data[id*9+8]
    }));
    for(const r of asset.metadata.renal?.segments??[])Object.assign(segments[r.index],{
        renalSide:r.side,renalSector:r.sector,renalHilar:r.hilar,renalLateral:r.lateral
    });
    return new ContrastFlowNetwork(segments);
}

function audit(net) {
    const rows = [];
    for (const root of net.edges) {
        if (root.transportExcluded || root.renalSide || root.parentEdgeIndex < 0 ||
            net.edges[root.parentEdgeIndex].childEdgeIndices.length < 2) continue;
        const queue = [root], path = [], terminals = [];
        let length = 0;
        while (queue.length && path.length < 200 && length < 200) {
            const edge = queue.pop();
            path.push(edge);
            length += edge.length;
            if (!edge.childEdgeIndices.length) terminals.push(edge);
            queue.push(...edge.childEdgeIndices.map(i => net.edges[i]));
        }
        if (queue.length || terminals.length > 3 || !terminals.length ||
            path.some(edge => edge.renalSide)) continue;
        const ownIndices = new Set(path.map(edge => edge.index));
        const others = net.edges.filter(edge => !edge.transportExcluded &&
            !edge.renalSide && !ownIndices.has(edge.index));
        let maximumDistanceRatio = 0;
        const owners = new Set();
        for (const edge of path) {
            for (const t of [0, .5, 1]) {
                const point = edge.start.clone().lerp(edge.end, t);
                let best = Infinity, owner = -1;
                for (const other of others) {
                    if (edge.axis.dot(other.axis) < .7) continue;
                    const axial = point.clone().sub(other.start).dot(other.axis);
                    if (axial < 0 || axial > other.length) continue;
                    const radius = THREE.MathUtils.lerp(other.rawRadiusStart,
                        other.rawRadiusEnd, axial / other.length);
                    const ratio = point.distanceTo(other.start.clone()
                        .addScaledVector(other.axis, axial)) / radius;
                    if (ratio < best) { best = ratio; owner = other.index; }
                }
                maximumDistanceRatio = Math.max(maximumDistanceRatio, best);
                owners.add(owner);
                if (maximumDistanceRatio > 1) break;
            }
            if (maximumDistanceRatio > 1) break;
        }
        if (maximumDistanceRatio < 1) rows.push({
            root: root.index, lengthMm: length, outlets: terminals.length,
            volumeMm3: path.reduce((sum, edge) => sum + edge.totalVolume, 0),
            maxRatio: maximumDistanceRatio, owners: [...owners],
            position: root.start.toArray()
        });
    }
    return rows;
}

for (const name of ['Aorta_plain', 'Aorta_plain_renal', 'Aorta_infrarenal_aneurysm', 'Aorta_infrarenal_aneurysm_renal']) {
    const net = system(name), candidates = audit(net);
    const duplicates = candidates.filter(row => row.maxRatio < .5);
    console.log(JSON.stringify({anatomy:name, edges:net.edges.length,
        removedTerminalStrands:net.containedTerminalSpurDiagnostics.roots.length,
        remainingContainedDuplicates:duplicates,
        nearLumenBoundary:candidates.filter(row => row.maxRatio >= .5)}, null, 2));
    if (duplicates.length) process.exitCode = 1;
}
