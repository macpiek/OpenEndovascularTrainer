import * as THREE from 'three';
import { inDeformationRegion } from './subclavianDeformation.mjs';

/** Preserve closure provenance and transport surface probes, including curved caps. */
export function deformOutletManifest(source, deformation, output) {
    const report = structuredClone(source);
    const points = [], records = [];
    const add = p => { const index = points.length; points.push(p.toArray()); return index; };
    for (const cap of report.caps) {
        const center = new THREE.Vector3(...cap.center), normal = new THREE.Vector3(...cap.normal);
        const u = new THREE.Vector3(Math.abs(normal.x) < .8 ? 1 : 0,
            Math.abs(normal.x) < .8 ? 0 : 1, 0).cross(normal).normalize();
        const v = normal.clone().cross(u);
        const probes = [];
        for (const fraction of [0, .25, .75, .95]) for (let k = 0; k < (fraction ? 32 : 1); k++) {
            const angle = k / 32 * 2 * Math.PI;
            const p = center.clone().addScaledVector(u, fraction * cap.minRadius * Math.cos(angle))
                .addScaledVector(v, fraction * cap.minRadius * Math.sin(angle));
            probes.push([add(p), add(p.clone().addScaledVector(u, .01)),
                add(p.clone().addScaledVector(v, .01))]);
        }
        records.push({ cap, probes });
    }
    const terminals = report.terminals.map(t => ({
        terminal: t,
        point: add(new THREE.Vector3(...t.point)),
        end: add(new THREE.Vector3(...t.point).add(new THREE.Vector3(...t.normal)))
    }));
    deformation.move(points.filter(inDeformationRegion));
    const direction = (a, b) => new THREE.Vector3(...points[b]).sub(new THREE.Vector3(...points[a]));
    for (const { cap, probes } of records) {
        cap.surfaceProbes = probes.map(([p, u, v]) => ({
            point: points[p], normal: direction(p, u).cross(direction(p, v)).normalize().toArray()
        }));
        cap.center = cap.surfaceProbes[0].point;
        cap.normal = cap.surfaceProbes[0].normal;
    }
    for (const { terminal, point, end } of terminals) {
        terminal.point = points[point];
        terminal.normal = direction(point, end).normalize().toArray();
        terminal.profileFrame = 'original, before subclavian alignment';
    }
    report.deformation = {
        method: 'Smooth transport with constrained cross-sections and local conforming subdivision',
        inputSha256: source.outputSha256,
        inputTriangles: source.triangles,
        outputSha256: output.sha256,
        outputTriangles: output.triangles
    };
    report.outputSha256 = output.sha256;
    report.triangles = output.triangles;
    report.method += ' Subclavian alignment subsequently deforms the shared wall and cap mesh; original/added triangle counts describe the pre-deformation model.';
    return report;
}
