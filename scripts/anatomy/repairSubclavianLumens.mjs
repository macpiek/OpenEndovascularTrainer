import * as THREE from 'three';
import { Brush, Evaluator, ADDITION, SUBTRACTION, HOLLOW_SUBTRACTION } from 'three-bvh-csg';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import { STLExporter } from 'three/examples/jsm/exporters/STLExporter.js';
import { transformAortaGeometry } from '../../src/aortaTransform.js';
import { generateVessel } from '../../src/vesselGeometry.js';
import { taperedTubeGeometry } from './cappedArteryTube.mjs';

export const SUBCLAVIAN_LUMEN_REPAIR = Object.freeze({
    innerRadius: 2.75, outerRadius: 3.65, membraneClearanceRadius: 2.6,
    distalInnerRadius: 2.15, distalOuterRadius: 3.05,
    rightAbsX: [90, 170], leftAbsX: [85, 170]
});

export function transportedShoulderBranches(deformation) {
    return [-1, 1].map(sign => {
        const control = [[sign < 0 ? -154 : 150, 64, -39], [sign * 136, 75, -27], [sign * 120, 88, -12]];
        const curve = new THREE.CatmullRomCurve3(control.map(p => new THREE.Vector3(...p)), false, 'centripetal');
        const firstLength = new THREE.Vector3(...control[0]).distanceTo(new THREE.Vector3(...control[1]));
        const totalLength = firstLength + new THREE.Vector3(...control[1]).distanceTo(new THREE.Vector3(...control[2]));
        const source = curve.getSpacedPoints(24).map(p => p.toArray());
        const points = deformation.move(source.map(p => p.slice()));
        const radii = source.map((_, i) => {
            const distance = i / 24 * totalLength;
            return distance < firstLength ? THREE.MathUtils.lerp(1.02, .88, distance / firstLength) :
                THREE.MathUtils.lerp(.88, .56, (distance - firstLength) / (totalLength - firstLength));
        });
        return { side: sign < 0 ? 'right' : 'left', points, radii };
    });
}

/** The old procedural extension seams can contain internal sheets. Restore a
 * continuous hollow junction and remove residual sheets from its central lumen.
 * The hollow operation also supports the original non-manifold cap overlaps.
 */
export function repairSubclavianLumens(bytes, curves, shoulderBranches = []) {
    const geometry = new STLLoader().parse(bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength));
    const transform = transformAortaGeometry(geometry, generateVessel(140, 0).vessel);
    geometry.deleteAttribute('normal');
    let wall = new Brush(geometry);
    wall.updateMatrixWorld();
    const evaluator = new Evaluator();
    evaluator.attributes = ['position'];
    evaluator.useGroups = false;
    const smooth = t => { t = Math.max(0, Math.min(1, t)); return t * t * (3 - 2 * t); };
    const trunks = curves.map(curve => {
        const [start, end] = SUBCLAVIAN_LUMEN_REPAIR[`${curve.side}AbsX`];
        const points = curve.actual.filter(p => Math.abs(p[0]) >= start && Math.abs(p[0]) <= end);
        return { side: curve.side, points,
            radii: points.map((_, i) => 2.75 - .6 * smooth((i / (points.length - 1) - .7) / .3)) };
    });
    // Union every outer volume before carving any lumen. Interleaving these
    // operations would let a later branch solid plug its parent artery.
    for (const operation of [ADDITION, SUBTRACTION, HOLLOW_SUBTRACTION]) {
        for (const [isTrunk, definitions] of [[true, trunks], [false, shoulderBranches]]) {
            for (const definition of definitions) {
                const points = definition.points.map((p, i) => ({
                    position: new THREE.Vector3(...p),
                    radius: definition.radii[i] + (operation === ADDITION ? (isTrunk ? .9 : .55) :
                        operation === HOLLOW_SUBTRACTION ? (isTrunk ? -.15 : -.1) : 0)
                }));
                if (operation === ADDITION && !isTrunk) {
                    const end = points.at(-1), previous = points.at(-2);
                    points.push({ position: end.position.clone().addScaledVector(
                        end.position.clone().sub(previous.position).normalize(), .6), radius: end.radius });
                } else if (operation !== ADDITION) {
                    const start = points[0];
                    points.unshift({ position: start.position.clone().addScaledVector(
                        start.position.clone().sub(points[1].position).normalize(), isTrunk ? 6 : 3), radius: start.radius });
                    if (isTrunk) {
                        const end = points.at(-1), previous = points.at(-2);
                        points.push({ position: end.position.clone().addScaledVector(
                            end.position.clone().sub(previous.position).normalize(), 6), radius: end.radius });
                    }
                }
                const tubeGeometry = taperedTubeGeometry({ name: definition.side, points });
                tubeGeometry.deleteAttribute('normal');
                const tube = new Brush(tubeGeometry);
                tube.updateMatrixWorld();
                const previous = wall;
                wall = evaluator.evaluate(wall, tube, operation);
                wall.updateMatrixWorld();
                previous.geometry.dispose();
                tubeGeometry.dispose();
            }
        }
    }
    const result = wall.geometry;
    result.translate(...transform.targetCenter.map(v => -v));
    result.scale(1 / transform.scale, 1 / transform.scale, 1 / transform.scale);
    result.rotateX(Math.PI / 2);
    result.translate(...transform.sourceCenter);
    result.computeVertexNormals();
    const data = new STLExporter().parse(new THREE.Mesh(result), { binary: true });
    const output = Buffer.from(data.buffer, data.byteOffset, data.byteLength);
    result.dispose();
    return output;
}
