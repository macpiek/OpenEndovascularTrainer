import {BufferAttribute, ShapeUtils, Vector2, Vector3} from 'three';

// Close only genuinely open, planar boundary loops, using their existing STL
// vertices. This supplies ray entry/exit coverage without circular billboards
// or changing the anatomical contour. Already closed outlets add no faces.
export function closeLumenBoundaries(geometry, vertexIds) {
    const position = geometry.getAttribute('position'), count = position.count;
    const ids = vertexIds ?? new Uint32Array(count), seen = vertexIds ? null : new Map();
    if (seen) for (let i = 0; i < count; i++) {
        const key = [position.getX(i), position.getY(i), position.getZ(i)].map(x => Math.abs(x) < 1e-7 ? 0 : x).join(',');
        if (!seen.has(key)) seen.set(key, i);
        ids[i] = seen.get(key);
    }
    const keys = new Float64Array(count);
    for (let i = 0; i < count; i += 3) for (let k = 0; k < 3; k++) {
        const a = ids[i + k], b = ids[i + (k + 1) % 3];
        keys[i + k] = (Math.min(a, b) * count + Math.max(a, b)) * 2 + (a > b ? 1 : 0);
    }
    keys.sort();
    const next = new Map(), ambiguous = new Set();
    let boundaryEdges = 0;
    for (let i = 0; i < count;) {
        const key = Math.floor(keys[i] / 2), first = i;
        while (i < count && Math.floor(keys[i] / 2) === key) i++;
        if (i - first !== 1) continue;
        const lo = Math.floor(key / count), hi = key % count;
        // Reverse the open edge so closure winding matches the inward wall.
        const a = keys[first] % 2 ? lo : hi, b = keys[first] % 2 ? hi : lo;
        if (next.has(a)) ambiguous.add(a);
        next.set(a, b); boundaryEdges++;
    }
    const visited = new Set(), triangles = [], normal = new Vector3(), a = new Vector3(), b = new Vector3();
    let loops = 0, skippedLoops = 0;
    for (const first of next.keys()) {
        if (visited.has(first)) continue;
        const loop = []; let vertex = first;
        while (!visited.has(vertex) && next.has(vertex) && !ambiguous.has(vertex)) {
            visited.add(vertex); loop.push(vertex); vertex = next.get(vertex);
        }
        if (vertex !== first || loop.length < 3) { skippedLoops++; continue; }
        const points = loop.map(i => new Vector3().fromBufferAttribute(position, i));
        const origin = points[0]; normal.set(0, 0, 0);
        for (let i = 1; i < points.length - 1; i++) normal.add(a.subVectors(points[i], origin).cross(b.subVectors(points[i + 1], origin)));
        if (normal.lengthSq() < 1e-16) { skippedLoops++; continue; }
        normal.normalize();
        const extent = Math.max(...points.map(p => p.distanceTo(origin)));
        if (points.some(p => Math.abs(a.subVectors(p, origin).dot(normal)) > Math.max(1e-4, extent * .002))) {
            skippedLoops++; continue;
        }
        const u = points[1].clone().sub(origin).normalize(), v = normal.clone().cross(u);
        const polygon = points.map(p => {a.subVectors(p, origin); return new Vector2(a.dot(u), a.dot(v));});
        for (const face of ShapeUtils.triangulateShape(polygon, [])) {
            const tri = face.map(i => loop[i]);
            a.subVectors(points[face[1]], points[face[0]]); b.subVectors(points[face[2]], points[face[0]]);
            if (a.cross(b).dot(normal) < 0) [tri[1], tri[2]] = [tri[2], tri[1]];
            triangles.push(...tri);
        }
        loops++;
    }
    if (triangles.length) for (const [name, attribute] of Object.entries(geometry.attributes)) {
        const array = new attribute.array.constructor((count + triangles.length) * attribute.itemSize);
        array.set(attribute.array);
        triangles.forEach((source, i) => array.set(attribute.array.subarray(source * attribute.itemSize, (source + 1) * attribute.itemSize),
            (count + i) * attribute.itemSize));
        geometry.setAttribute(name, new BufferAttribute(array, attribute.itemSize, attribute.normalized));
    }
    geometry.computeBoundingSphere();
    return {triangles: triangles.length / 3, loops, boundaryEdges, skippedLoops};
}
