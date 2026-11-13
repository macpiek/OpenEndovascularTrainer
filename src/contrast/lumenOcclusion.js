import {DoubleSide, Vector3} from 'three';

// Wall classification only needs to know whether a ray is blocked. A nearest
// surface hit also computes normals/barycentrics and visits more triangles.
// Keep Three's exact triangle intersection and inclusive near/far distances.
export function createLumenOcclusionTest(tree) {
    if (!tree) return () => false;
    if (typeof tree.shapecast !== 'function')
        return (ray, near, far) => !!tree.raycastFirst(ray, DoubleSide, near, far);
    const point = new Vector3();
    let ray, near, far, ix, iy, iz, ox, oy, oz;
    const intersectsBounds = box => {
        let tmin = ((ix >= 0 ? box.min.x : box.max.x) - ox) * ix;
        let tmax = ((ix >= 0 ? box.max.x : box.min.x) - ox) * ix;
        const tymin = ((iy >= 0 ? box.min.y : box.max.y) - oy) * iy;
        const tymax = ((iy >= 0 ? box.max.y : box.min.y) - oy) * iy;
        if (tmin > tymax || tymin > tmax) return false;
        if (tymin > tmin || Number.isNaN(tmin)) tmin = tymin;
        if (tymax < tmax || Number.isNaN(tmax)) tmax = tymax;
        const tzmin = ((iz >= 0 ? box.min.z : box.max.z) - oz) * iz;
        const tzmax = ((iz >= 0 ? box.max.z : box.min.z) - oz) * iz;
        if (tmin > tzmax || tzmin > tmax) return false;
        if (tzmin > tmin || Number.isNaN(tmin)) tmin = tzmin;
        if (tzmax < tmax || Number.isNaN(tmax)) tmax = tzmax;
        return tmin <= far && tmax >= near;
    };
    const callbacks = {
        intersectsBounds,
        intersectsTriangle(triangle) {
            if (!ray.intersectTriangle(triangle.a, triangle.b, triangle.c, false, point)) return false;
            const distance = ray.origin.distanceTo(point);
            return distance >= near && distance <= far;
        }
    };
    return (input, min = 0, max = Infinity) => {
        ray = input; near = min; far = max;
        ix = 1 / ray.direction.x; iy = 1 / ray.direction.y; iz = 1 / ray.direction.z;
        ox = ray.origin.x; oy = ray.origin.y; oz = ray.origin.z;
        return tree.shapecast(callbacks);
    };
}
