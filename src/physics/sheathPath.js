// Arc-length path shared by device feed, the mechanical lumen and imaging.
// Plain objects/arrays deliberately survive structuredClone and worker RPC.
const asPoint = p => Array.isArray(p) ? {x:p[0],y:p[1],z:p[2]} : p;
export function sheathLength(sheath) {
    const a=asPoint(sheath.start),b=asPoint(sheath.end);
    return sheath.path?.at(-1)?.s ?? Math.hypot(
        b.x-a.x,b.y-a.y,b.z-a.z);
}

export function sampleSheathPath(sheath, distance, out = {}) {
    const path = sheath.path;
    let a = asPoint(sheath.start), b = asPoint(sheath.end), start = 0, length = sheathLength(sheath);
    if (path?.length >= 2) {
        let low = 0, high = path.length - 1;
        while (low + 1 < high) {
            const middle = (low + high) >>> 1;
            if (path[middle].s <= distance) low = middle;
            else high = middle;
        }
        a = path[low]; b = path[low + 1]; start = a.s; length = b.s - a.s;
    }
    const t = (distance - start) / length;
    out.x = a.x + (b.x - a.x) * t;
    out.y = a.y + (b.y - a.y) * t;
    out.z = a.z + (b.z - a.z) * t;
    const chord = Math.hypot(b.x - a.x, b.y - a.y, b.z - a.z);
    out.tx = (b.x - a.x) / chord;
    out.ty = (b.y - a.y) / chord;
    out.tz = (b.z - a.z) / chord;
    return out;
}

export function closestSheathAxis(sheath, point) {
    const path = sheath.path ?? [{...asPoint(sheath.start), s:0}, {...asPoint(sheath.end), s:sheathLength(sheath)}];
    let best = null, bestDistanceSquared = Infinity;
    for (let i = 0; i < path.length - 1; i++) {
        const a = path[i], b = path[i + 1];
        const dx=b.x-a.x,dy=b.y-a.y,dz=b.z-a.z,length=Math.hypot(dx,dy,dz);
        const ax=dx/length,ay=dy/length,az=dz/length;
        const along=(point[0]-a.x)*ax+(point[1]-a.y)*ay+(point[2]-a.z)*az;
        // Open ends also support the external loading hub.
        const d=Math.max(i===0?-Infinity:0,Math.min(i===path.length-2?Infinity:length,along));
        const rx=point[0]-a.x-d*ax,ry=point[1]-a.y-d*ay,rz=point[2]-a.z-d*az;
        const squared=rx*rx+ry*ry+rz*rz;
        if (squared < bestDistanceSquared) {
            bestDistanceSquared=squared;
            best={ax,ay,az,rx,ry,rz,interior:d===along,coordinate:a.s+d/length*(b.s-a.s)};
        }
    }
    return {axis:best.interior?[best.ax,best.ay,best.az]:[0,0,0],
        radial:[best.rx,best.ry,best.rz],rho:Math.sqrt(bestDistanceSquared),coordinate:best.coordinate};
}
