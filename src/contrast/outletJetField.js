import {Box3, Vector3, Vector4} from 'three';

// The local solver carries the iodine which has not entrained into the bulk
// network yet. Reconstruct that SAME mass as a compact set of smooth 3-D
// distributions, rather than painting a whole transport cell at the outlet.
// No billboard, extra texture, or density subtraction is involved: the caller
// integrates these kernels only between the anatomical entry and exit walls.
export const MAX_OUTLET_JET_KERNELS = 24;
const NORMALIZATION = Math.pow(2 * Math.PI, 1.5);
const AGE_BIN_SECONDS = .012;
const SUPPORT_SIGMAS = 6;
const component = (p, i) => p?.getComponent ? p.getComponent(i) : p?.[i] ?? [p?.x, p?.y, p?.z][i] ?? 0;
const length = (x, y, z) => Math.hypot(x, y, z);

// Positive-tail form avoids subtracting two values rounded to erf(x) == 1.
function erfcPositive(x) {
    const t = 1 / (1 + .3275911 * Math.max(0, x));
    return (((((1.061405429 * t - 1.453152027) * t) + 1.421413741) * t - .284496736) * t + .254829592) * t * Math.exp(-x * x);
}
function erfDifference(a, b) {
    if (a >= 0) return erfcPositive(a) - erfcPositive(b);
    if (b <= 0) return erfcPositive(-b) - erfcPositive(-a);
    return 2 - erfcPositive(-a) - erfcPositive(b);
}
function eligible(kernel, a, b) {
    return kernel.pathA === a || kernel.pathA === b || kernel.pathB === a || kernel.pathB === b;
}

/** Exact ray integral of one normalized Gaussian, bounded to six sigmas. */
export function integrateOutletJetKernel(kernel, entry, ray, chord) {
    if (!(chord > 0)) return 0;
    const dx = component(entry, 0) - kernel.x, dy = component(entry, 1) - kernel.y, dz = component(entry, 2) - kernel.z;
    const rx = component(ray, 0), ry = component(ray, 1), rz = component(ray, 2);
    const za = dx * kernel.ax + dy * kernel.ay + dz * kernel.az;
    const zr = rx * kernel.ax + ry * kernel.ay + rz * kernel.az;
    const r = kernel.inverseRadialVariance, d = kernel.inverseAxialVariance - r;
    const A = r * (rx * rx + ry * ry + rz * rz) + d * zr * zr;
    const B = r * (dx * rx + dy * ry + dz * rz) + d * za * zr;
    const C = r * (dx * dx + dy * dy + dz * dz) + d * za * za;
    const t = -B / A, transverse = Math.max(0, C - B * B / A);
    if (transverse >= SUPPORT_SIGMAS * SUPPORT_SIGMAS) return 0;
    const span = Math.sqrt((SUPPORT_SIGMAS * SUPPORT_SIGMAS - transverse) / A);
    const lo = Math.max(0, t - span), hi = Math.min(chord, t + span);
    if (hi <= lo) return 0;
    const scale = Math.sqrt(A * .5);
    return Math.max(0, kernel.amplitude * Math.sqrt(Math.PI / (2 * A)) * Math.exp(-.5 * transverse) *
        erfDifference((lo - t) * scale, (hi - t) * scale));
}

function accumulator(pathA, pathB) {
    return {pathA, pathB, mass: 0, mx: 0, my: 0, mz: 0, ax: 0, ay: 0, az: 0,
        xx: 0, yy: 0, zz: 0, xy: 0, xz: 0, yz: 0, radius: 0, ageMoment: 0, travelMoment: 0};
}
function add(a, b) {
    for (const key of ['mass', 'mx', 'my', 'mz', 'ax', 'ay', 'az', 'xx', 'yy', 'zz', 'xy', 'xz', 'yz', 'radius', 'ageMoment', 'travelMoment']) a[key] += b[key];
}
function samePaths(a, b) {return a.pathA === b.pathA && a.pathB === b.pathB;}
const visibleEdge = edge => edge && !edge.transportExcluded && !edge.renderExcluded && !edge.lumenMappingExcluded;

export function createOutletJetField(network, territories) {
    const center = Array.from({length: MAX_OUTLET_JET_KERNELS}, () => new Vector4());
    const axis = Array.from({length: MAX_OUTLET_JET_KERNELS}, () => new Vector4());
    const shape = Array.from({length: MAX_OUTLET_JET_KERNELS}, () => new Vector4());
    const bounds = new Box3();
    const uniforms = {outletJetCount: {value: 0}, outletJetCenter: {value: center}, outletJetAxis: {value: axis}, outletJetShape: {value: shape},
        outletJetMin: {value: bounds.min}, outletJetMax: {value: bounds.max}};
    const offsets = new Uint32Array(network.edges.length);
    let cellCount = 0;
    for (const edge of network.edges) {offsets[edge.index] = cellCount; cellCount += edge.cellCount;}
    const fallbackMassByCell = new Float64Array(cellCount), location = {}, point = new Vector3();
    const kernels = [];
    let totalMassMg = 0, representedMassMg = 0;
    function update(solver, stock) {
        kernels.length = 0; bounds.makeEmpty(); fallbackMassByCell.fill(0); totalMassMg = representedMassMg = 0;
        uniforms.outletJetCount.value = 0;
        if (!solver?.count || !(stock > 0)) return;
        const groups = new Map(), pathGroups = new Map();
        for (let i = 0; i < solver.count; i++) {
            const mass = solver.iodineMassMg[i];
            if (!(mass > 0)) continue;
            totalMassMg += mass;
            const source = solver.sourceEdgeIndex[i];
            const mapped = solver.resolveSourceHandoffLocation?.(i, location);
            const edgeIndex = mapped?.edgeIndex >= 0 ? mapped.edgeIndex : source;
            const edge = network.edges[edgeIndex];
            const mappedPath = visibleEdge(edge) ? territories[edgeIndex] ?? -1 : -1;
            // Exclusions can change after the optical territories were built.
            // Keep their mass in the ordinary masked-cell path, rather than
            // resurrecting a hidden lumen through the independent jet field.
            // A parcel already in an eligible continuation may still appear
            // there, but must not illuminate its now-excluded source path.
            const sourcePath = visibleEdge(network.edges[source]) ? territories[source] ?? mappedPath : mappedPath;
            // Production devices have few outlets. If a custom source spans
            // more than 24 disconnected territories, preserve those excess
            // masses in their own bulk cells, never merge unrelated vessels.
            const pathA = Math.min(sourcePath, mappedPath), pathB = Math.max(sourcePath, mappedPath);
            const pathKey = `${pathA}:${pathB}`;
            if (pathA < 0 || (!pathGroups.has(pathKey) && pathGroups.size >= MAX_OUTLET_JET_KERNELS)) {
                if (edge) fallbackMassByCell[offsets[edge.index] + Math.max(0, Math.min(edge.cellCount - 1, mapped?.cellIndex ?? solver.sourceCellIndex[i]))] += mass;
                continue;
            }
            pathGroups.set(pathKey, true);
            const x = solver.positionX[i], y = solver.positionY[i], z = solver.positionZ[i];
            let ax = solver.jetVelocityX?.[i] || 0, ay = solver.jetVelocityY?.[i] || 0, az = solver.jetVelocityZ?.[i] || 0;
            let speed = length(ax, ay, az);
            if (speed < 1e-5) {
                ax = solver.outletDirectionX?.[i] ?? edge?.axis?.x ?? 0;
                ay = solver.outletDirectionY?.[i] ?? edge?.axis?.y ?? 1;
                az = solver.outletDirectionZ?.[i] ?? edge?.axis?.z ?? 0;
                speed = length(ax, ay, az);
            }
            if (speed < 1e-5) {ax = 0; ay = 1; az = 0; speed = 1;}
            ax /= speed; ay /= speed; az /= speed;
            const radius = edge ? Math.max(edge.radiusStart, edge.radiusEnd) : 2;
            const outletRadius = Math.max(.05, solver.outletRadiusMm?.[i] || .35);
            const resolvedOutletRadius = Math.max(outletRadius, solver.resolvedOutletRadiusMm?.[i] || outletRadius);
            const travel = Math.max(0, solver.travelDistanceMm?.[i] || 0);
            const progress = Math.min(1, travel / (2 * outletRadius));
            const entrainment = progress * progress * (3 - 2 * progress);
            // The local solver resolves the jet after the sub-millimetre
            // nozzle core, so it caps that very high nozzle velocity. Its
            // effective area must increase by the same Q = v A relation.
            // For a Gaussian profile A = 2πσ². Retain the actual outlet width
            // at the opening, then entrain over one outlet diameter rather
            // than concentrating all the injected volume into a slow needle.
            const nozzleSigma = (outletRadius + (resolvedOutletRadius - outletRadius) * entrainment) / Math.SQRT2;
            // A resolved, entraining jet broadens with distance. Intrinsic
            // parcel extent also covers sub-frame emission spacing; weighted
            // covariance below adds the actual spread of the moving parcels.
            const radial = Math.max(.12, Math.min(radius * .4, Math.hypot(nozzleSigma, travel * .095)));
            const axial = Math.max(.6, radial * 1.5);
            const rr = radial * radial, delta = axial * axial - rr;
            // Source packets mix within ~0.12 s. Keep a bounded terminal bin
            // for custom/legacy packets which may survive much longer.
            const age = Math.min(11, Math.max(0, solver.ageSeconds?.[i] || 0) / AGE_BIN_SECONDS);
            const lower = Math.floor(age), fraction = age - lower;
            // Linear assignment makes a parcel cross age-bin boundaries
            // continuously, rather than switching whole Gaussians per frame.
            for (const [bin, weight] of [[lower, 1 - fraction], [lower + 1, fraction]]) {
                if (weight <= 0) continue;
                const key = `${pathKey}:${source}:${bin}`, m = mass * weight;
                let g = groups.get(key);
                if (!g) {g = accumulator(pathA, pathB); groups.set(key, g);}
                g.mass += m; g.mx += m * x; g.my += m * y; g.mz += m * z;
                g.ax += m * ax; g.ay += m * ay; g.az += m * az;
                g.xx += m * (x * x + rr + delta * ax * ax); g.yy += m * (y * y + rr + delta * ay * ay); g.zz += m * (z * z + rr + delta * az * az);
                g.xy += m * (x * y + delta * ax * ay); g.xz += m * (x * z + delta * ax * az); g.yz += m * (y * z + delta * ay * az);
                g.radius += m * radius;
                g.ageMoment += m * Math.max(0, solver.ageSeconds?.[i] || 0);
                g.travelMoment += m * travel;
            }
        }
        const list = [...groups.values()];
        while (list.length > MAX_OUTLET_JET_KERNELS) {
            let bestA = -1, bestB = -1, distance = Infinity;
            for (let a = 0; a < list.length; a++) for (let b = a + 1; b < list.length; b++) {
                const ga = list[a], gb = list[b]; if (!samePaths(ga, gb)) continue;
                const d = (ga.mx / ga.mass - gb.mx / gb.mass) ** 2 + (ga.my / ga.mass - gb.my / gb.mass) ** 2 + (ga.mz / ga.mass - gb.mz / gb.mass) ** 2;
                if (d < distance) {distance = d; bestA = a; bestB = b;}
            }
            // At most 24 path groups were admitted, so a compatible pair
            // always exists. Preserve its exact mass, centroid and moments.
            add(list[bestA], list[bestB]); list.splice(bestB, 1);
        }
        for (const g of list) {
            const x = g.mx / g.mass, y = g.my / g.mass, z = g.mz / g.mass;
            const norm = length(g.ax, g.ay, g.az);
            const ax = norm > 1e-9 ? g.ax / norm : 0, ay = norm > 1e-9 ? g.ay / norm : 1, az = norm > 1e-9 ? g.az / norm : 0;
            const xx = g.xx / g.mass - x * x, yy = g.yy / g.mass - y * y, zz = g.zz / g.mass - z * z;
            const xy = g.xy / g.mass - x * y, xz = g.xz / g.mass - x * z, yz = g.yz / g.mass - y * z;
            const axialVariance = Math.max(.01, ax * ax * xx + ay * ay * yy + az * az * zz + 2 * (ax * ay * xy + ax * az * xz + ay * az * yz));
            const radialVariance = Math.max(.01, (xx + yy + zz - axialVariance) * .5);
            const volume = g.mass / stock;
            const kernel = {x, y, z, ax, ay, az, massMg: g.mass, volumeMm3: volume, pathA: g.pathA, pathB: g.pathB,
                ageSeconds: g.ageMoment / g.mass, travelDistanceMm: g.travelMoment / g.mass,
                inverseRadialVariance: 1 / radialVariance, inverseAxialVariance: 1 / axialVariance,
                amplitude: volume / (NORMALIZATION * radialVariance * Math.sqrt(axialVariance))};
            const i = kernels.length; kernels.push(kernel); representedMassMg += g.mass;
            center[i].set(x, y, z, kernel.amplitude); axis[i].set(ax, ay, az, kernel.inverseAxialVariance);
            const support = SUPPORT_SIGMAS * Math.sqrt(Math.max(radialVariance, axialVariance));
            shape[i].set(kernel.inverseRadialVariance, g.pathA, g.pathB, support * support);
            for (const sign of [-1, 1]) bounds.expandByPoint(point.set(x + sign * support, y + sign * support, z + sign * support));
        }
        uniforms.outletJetCount.value = kernels.length;
    }
    return {uniforms, bounds, kernels, fallbackMassByCell, offsets, update,
        get count() {return kernels.length;}, get active() {return kernels.length > 0;},
        get massMg() {return representedMassMg;}, get totalMassMg() {return totalMassMg;},
        sample(p, a, b = a) {
            let sum = 0;
            for (const k of kernels) {
                if (!eligible(k, a, b)) continue;
                const dx = component(p, 0) - k.x, dy = component(p, 1) - k.y, dz = component(p, 2) - k.z, axial = dx * k.ax + dy * k.ay + dz * k.az;
                const q = k.inverseRadialVariance * (dx * dx + dy * dy + dz * dz) + (k.inverseAxialVariance - k.inverseRadialVariance) * axial * axial;
                if (q < SUPPORT_SIGMAS * SUPPORT_SIGMAS) sum += k.amplitude * Math.exp(-.5 * Math.max(0, q));
            }
            return sum;
        },
        integrate(entry, ray, chord, a, b = a) {
            let sum = 0; for (const k of kernels) if (eligible(k, a, b)) sum += integrateOutletJetKernel(k, entry, ray, chord); return sum;
        }};
}

export const outletJetGLSL = /* glsl */`
uniform int outletJetCount;
uniform vec4 outletJetCenter[${MAX_OUTLET_JET_KERNELS}];
uniform vec4 outletJetAxis[${MAX_OUTLET_JET_KERNELS}];
uniform vec4 outletJetShape[${MAX_OUTLET_JET_KERNELS}];
uniform vec3 outletJetMin;
uniform vec3 outletJetMax;
float outletJetErfc(float x) {
    float t = 1. / (1. + .3275911 * max(0., x));
    return (((((1.061405429 * t - 1.453152027) * t) + 1.421413741) * t - .284496736) * t + .254829592) * t * exp(-x*x);
}
float outletJetErfDifference(float a, float b) {
    if (a >= 0.) return outletJetErfc(a) - outletJetErfc(b);
    if (b <= 0.) return outletJetErfc(-b) - outletJetErfc(-a);
    return 2. - outletJetErfc(-a) - outletJetErfc(b);
}
float outletJetIntegral(vec3 entry, vec3 ray, float chord, int entryPath, int exitPath) {
    if (outletJetCount <= 0) return 0.;
    // Almost all active bulk blood lies outside the small outlet region.
    // Reject it before entering the bounded kernel loop or evaluating erf.
    float lower = 0., upper = chord;
    for (int component = 0; component < 3; component++) {
        if (abs(ray[component]) < 1e-8) {
            if (entry[component] < outletJetMin[component] || entry[component] > outletJetMax[component]) return 0.;
        } else {
            float a = (outletJetMin[component] - entry[component]) / ray[component];
            float b = (outletJetMax[component] - entry[component]) / ray[component];
            lower = max(lower,min(a,b)); upper = min(upper,max(a,b));
            if (upper <= lower) return 0.;
        }
    }
    float sum = 0.;
    // Keep this loop dynamic. Unrolling the Gaussian body 24 times beside
    // the full deformation/BVH shader can make driver compilation expensive.
    // The uniform and the loop are both bounded by the same fixed capacity.
    int kernelCount = min(outletJetCount, ${MAX_OUTLET_JET_KERNELS});
    for (int i = 0; i < kernelCount; i++) {
        vec4 shape = outletJetShape[i];
        int a = int(shape.y + .5), b = int(shape.z + .5);
        if (a != entryPath && a != exitPath && b != entryPath && b != exitPath) continue;
        vec3 offset = entry - outletJetCenter[i].xyz;
        float nearest = clamp(-dot(offset,ray), 0., chord);
        vec3 separation = offset + ray * nearest;
        if (dot(separation,separation) > shape.w) continue;
        vec4 axis = outletJetAxis[i];
        float za = dot(offset,axis.xyz), zr = dot(ray,axis.xyz), d = axis.w - shape.x;
        float A = shape.x + d*zr*zr;
        float B = shape.x*dot(offset,ray) + d*za*zr;
        float C = shape.x*dot(offset,offset) + d*za*za;
        float t = -B/A, transverse = max(0.,C-B*B/A);
        if (transverse >= 36.) continue;
        float span = sqrt((36.-transverse)/A), lo=max(0.,t-span), hi=min(chord,t+span);
        if (hi <= lo) continue;
        float scale = sqrt(A*.5);
        sum += max(0.,outletJetCenter[i].w*sqrt(1.5707963267948966/A)*exp(-.5*transverse)*outletJetErfDifference((lo-t)*scale,(hi-t)*scale));
    }
    return sum;
}
`;
