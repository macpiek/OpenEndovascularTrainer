const MISS = Object.freeze({ skip: false, knownInside: false, lowerBound: null });
const finitePoint = p => p?.length === 3 && Number.isFinite(p[0]) && Number.isFinite(p[1]) && Number.isFinite(p[2]);
const validGrid = d => finitePoint(d.a) && finitePoint(d.b) && Number.isInteger(d.sampleCount) && d.sampleCount >= 1 &&
    d.geometryToken != null && d.gridToken != null;

/** A certificate for the existing affine capsule sample grid, not continuous
 * collision detection. For p(t)=(1-t)a+tb, endpoint displacement bounds every
 * sample displacement. Distance to a fixed surface is 1-Lipschitz; consequently
 * min(old distances)-max(endpoint displacement) bounds all new distances.
 *
 * The caller MUST certify the initial inside sign independently, or explicitly
 * accept its geometry provider's initial classification as physical. BVH only
 * certifies unsigned distance: a branch/SDF sign heuristic is not a proof of
 * mesh interior. geometryToken must change after any surface mutation, and
 * gridToken must identify the same material interval and clipping/sample grid.
 * Lookups retain the original certificate pose, so skipped motions accumulate.
 */
export function createSharedAxisDiscoveryCache({ capacity = 2048, retainOnAbort = false } = {}) {
    if (!Number.isInteger(capacity) || capacity < 1) throw new RangeError('Cache capacity must be a positive integer');
    const entries = new Map();
    const stats = { lookups: 0, clearSkips: 0, insideProofs: 0, certificates: 0, rejectedCertificates: 0, evictions: 0 };
    return {
        stats,
        get size() { return entries.size; },
        clear() { entries.clear(); },
        // The replay owner validates mesh identity before binding these numeric
        // certificates to a fresh BVH object. Never serialize object tokens.
        capture(geometryToken) {
            return structuredClone([...entries].filter(([,d])=>d.geometryToken===geometryToken)
                .map(([key,{geometryToken,...d}])=>[key,d]));
        },
        restore(values,geometryToken) {
            const restored=[];
            for(const [key,d] of values??[]) {
                if(typeof key!=='string'||typeof d.gridToken!=='string'||!validGrid({...d,geometryToken})||
                    !(Number.isFinite(d.minimumDistance)&&d.minimumDistance>0))throw new RangeError('Invalid discovery replay certificate');
                restored.push([key,{...d,a:Array.from(d.a),b:Array.from(d.b),geometryToken,
                    scale:Math.max(1,d.minimumDistance,...d.a.map(Math.abs),...d.b.map(Math.abs))}]);
            }
            entries.clear();for(const [key,d]of restored.slice(-capacity))entries.set(key,d);
        },
        invalidate(key) { entries.delete(key); },
        lookup(key, descriptor) {
            stats.lookups++;
            const old = entries.get(key), d = descriptor;
            if (!old || !validGrid(d) || !Number.isFinite(d.radius) || d.radius < 0 ||
                old.geometryToken !== d.geometryToken || old.gridToken !== d.gridToken || old.sampleCount !== d.sampleCount) return MISS;
            const margin = d.margin ?? .5;
            if (!Number.isFinite(margin) || margin < 0) return MISS;
            const displacement = Math.max(Math.hypot(d.a[0] - old.a[0], d.a[1] - old.a[1], d.a[2] - old.a[2]),
                Math.hypot(d.b[0] - old.b[0], d.b[1] - old.b[1], d.b[2] - old.b[2]));
            // Conservative roundoff guard for endpoint interpolation, vector
            // subtraction/norm and the final lower-bound comparison. It only
            // removes skips; it does not alter a physical contact tolerance.
            const scale = Math.max(old.scale, Math.abs(d.a[0]), Math.abs(d.a[1]), Math.abs(d.a[2]),
                Math.abs(d.b[0]), Math.abs(d.b[1]), Math.abs(d.b[2]), d.radius, margin);
            const lowerBound = old.minimumDistance - displacement - 128 * Number.EPSILON * scale;
            const knownInside = lowerBound > 0, skip = lowerBound > d.radius + margin;
            if (knownInside) stats.insideProofs++;
            if (skip) stats.clearSkips++;
            return { skip, knownInside, lowerBound };
        },
        /** Begin before the exact query, visit its already evaluated samples,
         * commit only on successful query completion; abort in the catch path.
         * A historical winner alone cannot certify the remaining sample sites.
         */
        begin(key, descriptor, { insideCertified = false, distanceError = 0 } = {}) {
            // A rejected Newton trial must not erase the certificate of an
            // earlier pose. It is an immutable geometric fact, independent of
            // whether the current trial succeeds. lookup still checks surface,
            // sample grid and the complete displacement from that old pose.
            // Publish a replacement only after every sample was certified.
            if(!retainOnAbort)entries.delete(key);
            const d = descriptor;
            const valid = validGrid(d) && insideCertified === true && Number.isFinite(distanceError) && distanceError >= 0;
            const a = valid ? Array.from(d.a) : null, b = valid ? Array.from(d.b) : null;
            const sampleCount = valid ? d.sampleCount : 0;
            const seen = new Uint8Array(sampleCount + 1);
            const geometryToken = d.geometryToken, gridToken = d.gridToken;
            let minimumDistance = Infinity, visits = 0, invalid = !valid, finished = false;
            return {
                visit(contact, t) {
                    if (finished || invalid) return;
                    const index = Math.round(t * sampleCount);
                    // Use exact grid values: accepting nearby t could miss a
                    // different sample, invalidating the endpoint-motion proof.
                    if (!Number.isInteger(index) || index < 0 || index > sampleCount || t !== index / sampleCount || seen[index] ||
                        contact?.source !== 'sparse-sdf-bvh' || !Number.isInteger(contact.faceIndex) || contact.faceIndex < 0 ||
                        !Number.isFinite(contact.signedDistance) || contact.signedDistance <= distanceError || contact.inside !== true) {
                        invalid = true; return;
                    }
                    seen[index] = 1; visits++;
                    minimumDistance = Math.min(minimumDistance, contact.signedDistance - distanceError);
                },
                commit() {
                    if (finished) return false;
                    finished = true;
                    if (invalid || visits !== sampleCount + 1) { stats.rejectedCertificates++; return false; }
                    if (entries.size >= capacity && !entries.has(key)) { entries.delete(entries.keys().next().value); stats.evictions++; }
                    entries.set(key, { a, b, geometryToken, gridToken, sampleCount, minimumDistance,
                        scale: Math.max(1, minimumDistance, ...a.map(Math.abs), ...b.map(Math.abs)) });
                    stats.certificates++; return true;
                },
                abort() { if (!finished) stats.rejectedCertificates++; finished = true; if(!retainOnAbort)entries.delete(key); }
            };
        }
    };
}
