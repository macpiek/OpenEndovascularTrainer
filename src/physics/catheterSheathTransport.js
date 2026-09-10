import { SHEATH_BOUNDARY_EPSILON } from './sheathBoundary.js';

/** Fixed material chain, driven only inside the straight introducer.
 * Coordinates are measured from the proximal valve, exactly as for wire feed.
 * Exposed positions, velocities, directors and contact histories are untouched.
 */
export function transportCatheterThroughSheath(body, sheath, progress, dt, state = {}, {reset = false} = {}) {
    const h = body.segmentLength, last = body.count - 1, length = last * h;
    if (!(h > 0) || !(dt > 0) || !Number.isFinite(progress) || progress < 0 || progress > length - h)
        throw new RangeError('Catheter feed exceeds its material reservoir');
    const dx = sheath.end.x - sheath.start.x, dy = sheath.end.y - sheath.start.y, dz = sheath.end.z - sheath.start.z;
    const sheathLength = Math.hypot(dx, dy, dz);
    if (!(sheathLength > 0)) throw new RangeError('Catheter requires a finite straight sheath');
    const ax = dx / sheathLength, ay = dy / sheathLength, az = dz / sheathLength;
    const initialize = reset || state.body !== body;
    const delta = initialize ? 0 : progress - state.progress;
    const speed = delta / dt;
    const activeStart = Math.max(0, Math.min(last - 1, Math.ceil((length - progress) / h) - 1));
    body.setActiveRange(activeStart, last);
    const positionValue = body.x instanceof Float32Array ? Math.fround : value => value;
    const firstOutside = Math.max(0, Math.min(body.count, Math.floor((length - progress + sheathLength + SHEATH_BOUNDARY_EPSILON) / h) + 1));
    for (let i = 0; i <= last; i++) {
        const s = progress - (last - i) * h;
        body.materialCoordinate[i] = s;
        const pinned = i < firstOutside;
        if (initialize || pinned) {
            const x = positionValue(sheath.start.x + ax * s), y = positionValue(sheath.start.y + ay * s), z = positionValue(sheath.start.z + az * s);
            if (initialize || body.x[i] !== x || body.y[i] !== y || body.z[i] !== z) body.setNodePosition(i, x, y, z);
            body.velocityX[i] = ax * speed; body.velocityY[i] = ay * speed; body.velocityZ[i] = az * speed;
        }
        if (initialize || Boolean(body.pinned[i]) !== pinned) body.setPinned(i, pinned);
    }
    if (initialize) {
        body.restLength.fill(h);
        for (const key of ['lengthLambda', 'controlEnabled', 'controlLambda', 'wallLambda',
            'wallFrictionLambda', 'wallFrictionLoad', 'wallActive', 'wallInsideClearance',
            'wallCapsuleSampleCount', 'wallProjectionX', 'wallProjectionY', 'wallProjectionZ',
            'toolProjectionX', 'toolProjectionY', 'toolProjectionZ']) body[key].fill(0);
        body.wallBranchId.fill(-1); body.wallFaceIndex.fill(-1); body.wallGap.fill(Infinity);
        body.setActiveRange(0, last);
        body.captureKirchhoffRestConfiguration({captureRestRotation:false});
        body.setActiveRange(activeStart, last);
    }
    const collisionStart = Math.max(activeStart, Math.min(last, firstOutside - 1));
    body.setCollisionRange(collisionStart, last - 1);
    body.setSheathMaterialEndNode(collisionStart);
    if (delta !== 0) body.wake();
    state.body = body; state.progress = progress; state.delta = delta;
    state.firstOutside = firstOutside; state.sheathLength = sheathLength;
    state.lumenStart = activeStart;
    state.lumenOrigin = body.materialCoordinate[activeStart];
    return state;
}
