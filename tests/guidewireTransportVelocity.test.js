import assert from 'node:assert/strict';
import {
    DEFAULT_TOOL_PROFILES,
    EndovascularPhysicsWorld
} from '../src/physics/endovascularPhysicsWorld.js';
import { ElasticRod } from '../src/physics/elasticRod.js';
import { GuidewireSolver } from '../src/physics/guidewireSolver.js';

const FIXED_DT = 1 / 120;
const OPERATOR_FEED_MM_PER_SECOND = 44;

assert.equal(
    DEFAULT_TOOL_PROFILES.guidewire.maxSpeed,
    Infinity,
    'guidewire material motion must not share the operator feed limit'
);

const unchangedCatheterWorld = new EndovascularPhysicsWorld();
const unchangedCatheter = unchangedCatheterWorld.createRod(
    'unchanged-catheter-contact-path',
    3,
    2,
    DEFAULT_TOOL_PROFILES.catheter
);
assert.equal(
    unchangedCatheter.wallProjectionVelocityRetention,
    1,
    'the guidewire contact filter must not change catheter velocity reconstruction'
);
assert.equal(
    unchangedCatheter.sweptContactPreserveTangentialMotion,
    false,
    'the guidewire swept-slide path must not change catheter collision physics'
);

const transportRod = new ElasticRod(21, 5);
const transportSolver = new GuidewireSolver({
    rod: transportRod,
    segmentLength: 5,
    guidewireLength: 100,
    sheath: {
        start: { x: 0, y: 0, z: 0 },
        end: { x: 20, y: 0, z: 0 },
        radius: 1
    },
    advanceRate: OPERATOR_FEED_MM_PER_SECOND,
    minInsert: 0,
    maxInsert: 100
});
transportSolver.initialize();
const transportDelta = transportSolver.advance(1, FIXED_DT, null, {
    routeAssist: false,
    boundaryDriven: true
});
const transportStats = transportSolver.getPerformanceStats();
assert.ok(
    Math.abs(transportDelta - OPERATOR_FEED_MM_PER_SECOND * FIXED_DT) < 1e-12,
    'operator transport must remain limited by advanceRate'
);
assert.equal(
    transportStats.transportSpeedMmPerSecond,
    OPERATOR_FEED_MM_PER_SECOND,
    'diagnostics must report prescribed transport independently'
);
assert.equal(transportStats.boundaryDrivenFeed, true);

function translatedBody(maxSpeed) {
    const world = new EndovascularPhysicsWorld({ fixedDt: FIXED_DT });
    const body = world.createRod('velocity-separation', 8, 5, {
        ...DEFAULT_TOOL_PROFILES.guidewire,
        maxSpeed,
        linearDamping: 1,
        stretchCompliance: 1,
        bendCompliance: 1,
        foldLimitStrength: 0,
        maxBendAngle: 179,
        sleepFrames: 1000
    });
    for (let index = 0; index < body.count; index++) {
        body.setNodePosition(index, index * 5, 0, 0);
        // Uniform axial transport plus a uniform transverse elastic/contact
        // response leaves segment lengths unchanged, isolating the velocity
        // limiter from the constitutive solver.
        body.velocityX[index] = OPERATOR_FEED_MM_PER_SECOND;
        body.velocityY[index] = OPERATOR_FEED_MM_PER_SECOND;
    }
    world.stepFixed();
    return Math.hypot(body.velocityX[3], body.velocityY[3], body.velocityZ[3]);
}

const unlimitedMaterialSpeed = translatedBody(
    DEFAULT_TOOL_PROFILES.guidewire.maxSpeed
);
const explicitlyGuardedSpeed = translatedBody(45);
assert.ok(
    unlimitedMaterialSpeed > 60,
    `elastic/material motion was still clipped (${unlimitedMaterialSpeed} mm/s)`
);
assert.ok(
    explicitlyGuardedSpeed <= 45 + 1e-4,
    'the optional generic velocity guard must remain available for fixtures'
);

class PlanarLumenContactField {
    constructor() {
        this.voxelSize = 0.5;
    }

    #write(position, radius, out) {
        // The lumen occupies y <= 0. Its surface normal points inward (-Y).
        const signedDistance = -position.y;
        const signedGap = signedDistance - radius;
        const penetration = Math.max(0, -signedGap);
        out.inside = signedDistance >= 0;
        out.violation = penetration > 0;
        out.signedDistance = signedDistance;
        out.signedGap = signedGap;
        out.penetration = penetration;
        out.branchId = 0;
        out.faceIndex = 0;
        out.source = 'planar-test';
        out.point.x = position.x;
        out.point.y = position.y;
        out.point.z = position.z;
        out.closestPoint.x = position.x;
        out.closestPoint.y = 0;
        out.closestPoint.z = position.z;
        out.inward.x = 0;
        out.inward.y = -1;
        out.inward.z = 0;
        out.normal.x = 0;
        out.normal.y = -1;
        out.normal.z = 0;
        out.target.x = position.x;
        out.target.y = position.y - penetration;
        out.target.z = position.z;
        out.timeOfImpact = out.violation ? 0 : 1;
        return out;
    }

    querySphere(position, radius, out) {
        return this.#write(position, radius, out);
    }

    queryCapsule(start, end, radius, out) {
        const useEnd = end.y >= start.y;
        this.#write(useEnd ? end : start, radius, out);
        out.segmentT = useEnd ? 1 : 0;
        return out;
    }

    sweepSphere(previous, current, radius, out) {
        const previousGap = -previous.y - radius;
        const currentGap = -current.y - radius;
        this.#write(current, radius, out);
        if (currentGap >= 0) {
            out.timeOfImpact = 1;
            return out;
        }
        if (previousGap <= 0) {
            out.timeOfImpact = 0;
            return out;
        }
        out.violation = true;
        out.timeOfImpact = previousGap / Math.max(1e-9, previousGap - currentGap);
        return out;
    }
}

function createWallGuidewire(id, centerY) {
    const world = new EndovascularPhysicsWorld({
        fixedDt: FIXED_DT,
        contactField: new PlanarLumenContactField(),
        iterations: 6,
        penetrationIterations: 8
    });
    const body = world.createRod(id, 5, 2, {
        ...DEFAULT_TOOL_PROFILES.guidewire,
        radius: 0.45,
        stretchCompliance: 1,
        bendCompliance: 1,
        foldLimitStrength: 0,
        maxBendAngle: 179,
        wallStaticFriction: 0,
        wallKineticFriction: 0,
        linearDamping: 1,
        sleepFrames: 1000
    });
    for (let index = 0; index < body.count; index++) {
        body.setNodePosition(index, index * 2, centerY, 0);
    }
    body.copyCurrentToPrevious();
    return { world, body };
}

// A direct non-penetration projection changes position, but at zero
// restitution it must not become a fresh inward material impulse.
{
    const { world, body } = createWallGuidewire(
        'non-inertial-wall-projection',
        -0.2
    );
    world.stepFixed();
    const stats = world.getStats().bodies[0];
    assert.ok(
        world.settledMaxPenetration <= 0.02,
        `wall projection left ${world.settledMaxPenetration} mm penetration`
    );
    assert.ok(
        stats.maximumWallProjectionSpeed > 10,
        'fixture must exercise a measurable positional wall correction'
    );
    assert.ok(
        stats.maximumReconstructedSpeed < 1,
        `wall correction leaked into guidewire momentum (` +
        `${stats.maximumReconstructedSpeed} mm/s)`
    );
    assert.equal(stats.wallProjectionVelocityRetention, 0);
}

// Physical motion away from the wall remains unconstrained. The contact fix
// removes only the projection contribution, not legitimate elastic/release
// velocity.
{
    const { world, body } = createWallGuidewire('inward-release', -0.5);
    body.velocityY.fill(-30);
    world.stepFixed();
    assert.ok(
        body.velocityY[2] < -29,
        `inward guidewire release was damped (${body.velocityY[2]} mm/s)`
    );
}

// A fast approach is stopped without penetration or a numerically generated
// rebound. Tangential transport stays independent and may exceed 45 mm/s.
{
    const { world, body } = createWallGuidewire('zero-restitution-sweep', -0.55);
    body.velocityX.fill(60);
    body.velocityY.fill(120);
    world.stepFixed();
    assert.ok(
        world.settledMaxPenetration <= 0.02,
        `swept contact left ${world.settledMaxPenetration} mm penetration`
    );
    assert.ok(
        Math.abs(body.velocityY[2]) < 1,
        `contact generated a normal rebound (${body.velocityY[2]} mm/s)`
    );
    assert.ok(
        body.velocityX[2] > 55,
        `wall response clipped tangential material motion (${body.velocityX[2]} mm/s)`
    );
}

console.log('Guidewire transport/velocity separation tests passed');
