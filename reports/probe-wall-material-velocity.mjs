import assert from 'node:assert/strict';
import {
    DEFAULT_TOOL_PROFILES,
    EndovascularPhysicsWorld
} from '../src/physics/endovascularPhysicsWorld.js';
import { RodState } from '../src/physics/rodState.js';
import { GuidewireTransport } from '../src/physics/guidewireTransport.js';

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

const transportRod = new RodState(21, 5);
const transportSolver = new GuidewireTransport({
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
const transportDelta = transportSolver.advance(1, FIXED_DT);
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

const rows=[];
for(const config of [
 {name:'zero-input', y:-.2},
 {name:'valid-release',y:-.5,vy:-30},
 {name:'overlap-release-1',y:-.2,vy:-1},
 {name:'overlap-force-1',y:-.2,forceSpeed:-1},
 {name:'overlap-release-12',y:-.2,vy:-12},
 {name:'overlap-force-12',y:-.2,forceSpeed:-12},
 {name:'overlap-tangent-60',y:-.2,vx:60},
 {name:'valid-transverse-force',y:-.5,forceSpeed:-30},
 {name:'overlap-intrinsic-bend',y:-.2,rest:.05},
 {name:'overlap-intrinsic-bend-normal',y:-.2,restNormal:1},
 {name:'valid-intrinsic-bend-normal',y:-.5,restNormal:1}
]) {
 const {world,body}=createWallGuidewire(config.name,config.y);
 body.velocityX.fill(config.vx??0);body.velocityY.fill(config.vy??0);
 if(config.forceSpeed)for(let i=0;i<body.count;i++)body.forceY[i]=config.forceSpeed/FIXED_DT/body.inverseMass[i];
 if(config.rest)body.restRotation1.fill(config.rest);
 if(config.restNormal)body.restRotation2.fill(config.restNormal);
 const trace=[];
 body.debugConstraintPhase=(phase,b)=>{
  if(['afterIntegrate','primary','final','closureEnd'].includes(phase))trace.push({phase,
   x:[...b.x],y:[...b.y],wallY:[...b.wallProjectionY],
   vy:[...b.velocityY],angle:[...b.orientationX], previousAngle:[...b.previousOrientationX]});
 };
 world.stepFixed();
 const after=trace.at(-1), prediction=trace[0];
 rows.push({config,stats:world.getStats().bodies[0],
  nodes:Array.from({length:body.count},(_,i)=>{
   const net=[body.x[i]-body.previousX[i],body.y[i]-body.previousY[i],body.z[i]-body.previousZ[i]];
   const wall=[body.wallProjectionX[i],body.wallProjectionY[i],body.wallProjectionZ[i]],len=Math.hypot(...wall),n=wall.map(x=>len?x/len:0);
   const aligned=Math.max(0,net.reduce((s,v,k)=>s+v*n[k],0));
   const predicted=[prediction.x[i]-body.previousX[i],prediction.y[i]-body.previousY[i],0];
   const predictedInward=Math.max(0,predicted.reduce((s,v,k)=>s+v*n[k],0));
   const inputInward=Math.max(0,(config.vy??0)*FIXED_DT*n[1]+(config.vx??0)*FIXED_DT*n[0]);
   return {i,net,wall,actualVelocity:[body.velocityX[i],body.velocityY[i],body.velocityZ[i]],
    rejectAllNormal:net.map((v,k)=>(v-n[k]*aligned)/FIXED_DT),
    preserveInputNormal:net.map((v,k)=>(v-n[k]*Math.max(0,aligned-inputInward))/FIXED_DT),
    preservePredictedNormal:net.map((v,k)=>(v-n[k]*Math.max(0,aligned-predictedInward))/FIXED_DT),
    angularVelocity:[body.angularVelocityX[i]??null,body.angularVelocityY[i]??null,body.angularVelocityZ[i]??null]};
  }),trace});
}
console.log(JSON.stringify(rows,null,2));
