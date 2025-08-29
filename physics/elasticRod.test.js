import { ElasticRod, setWallFriction } from './elasticRod.js';

// simple test: simulate slightly bent rod and ensure length deviation <=1%
const rod = new ElasticRod(5, 1, { mass: 1, bendingStiffness: 0.5 });
// introduce a perturbation
rod.nodes[2].y = 0.5;

const dt = 0.01;
for (let i = 0; i < 200; i++) {
    rod.step(dt);
}

let maxErr = 0;
const L = rod.segmentLength;
for (let i = 0; i < rod.nodes.length - 1; i++) {
    const n0 = rod.nodes[i];
    const n1 = rod.nodes[i + 1];
    const dist = Math.hypot(n1.x - n0.x, n1.y - n0.y, n1.z - n0.z);
    const err = Math.abs(dist - L) / L;
    if (err > maxErr) maxErr = err;
}

console.log('max length deviation', maxErr.toFixed(4));

// check that curvature is computed and tends toward zero after simulation
rod.updateCurvature();
const k = Math.hypot(rod.nodes[2].kx, rod.nodes[2].ky, rod.nodes[2].kz);
console.log('center curvature magnitude', k.toFixed(4));

// verify optional Laplacian smoothing moves nodes toward neighbor average
const smoothRod = new ElasticRod(5, 1, { mass: 1, bendingStiffness: 0, smoothingIterations: 5 });
smoothRod.nodes[2].y = 1;
smoothRod.step(dt);
console.log('smoothed center y', smoothRod.nodes[2].y.toFixed(4));

// verify bending forces increase with curvature and spread to neighbours
const forceRod = new ElasticRod(5, 1, { bendingStiffness: 1 });
forceRod.nodes[2].y = 1;
forceRod.resetForces();
forceRod.updateCurvature();
forceRod.accumulateBendingForces();
console.log('center bending fy', forceRod.nodes[2].fy.toFixed(4));
console.log('neighbor bending fy', forceRod.nodes[1].fy.toFixed(4));
console.assert(Math.abs(forceRod.nodes[1].fy) > 0, 'neighbour should receive bending force');

const smallBend = new ElasticRod(3, 1, { bendingStiffness: 1 });
smallBend.nodes[1].y = 0.2;
smallBend.resetForces();
smallBend.updateCurvature();
smallBend.accumulateBendingForces();
const smallForce = Math.abs(smallBend.nodes[1].fy);

const largeBend = new ElasticRod(3, 1, { bendingStiffness: 1 });
largeBend.nodes[1].y = 1;
largeBend.resetForces();
largeBend.updateCurvature();
largeBend.accumulateBendingForces();
const largeForce = Math.abs(largeBend.nodes[1].fy);
console.log('force small bend', smallForce.toFixed(4));
console.log('force large bend', largeForce.toFixed(4));
console.assert(largeForce > smallForce, 'larger curvature should yield greater straightening force');

// higher stiffness should produce proportionally larger straightening force
const soft = new ElasticRod(3, 1, { bendingStiffness: 1 });
soft.nodes[1].y = 1;
soft.resetForces();
soft.updateCurvature();
soft.accumulateBendingForces();
const softForce = Math.abs(soft.nodes[1].fy);

const stiff = new ElasticRod(3, 1, { bendingStiffness: 5 });
stiff.nodes[1].y = 1;
stiff.resetForces();
stiff.updateCurvature();
stiff.accumulateBendingForces();
const stiffForce = Math.abs(stiff.nodes[1].fy);
console.log('force low stiffness', softForce.toFixed(4));
console.log('force high stiffness', stiffForce.toFixed(4));
console.assert(stiffForce > softForce * 4, 'higher stiffness should greatly increase straightening force');

// collision test: node outside vessel should be clamped to surface
const collisionRod = new ElasticRod(2, 1);
// place second node outside a unit-radius vessel centered along x-axis
collisionRod.nodes[1].x = 1;
collisionRod.nodes[1].y = 2;
collisionRod.nodes[1].vy = 1;
const vessel = { segments: [{ start: { x: 0, y: 0, z: 0 }, end: { x: 2, y: 0, z: 0 }, radius: 1 }] };
collisionRod.collide(vessel);
console.log('collision y', collisionRod.nodes[1].y.toFixed(4));
console.log('collision vy', collisionRod.nodes[1].vy.toFixed(4));

// friction tests
setWallFriction(0.5, 0.25);

// tangential velocity below static threshold should be zeroed
const stickRod = new ElasticRod(2, 1);
stickRod.nodes[1].x = 1;
stickRod.nodes[1].y = 2;
stickRod.nodes[1].vx = 0.1;
stickRod.nodes[1].vy = -1;
stickRod.collide(vessel);
console.log('static friction vx', stickRod.nodes[1].vx.toFixed(4));
console.log('static friction vy', stickRod.nodes[1].vy.toFixed(4));
console.assert(Math.abs(stickRod.nodes[1].vx) < 1e-6, 'static friction should zero tangential velocity');
console.assert(Math.abs(stickRod.nodes[1].vy) < 1e-6, 'normal velocity should be zero after collision');

// tangential velocity above static threshold should be reduced by kinetic friction
const slideRod = new ElasticRod(2, 1);
slideRod.nodes[1].x = 1;
slideRod.nodes[1].y = 2;
slideRod.nodes[1].vx = 2;
slideRod.nodes[1].vy = -1;
slideRod.collide(vessel);
console.log('kinetic friction vx', slideRod.nodes[1].vx.toFixed(4));
console.log('kinetic friction vy', slideRod.nodes[1].vy.toFixed(4));
console.assert(Math.abs(slideRod.nodes[1].vx - 1.75) < 1e-6, 'kinetic friction should reduce tangential velocity');
console.assert(Math.abs(slideRod.nodes[1].vy) < 1e-6, 'normal velocity should be zero after collision');
