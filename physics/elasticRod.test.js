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
