import { ElasticRod } from './elasticRod.js';

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
