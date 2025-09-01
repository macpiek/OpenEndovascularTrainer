import { ElasticRod } from '../../physics/elasticRod.js';
import fs from 'fs';

const log = [];
// Start the rod before the branch so the tip must choose a path at the
// bifurcation. A shorter initial length ensures the tip reaches the branch
// after simulation begins rather than starting past it.
const rod = new ElasticRod(10, 0.3, {
    logger: entry => log.push(entry)
});

// vessel with a side branch
const vessel = {
    segments: [
        { start: { x: 0, y: 0, z: 0 }, end: { x: 3, y: 0, z: 0 }, radius: 1 },
        { start: { x: 3, y: 0, z: 0 }, end: { x: 6, y: 0, z: 0 }, radius: 1 },
        { start: { x: 3, y: 0, z: 0 }, end: { x: 3, y: 3, z: 0 }, radius: 1 }
    ]
};

const dt = 0.01;
for (let i = 0; i < 400; i++) {
    // push tip forward
    rod.nodes[rod.nodes.length - 1].vx = 1;
    // bias upward to prefer the branch
    if (rod.nodes[rod.nodes.length - 1].x > 2.5) {
        rod.nodes[rod.nodes.length - 1].vy = 1;
    }
    rod.step(dt);
    rod.collide(vessel, dt);
}

// After navigating the branch the tip should have moved significantly upward.
console.assert(
    rod.nodes[rod.nodes.length - 1].y > 1.5,
    'tip should enter the branch and move upward'
);

const logPath = new URL('./branch-collision.log', import.meta.url);
fs.writeFileSync(logPath, JSON.stringify(log, null, 2));
console.log('saved log to', logPath.pathname);
console.log('final tip', rod.nodes[rod.nodes.length - 1]);
