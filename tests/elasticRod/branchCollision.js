import { ElasticRod } from '../../physics/elasticRod.js';
import fs from 'fs';

const log = [];
const rod = new ElasticRod(20, 0.3, {
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

const logPath = new URL('./branch-collision.log', import.meta.url);
fs.writeFileSync(logPath, JSON.stringify(log, null, 2));
console.log('saved log to', logPath.pathname);
console.log('final tip', rod.nodes[rod.nodes.length - 1]);
