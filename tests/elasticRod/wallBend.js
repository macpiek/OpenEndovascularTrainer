import { ElasticRod } from '../../physics/elasticRod.js';
import fs from 'fs';

const log = [];
const rod = new ElasticRod(10, 0.5, {
    logger: entry => log.push(entry)
});

// simple vessel: straight tube along x
const vessel = {
    segments: [
        { start: { x: 0, y: 0, z: 0 }, end: { x: 5, y: 0, z: 0 }, radius: 1 }
    ]
};

// place tip slightly outside to force contact with wall
rod.nodes[rod.nodes.length - 1].y = 1.2;

const dt = 0.01;
for (let i = 0; i < 200; i++) {
    // push tip along the vessel
    rod.nodes[rod.nodes.length - 1].vx = 1;
    rod.step(dt);
    rod.collide(vessel, dt);
}

const logPath = new URL('./wall-bend.log', import.meta.url);
fs.writeFileSync(logPath, JSON.stringify(log, null, 2));
console.log('saved log to', logPath.pathname);
console.log('final tip', rod.nodes[rod.nodes.length - 1]);
