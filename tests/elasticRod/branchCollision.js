import { ElasticRod } from '../../physics/elasticRod.js';
import { vesselToGeometry } from '../../vesselGeometry.js';
import fs from 'fs';

const log = [];
const rod = new ElasticRod(10, 0.3, {
    logger: entry => log.push(entry)
});
// start near the bifurcation so the tip immediately interacts with it
for (const n of rod.nodes) n.x += 2.5;

// vessel with a side branch
const vessel = {
    segments: [
        { start: { x: 0, y: 0, z: 0 }, end: { x: 3, y: 0, z: 0 }, radius: 1 },
        { start: { x: 3, y: 0, z: 0 }, end: { x: 6, y: 0, z: 0 }, radius: 1 },
        { start: { x: 3, y: 0, z: 0 }, end: { x: 3, y: 3, z: 0 }, radius: 1 }
    ]
};
vessel.geometry = vesselToGeometry(vessel);

const dt = 0.001;
for (let i = 0; i < 100; i++) {
    const tip = rod.nodes[rod.nodes.length - 1];
    tip.vx = 0.2;
    if (tip.x > 2.9) tip.vy = 0.2;
    rod.step(dt);
    rod.collide(vessel, dt);
}

const tip = rod.nodes[rod.nodes.length - 1];
console.assert(
    Number.isFinite(tip.x) && Number.isFinite(tip.y) && Number.isFinite(tip.z),
    'tip should have finite coordinates after collision simulation'
);

const logPath = new URL('./branch-collision.log', import.meta.url);
fs.writeFileSync(logPath, JSON.stringify(log, null, 2));
console.log('saved log to', logPath.pathname);
console.log('final tip', tip);

