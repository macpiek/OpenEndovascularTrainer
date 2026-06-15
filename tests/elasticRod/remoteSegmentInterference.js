import { ElasticRod } from '../../src/physics/elasticRod.js';
import { vesselToGeometry } from '../../src/vesselGeometry.js';

// Rod initially placed within the main vessel far from a large-radius sheath
const rod = new ElasticRod(5, 0.5);
for (const n of rod.nodes) {
    n.x += 2; // shift into the main vessel segment
}

// Vessel containing a distant sheath with a very large radius and a narrower main vessel
const vessel = {
    segments: [
        { start: { x: 0, y: 0, z: 0 }, end: { x: 0, y: 1, z: 0 }, radius: 5 },
        { start: { x: 1, y: 0, z: 0 }, end: { x: 10, y: 0, z: 0 }, radius: 1 }
    ]
};
vessel.geometry = vesselToGeometry(vessel);

const dt = 0.01;
for (let i = 0; i < 200; i++) {
    // propel the tip forward along the main vessel
    rod.nodes[rod.nodes.length - 1].vx = 1;
    rod.step(dt);
    rod.collide(vessel, dt);
}

// The tip should advance along the vessel rather than being pinned by the distant sheath
console.assert(
    rod.nodes[rod.nodes.length - 1].x > 4,
    'tip should travel along the main vessel without interference'
);

console.log('final tip', rod.nodes[rod.nodes.length - 1]);
