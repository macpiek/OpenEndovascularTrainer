import { ElasticRod } from '../../physics/elasticRod.js';
import fs from 'fs';

const log = [];
const rod = new ElasticRod(10, 1, {
    logger: entry => log.push(entry)
});

// bend the rod at the center
rod.nodes[5].y = 2;

const dt = 0.01;
for (let i = 0; i < 200; i++) {
    rod.step(dt);
}

const logPath = new URL('./straightening.log', import.meta.url);
fs.writeFileSync(logPath, JSON.stringify(log, null, 2));
console.log('saved log to', logPath.pathname);
console.log('final curvature', log[log.length - 1].curvature.toFixed(4));
