import { VoxelContrastAgent, getVoxelMeshes } from '../voxelContrastAgent.js';

// Minimal straight vessel composed of two aligned segments
const vessel = {
    segments: [
        {
            start: { x: 0, y: 0, z: 0 },
            end: { x: 0, y: 10, z: 0 },
            radius: 2,
            length: 10,
            flowSpeed: 20
        },
        {
            start: { x: 0, y: 10, z: 0 },
            end: { x: 0, y: 20, z: 0 },
            radius: 2,
            length: 10,
            flowSpeed: 20
        }
    ]
};

const agent = new VoxelContrastAgent(vessel, 2, 0.05);
agent.inject(0.5); // inject 0.5 ml

for (let frame = 0; frame < 5; frame++) {
    agent.update(0.1);
    const conc = agent.getSegmentConcentrations(0);
    console.log(`Frame ${frame + 1}:`, conc.slice(0, 5).map(v => v.toFixed(4)).join(', '));
}

const meshes = getVoxelMeshes(agent, 0, true);
console.log('Wireframe mode:', meshes.length > 0 && meshes.every(m => m.material.wireframe));
