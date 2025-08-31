import { VoxelContrastAgent, getVoxelMeshes } from '../voxelContrastAgent.js';

// Simple T-shaped vessel: a main segment that splits into two branches.
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
            end: { x: 10, y: 20, z: 0 },
            radius: 1,
            length: 14.14,
            flowSpeed: 10
        },
        {
            start: { x: 0, y: 10, z: 0 },
            end: { x: -10, y: 20, z: 0 },
            radius: 1,
            length: 14.14,
            flowSpeed: 10
        }
    ],
    // Map each segment to its downstream neighbours
    segmentGraph: [ [1, 2], [], [] ]
};

const agent = new VoxelContrastAgent(vessel, 2, 0.05);
agent.inject(1); // inject 1 ml into the first segment

// Advance the simulation so contrast reaches the branches
for (let frame = 0; frame < 5; frame++) {
    agent.update(0.2);
}

const rightBranch = agent.getSegmentConcentrations(1);
const leftBranch = agent.getSegmentConcentrations(2);
console.log('Right branch receives contrast:', rightBranch.some(v => v > 0));
console.log('Left branch receives contrast:', leftBranch.some(v => v > 0));

const meshes = getVoxelMeshes(agent, 0, true);
console.log('Wireframe mode:', meshes.length > 0 && meshes.every(m => m.material.wireframe));

