import { VoxelContrastAgent } from './voxelContrastAgent.js';

// Minimal vessel with a parent segment and a distal branch segment
const vessel = {
  left: { end: { x: 0, y: 20, z: 0 } },
  segments: [
    // Parent segment (proximal)
    {
      start: { x: 0, y: 0, z: 0 },
      end: { x: 0, y: 10, z: 0 },
      radius: 2,
      length: 10,
      volume: Math.PI * 2 * 2 * 10,
      startNode: 0,
      endNode: 1,
        // Faster flow in proximal segment to reflect quicker contrast movement
        flowSpeed: 20,
      parent: null,
    },
    // Distal segment
    {
      start: { x: 0, y: 10, z: 0 },
      end: { x: 0, y: 20, z: 0 },
      radius: 2,
      length: 10,
      volume: Math.PI * 2 * 2 * 10,
      startNode: 1,
      endNode: 2,
        // Faster flow in distal segment as well
        flowSpeed: 20,
      parent: 0,
    },
  ],
  nodes: [
    { position: { x: 0, y: 0, z: 0 }, segments: [0] },
    { position: { x: 0, y: 10, z: 0 }, segments: [0, 1] },
    { position: { x: 0, y: 20, z: 0 }, segments: [1] },
  ],
};

const agent = new VoxelContrastAgent(vessel, 2, 0.05);

// Inject 1 ml contrast into the main segment
agent.inject(1);

// Log average concentrations for the main segment and its child over several updates
const mainIndex = 0;
const childIndex = 1; // only child segment in this demo

for (let frame = 0; frame < 5; frame++) {
  agent.update(0.1);
  const mainConc = agent.getSegmentConcentrations(mainIndex)[0] || 0;
  const childConc = agent.getSegmentConcentrations(childIndex)[0] || 0;
  console.log(`Frame ${frame + 1}: main=${mainConc.toFixed(3)}, child=${childConc.toFixed(3)}`);
}
