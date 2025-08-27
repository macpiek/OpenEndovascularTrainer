import { ContrastAgent } from './contrastAgent.js';

// Minimal vessel with a parent segment and a distal sheath-connected segment
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
      flowSpeed: 5,
      parent: null,
    },
    // Distal segment connected to sheath
    {
      start: { x: 0, y: 10, z: 0 },
      end: { x: 0, y: 20, z: 0 },
      radius: 2,
      length: 10,
      volume: Math.PI * 2 * 2 * 10,
      startNode: 1,
      endNode: 2,
      flowSpeed: 5,
      parent: 0,
    },
  ],
  nodes: [
    { position: { x: 0, y: 0, z: 0 }, segments: [0] },
    { position: { x: 0, y: 10, z: 0 }, segments: [0, 1] },
    { position: { x: 0, y: 20, z: 0 }, segments: [1] },
  ],
};

const agent = new ContrastAgent(vessel);

// Inject 1 ml contrast into sheath segment (index resolved internally)
agent.inject(1);

// Log concentrations for sheath and its parent over several updates
const sheathIndex = agent.sheathIndex;
const parentIndex = vessel.segments[sheathIndex].parent;

for (let frame = 0; frame < 5; frame++) {
  agent.update(0.1);
  const sheathConc = agent.concentration[sheathIndex];
  const parentConc = agent.concentration[parentIndex];
  console.log(`Frame ${frame + 1}: sheath=${sheathConc.toFixed(3)}, parent=${parentConc.toFixed(3)}`);
}
