import {Vector3} from 'three';
import {ContrastFlowNetwork} from '../../src/contrast/flowNetwork.js';

/** The same pulsatile flow in the Node and browser crossover benchmarks. */
export function createCrossoverFlowFromAsset(asset) {
    const a=asset.arrays,segments=[];
    for(let i=0;i<a.centerlineSegments.length;i+=9)segments.push({id:i/9,
        start:new Vector3().fromArray(a.centerlineSegments,i),end:new Vector3().fromArray(a.centerlineSegments,i+3),
        radiusStart:a.centerlineSegments[i+6],radiusEnd:a.centerlineSegments[i+7],safeRadius:a.centerlineSegments[i+8],
        nodeStartId:a.centerlineEdges[i/9*2],nodeEndId:a.centerlineEdges[i/9*2+1]});
    return new ContrastFlowNetwork(segments,{cardiacOutputMlPerMin:5000,heartRateBpm:72});
}
