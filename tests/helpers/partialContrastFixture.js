import * as THREE from 'three';
import {buildGraftPart} from '../../src/devices/stentGraftPart.js';
import {HybridContrastSystem} from '../../src/contrast/hybridContrastSystem.js';
export function partialFlowFixture() {
 const nodes=[[-20,0,0],[20,0,0],[50,0,0],[80,-6,0],[110,-12,0],[70,6,0],[110,12,0]];
 const segments=[[0,1],[1,2],[2,3],[3,4],[2,5],[5,6]].map(([a,b],id)=>({id,nodeStartId:a,nodeEndId:b,start:new THREE.Vector3(...nodes[a]),end:new THREE.Vector3(...nodes[b]),radiusStart:10,radiusEnd:10}));
 const material=new THREE.MeshBasicMaterial();
 const make=(points,radius)=>buildGraftPart({points:points.map(p=>new THREE.Vector3(...p)),radius,wallFit:{fit:p=>p},fabricMaterial:material});
 const parts=[make([[20,0,0],[28,0,0],[36,0,0],[43,0,0],[50,0,0]],7),make([[48,-2,0],[65,-4,0],[80,-6,0]],4),make([[48,2,0],[60,4,0],[70,6,0]],4)];
 parts[0].exposure.set([1,1,1,0,0]);
 for(const p of parts)p.mesh.geometry.attributes.position.array.set(p.target);
 const device={id:1,type:'body',phase:'deploying',parts};
 const system=new HybridContrastSystem({centerlineSegments:segments,flowOptions:{rootPoint:new THREE.Vector3(-20,0,0),cardiacOutputMlPerMin:600,axialDispersionMm2PerS:0}});
 return {parts,device,system,dispose(){for(const p of parts)p.mesh.geometry.dispose();material.dispose();}};
}
