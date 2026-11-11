import * as THREE from 'three';
import {AnatomicalContrastSurface,buildAnatomicalContrastSurface} from '../../src/contrast/anatomicalContrastSurface.js';
import {createFlowColumnIntegral} from '../../src/contrast/flowColumnIntegral.js';
import {createFlowColumnGeometry} from '../../src/contrast/flowColumnGeometry.js';
import {createFlowJunctionProjection} from '../../src/contrast/flowJunctionProjection.js';
export function aortoiliacColumnFixture(data) {
    const nodes = new Map(), edges = data.edges.map((e,index) => {
        const start = new THREE.Vector3(...e.start), end = new THREE.Vector3(...e.end), length = start.distanceTo(end);
        const edge = {...e,index,start,end,length,axis:end.clone().sub(start).normalize(),cellCount:e.areas.length,
            cellLength:length/e.areas.length, areas:Float64Array.from(e.areas), volumes:Float64Array.from(e.volumes), massMg:new Float64Array(e.areas.length)};
        for(const [id,point] of [[e.startNodeId,start],[e.endNodeId,end]]) if(!nodes.has(id))nodes.set(id,{id,point,parentEdgeIndex:-1,childEdgeIndices:[]});
        nodes.get(e.startNodeId).childEdgeIndices.push(index);nodes.get(e.endNodeId).parentEdgeIndex=index;
        return edge;
    });
    let frame = data.frames[0], count = 0;
    const network = {edges,nodes,time:0,hemodynamics:{heartRateBpm:72},
        getFaceSignedFlowMm3PerS:(index,face)=>frame.boundaryFlows[index][face===0?0:1]};
    const offsets = Uint32Array.from(edges,e=>{const at=count;count+=e.cellCount;return at;});
    const geometry = new THREE.BufferGeometry().setAttribute('position',new THREE.Float32BufferAttribute(data.positions,3));
    const surface = new AnatomicalContrastSurface(network,offsets,{fallbackGeometry:geometry},{projection:'column'});
    let cpuData;
    function cpu() {
        if(cpuData)return cpuData;
        const built=buildAnatomicalContrastSurface(network,{fallbackGeometry:geometry},surface.field,{projection:'column'});
        const integral=createFlowColumnIntegral(network,surface.field,built.territories,built.continuations);
        const spatial=createFlowColumnGeometry(network,integral,built.territories,built.continuations);
        const junctions=createFlowJunctionProjection(network,integral,built.territories,spatial,built.junctionField);
        integral.update(surface.current);junctions.shared.update(surface.current);
        return cpuData={built,integral,spatial,junctions};
    }
    const plume = new Float32Array(count);
    function show(index) {frame=data.frames[index];network.time=frame.time;edges.forEach((e,i)=>e.massMg.set(frame.mass[i]));surface.update(plume,.3,false);surface.previous.set(surface.current);surface.projection.update(surface.current,surface.current);cpuData?.integral.update(surface.current);cpuData?.junctions.shared.update(surface.current);return frame;}
    show(0);
    return {data,network,offsets,geometry,surface,show,cpu,dispose(){if(cpuData){cpuData.built.geometry.dispose();cpuData.integral.dispose();cpuData.spatial.dispose();cpuData.junctions.dispose();}surface.dispose();geometry.dispose();}};
}
