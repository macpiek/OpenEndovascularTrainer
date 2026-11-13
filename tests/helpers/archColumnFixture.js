import * as T from 'three';
import {AnatomicalContrastSurface,buildAnatomicalContrastSurface} from '../../src/contrast/anatomicalContrastSurface.js';
import {createFlowColumnIntegral} from '../../src/contrast/flowColumnIntegral.js';
import {createFlowColumnGeometry} from '../../src/contrast/flowColumnGeometry.js';
import {createFlowJunctionProjection} from '../../src/contrast/flowJunctionProjection.js';
export function archColumnFixture(data,{reference=false}={}) {
 const nodes=new Map(),edges=data.edges.map((e,index)=>{const start=new T.Vector3(...e.start),end=new T.Vector3(...e.end),length=start.distanceTo(end);for(const[id,point]of[[e.startNodeId,start],[e.endNodeId,end]])if(!nodes.has(id))nodes.set(id,{id,point,parentEdgeIndex:-1,childEdgeIndices:[]});nodes.get(e.startNodeId).childEdgeIndices.push(index);nodes.get(e.endNodeId).parentEdgeIndex=index;return{...e,index,start,end,length,axis:end.clone().sub(start).normalize(),cellCount:e.areas.length,cellLength:length/e.areas.length,areas:Float64Array.from(e.areas),volumes:Float64Array.from(e.volumes),massMg:new Float64Array(e.areas.length)};});
 // Retired atlas edges remain in the edge table but are detached from the
 // transport graph. New fixtures preserve explicit node connectivity.
 if(data.nodes){nodes.clear();for(const node of data.nodes)nodes.set(node.id,{...node,point:new T.Vector3(...node.point),childEdgeIndices:[...node.childEdgeIndices]});}
 let frame=data.frames[0],count=0;
 const network={edges,nodes,time:0,hemodynamics:{heartRateBpm:72},getFaceSignedFlowMm3PerS:(index,face)=>frame.boundaryFlows[index][face===0?0:1]};
 const offsets=Uint32Array.from(edges,e=>{const at=count;count+=e.cellCount;return at;});
 const geometry=new T.BufferGeometry().setAttribute('position',new T.Float32BufferAttribute(data.positions,3));
 const surface=new AnatomicalContrastSurface(network,offsets,{fallbackGeometry:geometry},{projection:'column',junctionReconstruction:!reference});
 if(reference&&data.referenceShader)surface.material.fragmentShader=data.referenceShader;
 const referenceTextures=new Set();
 function decode(value){
  if(value?.kind==='array'){const bytes=Uint8Array.from(atob(value.data),c=>c.charCodeAt(0));return new globalThis[value.type](bytes.buffer);}
  if(value?.kind==='texture'){const texture=new T.DataTexture(decode(value.data),value.width,value.height,value.format,value.type);Object.assign(texture,{internalFormat:value.internalFormat,minFilter:value.minFilter,magFilter:value.magFilter,flipY:value.flipY,generateMipmaps:false,needsUpdate:true});referenceTextures.add(texture);return texture;}
  if(value?.kind&&T[value.kind])return new T[value.kind]().fromArray(value.data);
  if(Array.isArray(value))return value.map(decode);
  if(value&&typeof value==='object')return Object.fromEntries(Object.entries(value).map(([k,v])=>[k,decode(v)]));
  return value;
 }
 const saved=reference?data.referenceProjection:null;
 let savedFrames;
 if(saved){
  const meshGeometry=new T.BufferGeometry();
  for(const[key,a]of Object.entries(saved.attributes))meshGeometry.setAttribute(key,new T.BufferAttribute(decode(a.array),a.itemSize,a.normalized));
  if(saved.index)meshGeometry.setIndex(new T.BufferAttribute(decode(saved.index),1));
  meshGeometry.computeBoundingSphere();surface.mesh.geometry.dispose();surface.mesh.geometry=meshGeometry;
  surface.material.vertexShader=saved.vertexShader;surface.material.fragmentShader=saved.fragmentShader;
  for(const[key,value]of Object.entries(saved.uniforms))surface.material.uniforms[key]={value:decode(value)};
  savedFrames=saved.frames.map(frame=>Object.fromEntries(Object.entries(frame).map(([key,value])=>[key,decode(value)])));
  surface.mesh.onBeforeRender=(renderer,scene,camera)=>{
   const u=surface.material.uniforms;
   u.columnInverseProjection.value.copy(camera.projectionMatrixInverse);
   u.columnViewToLocal.value.multiplyMatrices(camera.matrixWorldInverse,surface.mesh.matrixWorld).invert();
   renderer.getCurrentViewport(u.columnViewport.value);
  };
 }
 const plume=new Float32Array(count);let cpuData;
 function cpu(){if(cpuData)return cpuData;const built=buildAnatomicalContrastSurface(network,{fallbackGeometry:geometry},surface.field,{projection:'column',junctionReconstruction:!reference});const integral=createFlowColumnIntegral(network,surface.field,built.territories,built.continuations),spatial=createFlowColumnGeometry(network,integral,built.territories,built.continuations),junctions=createFlowJunctionProjection(network,integral,built.territories,spatial,built.junctionField);integral.update(surface.current);junctions.shared.update(surface.current);return cpuData={built,integral,spatial,junctions};}
 function show(index){frame=data.frames[index];network.time=frame.time;edges.forEach((e,i)=>e.massMg.set(frame.mass[i]));surface.update(plume,.3,false);if(reference&&frame.referenceFieldValues){surface.current.fill(0);surface.current.set(frame.referenceFieldValues.subarray?.(0,surface.current.length)??frame.referenceFieldValues.slice(0,surface.current.length));}surface.previous.set(surface.current);surface.projection.update(surface.current,surface.current);cpuData?.integral.update(surface.current);cpuData?.junctions.shared.update(surface.current);
 if(saved){for(const[key,value]of Object.entries(savedFrames[index]))surface.material.uniforms[key]={value};surface.material.uniforms.projectionCullActive.value=false;surface.mesh.geometry.clearGroups();surface.mesh.geometry.addGroup(0,surface.mesh.geometry.index?.count??surface.mesh.geometry.attributes.position.count,0);surface.mesh.visible=true;}
 return frame;}
 show(0);return{data,network,offsets,geometry,surface,show,cpu,dispose(){if(cpuData){cpuData.built.geometry.dispose();cpuData.integral.dispose();cpuData.spatial.dispose();cpuData.junctions.dispose();}surface.dispose();geometry.dispose();for(const texture of referenceTextures)texture.dispose();}};
}
