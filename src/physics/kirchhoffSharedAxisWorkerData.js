import {BufferGeometry,BufferAttribute} from 'three';
import {MeshBVH} from 'three-mesh-bvh';
import {VesselContactField} from './collision/vesselContactField.js';

export const SHARED_AXIS_BODY_FIELDS=['x','y','z','velocityX','velocityY','velocityZ',
    'orientationX','orientationY','orientationZ','orientationW'];
const bodyProperties=['count','segmentCount','mass','radius','wallStaticFriction','wallKineticFriction'];

// postMessage clones these buffers once. Never transfer/detach geometry still
// used by the renderer or either main-thread access. Preserve the exact BVH
// and triangle order rather than rebuilding a possibly different hierarchy.
export function sharedAxisFieldMessage(field) {
    if(field.fallbackCollider)throw new Error('Worker requires a serializable finite-mesh contact field');
    const geometry=field.fallbackGeometry;
    if(!field.metadata||!field.arrays||!geometry?.boundsTree)throw new Error('Worker requires the loaded finite-mesh contact field');
    return {metadata:field.metadata,arrays:field.arrays,position:geometry.attributes.position.array,
        groups:geometry.groups,drawRange:geometry.drawRange,bvh:MeshBVH.serialize(geometry.boundsTree,{cloneBuffers:false}),
        bvhValidationDistance:field.bvhValidationDistance,capsuleBvhValidation:field.capsuleBvhValidationGap};
}
export function restoreSharedAxisFieldMessage(data) {
    const geometry=new BufferGeometry();geometry.setAttribute('position',new BufferAttribute(data.position,3));
    geometry.groups=data.groups;geometry.drawRange=data.drawRange;
    geometry.boundsTree=MeshBVH.deserialize(data.bvh,geometry);geometry.computeBoundingBox();
    return new VesselContactField({metadata:data.metadata,arrays:data.arrays},{fallbackGeometry:geometry,
        bvhValidationDistance:data.bvhValidationDistance,capsuleBvhValidation:data.capsuleBvhValidation});
}
export function sharedAxisToolMessage(tool) {
    const {body,...input}=tool;
    return {...input,nodeCoordinates:Array.from(tool.nodeCoordinates),
        body:Object.fromEntries(bodyProperties.map(k=>[k,body[k]]))};
}
export function updateSharedAxisWorkerTools(tools,previous=new Map()) {
    return tools.map(tool=>{
        let body=previous.get(tool.id);
        if(!body||body.count!==tool.body.count||body.segmentCount!==tool.body.segmentCount) {
            body={...tool.body};
            for(const key of SHARED_AXIS_BODY_FIELDS)body[key]=new Float64Array(key.startsWith('orientation')?body.segmentCount:body.count);
            body.orientationW.fill(1);previous.set(tool.id,body);
        }else Object.assign(body,tool.body);
        return {...tool,body};
    });
}
export function captureSharedAxisWorkerBodies(tools) {
    return tools.map(({id,body})=>({id,jointStateView:body.jointStateView,sharedAxisDiagnostics:body.sharedAxisDiagnostics,
        ...Object.fromEntries(SHARED_AXIS_BODY_FIELDS.map(key=>[key,body[key]]))}));
}
export function publishSharedAxisWorkerBodies(tools,bodies) {
    // Validate all buffers before touching either tool's published pose.
    for(const tool of tools) {
        const output=bodies.find(b=>b.id===tool.id);
        if(!output)throw new Error('Worker omitted a tool publication');
        for(const key of SHARED_AXIS_BODY_FIELDS)if(tool.body[key]&&output[key]?.length!==tool.body[key].length)
            throw new Error(`Worker publication layout changed: ${tool.id}/${key}`);
    }
    for(const tool of tools) {
        const output=bodies.find(b=>b.id===tool.id);
        for(const key of SHARED_AXIS_BODY_FIELDS)tool.body[key]?.set(output[key]);
        tool.body.jointStateView=output.jointStateView;tool.body.sharedAxisDiagnostics=output.sharedAxisDiagnostics;
    }
}
