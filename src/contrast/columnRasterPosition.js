import {BVHShaderGLSL} from 'three-mesh-bvh';
import {vesselDeformationGLSL} from '../vesselDeformation.js';

// The column raster is an unindexed copy of the original triangle order.
// Each face already names its indirect BVH triangle. Its three vertex IDs
// therefore address the SAME revision-cached positions used for ray exits.
// Reusing them avoids evaluating every contact again at millions of duplicate
// wall vertices and avoids a second, slightly different floating-point wall.
export function attachColumnRasterPosition(material, geometry) {
    if (geometry.index || material.userData.columnRasterPositionCached) return false;
    const hook = 'vec3 projectionVertexPosition(vec3 point) { return point + vesselDisplacement(point); }';
    if (!material.vertexShader.includes(hook)) return false;
    material.vertexShader = `
precision highp usampler2D;
precision highp isampler2D;
${BVHShaderGLSL.common_functions}
${BVHShaderGLSL.bvh_struct_definitions}
uniform BVH lumenBvh;
` + material.vertexShader.replace(vesselDeformationGLSL, '').replace(hook, `
vec3 projectionVertexPosition(vec3 point) {
    uvec3 vertices = uTexelFetch1D(lumenBvh.index, uint(columnFace + .5)).xyz;
    return texelFetch1D(lumenBvh.position, vertices[gl_VertexID % 3]).xyz;
}`);
    material.userData.columnRasterPositionCached = true;
    material.needsUpdate = true;
    return true;
}
