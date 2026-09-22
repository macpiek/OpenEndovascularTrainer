import {Vector4} from 'three';
import {LineSegments2} from 'three/addons/lines/LineSegments2.js';
import {LineSegmentsGeometry} from 'three/addons/lines/LineSegmentsGeometry.js';
import {LineMaterial} from 'three/addons/lines/LineMaterial.js';

/** Round, continuous wires. A sampling footprint keeps subpixel wires continuous, but its opacity
 * is proportional to physical pixel coverage: the footprint is not wire thickness. */
export function graftWire(count,source,radius=.045) {
    const geometry=new LineSegmentsGeometry();geometry.setPositions(new Float32Array(count*6));
    const material=new LineMaterial({color:source.color,linewidth:1,transparent:true,depthWrite:false,alphaToCoverage:false});
    const projection=new LineMaterial({color:0xffffff,linewidth:1,transparent:true,depthWrite:false,alphaToCoverage:false,depthTest:false,toneMapped:false});
    const mesh=new LineSegments2(geometry,material);mesh.frustumCulled=false;
    mesh.userData.wireRadius=radius;mesh.userData.projectionMaterial=projection;
    const viewport=new Vector4();
    mesh.count=count;
    mesh.onBeforeRender=function(renderer,scene,camera,geometry,activeMaterial,group) {
        renderer.getCurrentViewport(viewport);
        activeMaterial.resolution.set(viewport.z,viewport.w);
        // Match the current viewport, including fluoroscopic render targets.
        const h=activeMaterial.resolution.y;
        const center=(this.userData.wireCenter??this.position).clone().applyMatrix4(this.matrixWorld).applyMatrix4(camera.matrixWorldInverse);
        const depth=camera.isPerspectiveCamera?Math.max(1,-center.z):1;
        const physicalPixels=radius*h*Math.abs(camera.projectionMatrix.elements[5])/depth;
        activeMaterial.linewidth=Math.max(1,physicalPixels);
        activeMaterial.opacity=Math.min(1,physicalPixels/activeMaterial.linewidth);
        // Use ordinary alpha blending, not 4-sample alpha-to-coverage: very
        // faint wires must not quantize back into intermittent visible dots.
    };
    return mesh;
}
export function wireSegment(mesh,index,a,b) {
    mesh.geometry.attributes.instanceStart.setXYZ(index,a.x,a.y,a.z);
    mesh.geometry.attributes.instanceEnd.setXYZ(index,b.x,b.y,b.z);
    if(index===0)mesh.userData.wireCenter=a.clone();
}
export function updateWire(mesh) {
    mesh.geometry.attributes.instanceStart.data.needsUpdate=true;
}
export function disposeWire(object) {
    if(object.isLineSegments2){object.material.dispose();object.userData.projectionMaterial.dispose();}
}
