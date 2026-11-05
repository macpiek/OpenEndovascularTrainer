import * as THREE from 'three';
import {sampleSheathPath} from '../physics/sheathPath.js';
import {dilatorDimensions,dilatorRadiusAtDistance} from './introducerDilator.js';

export function createIntroducerDilatorProjectionMaterial() {
    // The projection target stores positive attenuation. Black RGB disappears
    // in the display shader, regardless of alpha. Keep polymer below metal.
    return new THREE.MeshBasicMaterial({color:0xffffff,side:THREE.DoubleSide,
        transparent:true,opacity:.12,depthWrite:false,depthTest:false,toneMapped:false});
}

export function createIntroducerDilatorGeometry(sheath) {
    const {length,radius}=dilatorDimensions(sheath),insertion=sheath.dilatorInsertion??0;
    const source={...sheath,path:sheath.dilatorPath??sheath.shortPath,insertion};
    const curve=new THREE.Curve();
    curve.getPoint=(t,out=new THREE.Vector3())=>{
        const p=sampleSheathPath(source,insertion-length+t*length);return out.set(p.x,p.y,p.z);
    };
    const segments=Math.max(8,Math.ceil(length/2)),radial=12;
    const geometry=new THREE.TubeGeometry(curve,segments,1,radial,false),positions=geometry.attributes.position;
    const center=new THREE.Vector3(),p=new THREE.Vector3();
    for(let i=0;i<=segments;i++) {
        curve.getPoint(i/segments,center);
        const r=dilatorRadiusAtDistance(length*(1-i/segments),radius);
        for(let j=0;j<=radial;j++) {
            const at=i*(radial+1)+j;p.fromBufferAttribute(positions,at).sub(center).multiplyScalar(r).add(center);
            positions.setXYZ(at,p.x,p.y,p.z);
        }
    }
    geometry.computeVertexNormals();geometry.computeBoundingSphere();return geometry;
}
