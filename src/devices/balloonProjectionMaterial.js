import {MeshBasicMaterial,DoubleSide} from 'three';

/** The fluoroscopy targets store positive RGB attenuation, not the final
 * displayed colour. Black RGB means no signal regardless of alpha; the
 * display shader turns this positive mask into a dark balloon silhouette. */
export function createBalloonProjectionMaterial() {
    return new MeshBasicMaterial({color:0xffffff,side:DoubleSide,
        transparent:true,opacity:.28,depthWrite:false,depthTest:false,toneMapped:false});
}
