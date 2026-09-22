import * as THREE from 'three';
import {createContactResult} from '../physics/collision/vesselContactField.js';

/** Geometric wall fit, shared by fabric and bare metal. This does not replace
 * elastic equilibrium; it prevents the release animation from cutting the wall. */
export class StentGraftWallFit {
    constructor(anatomy) {
        this.field=anatomy?.contactField;
        this.wall=anatomy?.geometry?.boundsTree;
        this.contact=createContactResult();
    }
    connected(a,b) {
        if(!this.field)return a.distanceTo(b)<=14;
        const count=Math.max(1,Math.ceil(a.distanceTo(b)));
        for(let i=0;i<=count;i++)if(this.field.querySphere(a.clone().lerp(b,i/count),0,this.contact).violation)return false;
        return true;
    }
    fit(point,anchor,clearance=.35) {
        const center=anchor.clone();
        if(this.field)for(let i=0;i<12;i++) {
            const hit=this.field.querySphere(center,clearance,this.contact);
            if(!hit.violation)break;
            center.addScaledVector(hit.normal,hit.penetration+.02);
        }
        const direction=point.clone().sub(center),length=direction.length();
        if(length<1e-8)return point.copy(center);
        direction.divideScalar(length);
        const hit=this.wall?.raycastFirst(new THREE.Ray(center,direction),THREE.DoubleSide,0,length+clearance);
        let end=hit?Math.min(length,Math.max(0,hit.distance-clearance)):length;
        point.copy(center).addScaledVector(direction,end);
        if(this.field&&this.field.querySphere(point,clearance,this.contact).violation) {
            let start=0;
            for(let i=0;i<16;i++) {
                const mid=(start+end)/2;
                point.copy(center).addScaledVector(direction,mid);
                if(this.field.querySphere(point,clearance,this.contact).violation)end=mid;else start=mid;
            }
            point.copy(center).addScaledVector(direction,start);
        }
        return point;
    }
}
