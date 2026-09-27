import * as THREE from 'three';

export const RING_GAP_MM=1.5;
export const RING_HEIGHT_MM=8;
// Full circumference includes 12 smooth waves. The proximal ring has the
// alternating valleys of an M crown. All coordinates are material distances.
export function ringWave(index,proximal) {
    const arm=Math.floor(index),t=index-arm;
    const level=n=>n%2?1:proximal&&n%4===2?.25:0;
    return THREE.MathUtils.lerp(level(arm),level(arm+1),(1-Math.cos(Math.PI*t))/2)-.5;
}
export function ringLayout(length,sides=24,nominalRadius=7,scale=1,proximalInset=0) {
    const count=Math.max(1,Math.floor((length/scale+2)/12)),pitch=(length-proximalInset)/count;
    return Array.from({length:count},(_,i)=>{
        const start=proximalInset+i*pitch+(i?RING_GAP_MM/2:0);
        const end=proximalInset+(i+1)*pitch-(i<count-1?RING_GAP_MM/2:0);
        const height=Math.min(RING_HEIGHT_MM*scale,end-start),samples=sides*12;
        const ring={center:(start+end)/2,height,maxHeight:end-start,proximal:i===0,sides,samples};
        const radius=typeof nominalRadius==='function'?nominalRadius(ring.center):nominalRadius;
        let length=0,previous;
        for(let j=0;j<=samples;j++) {
            const a=j/samples*2*Math.PI,p=new THREE.Vector3(radius*Math.cos(a),radius*Math.sin(a),height*ringWave(j/12,ring.proximal));
            if(previous)length+=p.distanceTo(previous);previous=p;
        }
        ring.restLength=length;return ring;
    });
}

/** Bend the wave instead of stretching its wire as its circumference changes.
 * Arc length is solved on the current fabric surface. A fixed material band
 * reserves clearance to both neighbouring rings, including while crimped.
 */
export function inextensibleRing(ring,sample) {
    const evaluate=height=>{
        const points=[];let length=0;
        for(let j=0;j<=ring.samples;j++) {
            const angle=j/ring.samples*2*Math.PI;
            const p=sample(ring.center+height*ringWave(j/12,ring.proximal),angle);
            if(j)length+=p.distanceTo(points[j-1]);points.push(p);
        }
        return {points,length,height};
    };
    let result=evaluate(ring.currentHeight??ring.height);
    const tolerance=ring.restLength*1e-6;
    if(Math.abs(result.length-ring.restLength)>tolerance) {
        let low=evaluate(0),high=evaluate(ring.maxHeight);
        result=high;
        // A safeguarded secant avoids 20 full resamplings on every committed
        // frame. Reusing the previous height makes rigid motions one evaluation.
        if(high.length>=ring.restLength&&low.length<=ring.restLength)for(let i=0;i<20;i++) {
            const t=THREE.MathUtils.clamp((ring.restLength-low.length)/(high.length-low.length),.02,.98);
            result=evaluate(THREE.MathUtils.lerp(low.height,high.height,t));
            if(Math.abs(result.length-ring.restLength)<=tolerance)break;
            if(result.length<ring.restLength)low=result;else high=result;
        }
    }
    ring.lengthError=result.length/ring.restLength-1;
    ring.currentHeight=result.height;
    return result.points;
}
