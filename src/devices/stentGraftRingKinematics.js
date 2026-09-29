import * as THREE from 'three';

export const RING_GAP_MM=1.5;
export const RING_HEIGHT_MM=8;
// Full circumference includes 12 smooth waves. The proximal ring has the
// alternating valleys of an M crown. All coordinates are material distances.
export function ringWave(index,proximal) {
    const arm=Math.floor(index),t=index-arm;
    const level=n=>n%2?1:proximal&&n%4===2?.25:0;
    // Keep the middle of each wire arm nearly straight; round only the crowns.
    // A cosine over the entire arm created wide transverse loops on a flared
    // sleeve, although opening should primarily change the angle of the arms.
    const shoulder=.15;
    const wave=t<shoulder?t*t/(2*shoulder*(1-shoulder)):
        t>1-shoulder?1-(1-t)**2/(2*shoulder*(1-shoulder)):(t-shoulder/2)/(1-shoulder);
    return THREE.MathUtils.lerp(level(arm),level(arm+1),wave)-.5;
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

/** Sewn crowns retain their material coordinates. Wire bends between
 * those anchors to retain the ring’s total rest length, rather than sliding its crowns up
 * and down the fabric by changing the ring's material height. */
export function inextensibleRing(ring,sample) {
    ring.attachmentHeight=ring.height;ring.sewnReady=true;
    if(ring.packed)return packedRing(ring,sample);
    const bases=[],offsets=[];
    for(let arm=0;arm<ring.sides;arm++) {
        const start=sample(ring.center+ring.attachmentHeight*ringWave(arm,ring.proximal),arm/ring.sides*2*Math.PI);
        const end=sample(ring.center+ring.attachmentHeight*ringWave(arm+1,ring.proximal),(arm+1)/ring.sides*2*Math.PI);
        for(let k=arm?1:0;k<=12;k++) {
            const t=k/12,base=start.clone().lerp(end,t);
            const p=sample(ring.center+ring.attachmentHeight*ringWave(arm+t,ring.proximal),(arm+t)/ring.sides*2*Math.PI);
            bases.push(base);offsets.push(p.sub(base));
        }
    }
    const lengthAt=scale=>{
        let length=0;
        for(let i=1;i<bases.length;i++)length+=Math.hypot(
            bases[i].x-bases[i-1].x+scale*(offsets[i].x-offsets[i-1].x),
            bases[i].y-bases[i-1].y+scale*(offsets[i].y-offsets[i-1].y),
            bases[i].z-bases[i-1].z+scale*(offsets[i].z-offsets[i-1].z));
        return length;
    };
    let scale=ring.currentBendScale??1,length=lengthAt(scale);
    if(Math.abs(length-ring.restLength)>ring.restLength*1e-7) {
        let low=0,high=Math.max(1,scale),lowLength=lengthAt(0),highLength=lengthAt(high);
        while(highLength<ring.restLength&&high<1024){high*=2;highLength=lengthAt(high);}
        if(lowLength<=ring.restLength)for(let k=0;k<24;k++) {
            scale=THREE.MathUtils.lerp(low,high,THREE.MathUtils.clamp((ring.restLength-lowLength)/(highLength-lowLength),.05,.95));
            length=lengthAt(scale);
            if(Math.abs(length-ring.restLength)<ring.restLength*1e-7)break;
            if(length<ring.restLength){low=scale;lowLength=length;}else{high=scale;highLength=length;}
        }
        else {scale=0;length=lowLength;}
    }
    ring.currentBendScale=scale;ring.currentHeight=ring.attachmentHeight;ring.lengthError=length/ring.restLength-1;
    return bases.map((base,i)=>base.addScaledVector(offsets[i],scale));
}

// Fully covered cloth remains folded. Once exposed, fixed sewn anchors above
// replace this packed-shape approximation.
function packedRing(ring,sample) {
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
