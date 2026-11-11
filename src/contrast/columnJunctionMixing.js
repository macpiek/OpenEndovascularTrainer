import * as THREE from 'three';
import {closestTriangleWeights} from './junctionConcentrationKernel.js';

function texture(values) {
    const width=Math.max(1,Math.min(1024,Math.ceil(values.length/4))), height=Math.max(1,Math.ceil(values.length/4/width));
    const data=new Float32Array(width*height*4);data.set(values);
    const result=new THREE.DataTexture(data,width,height,THREE.RGBAFormat,THREE.FloatType);
    result.needsUpdate=true;return result;
}
const smooth=x=>{x=Math.max(0,Math.min(1,x));return x*x*(3-2*x);};

// Reuse the common-lumen reconstruction used by the anatomical surface,
// sampling LOCAL transported cells along each arm, never remote anchors.
// Column projection previously lost this field and selected a tube owner.
export function createColumnJunctionMixing(network,integral,territories,junctionField) {
    const junctions=junctionField?.junctions ?? [], indexByParent=new Map();
    const records=new Float32Array(Math.max(1,junctions.length)*48);
    const values=new Float32Array(Math.max(1,junctions.length)*4);
    const items=junctions.map((j,index)=>{
        const at=index*48,anchors=j.anchors.map(a=>a.point),hub=closestTriangleWeights(j.center,anchors);
        indexByParent.set(j.edges[0],index);
        records.set([...j.center.toArray(),j.radius,...j.axis.toArray(),index,...hub,0],at);
        const arms=j.anchors.map((anchor,arm)=>{
            const edge=network.edges[j.edges[arm]],sign=arm===0?-1:1;
            const path=territories[edge.index],s=integral.edgeStart[edge.index]+(arm===0?edge.length:0),half=edge.cellLength*.5;
            records.set([...anchor.point.toArray(),path,...anchor.direction.toArray(),anchor.travelled,s,sign,half,0],at+12+arm*12);
            return {path,s,sign,half,direction:anchor.direction,reach:anchor.travelled};
        });
        return {junction:j,arms,anchors,hub,index};
    });
    const history=texture(values);
    function update(current,previous=current) {
        for(const item of items){const at=item.index*4,node=item.junction.nodeSample;
            history.image.data[at]=Math.max(0,current[node]);history.image.data[at+1]=Math.max(0,previous[node]);}
        history.needsUpdate=true;
    }
    function sample(index,point) {
        const item=items[index],j=item.junction,delta=point.clone().sub(j.center),axial=delta.dot(j.axis);
        const metric=Math.sqrt((axial/(j.radius*1.2))**2+Math.max(0,delta.lengthSq()-axial*axial)/(j.radius*2)**2);
        const blend=1-smooth(metric-1);if(blend<=0)return {value:0,blend:0};
        const node=history.image.data[index*4];
        const armValue=(arm,distance)=>{
            const a=item.arms[arm],d=Math.max(0,Math.min(a.reach,distance));
            // The node is a finite common blood volume, not a zero-length
            // interface. Resolve its mixture over an anatomical mixing length
            // instead of dropping it within half of one transport cell.
            const local = integral.sample(a.path,a.s+a.sign*Math.max(a.half,d));
            return node + (local-node)*smooth(d/Math.max(a.half,j.radius*.8));
        };
        const w=closestTriangleWeights(point,item.anchors),h=item.hub;
        let hubWeight=1;for(let i=0;i<3;i++)if(h[i]>1e-6)hubWeight=Math.min(hubWeight,w[i]/h[i]);
        let value=hubWeight*(axial>0?armValue(0,axial):node);
        for(let arm=0;arm<3;arm++){
            const projected=delta.dot(item.arms[arm].direction),parent=arm>0&&projected<0&&axial>0;
            value+=Math.max(0,w[arm]-hubWeight*h[arm])*armValue(parent?0:arm,parent?-projected:projected);
        }
        return {value,blend};
    }
    const hasPath = (index, a, b = a) => items[index].arms.some(arm => arm.path === a || arm.path === b);
    return {indexByParent,items,records,history,update,sample,hasPath,
        memoryBytes:history.image.data.byteLength,
        dispose(){history.dispose();}};
}

export const columnJunctionMixingGLSL=/* glsl */`
uniform int columnSharedOffset;
uniform sampler2D columnSharedHistory;
vec4 sharedRecord(uint index) {return texelFetch1D(columnJunctionRecords,uint(columnSharedOffset)+index);}
vec3 sharedTriangleWeights(vec3 point, vec3 a, vec3 b, vec3 c) {
    vec3 ab = b - a, ac = c - a, ap = point - a;
    float ab2 = dot(ab, ab), ac2 = dot(ac, ac), crossDot = dot(ab, ac);
    if (ab2 * ac2 - crossDot * crossDot <= 1e-12 * max(ab2 * ac2, 1e-30)) {
        float bc2 = dot(c - b, c - b);
        if (ab2 >= ac2 && ab2 >= bc2) {
            float t = ab2 > 0. ? clamp(dot(ap, ab) / ab2, 0., 1.) : 0.;
            return vec3(1. - t, t, 0.);
        }
        if (ac2 >= bc2) {
            float t = ac2 > 0. ? clamp(dot(ap, ac) / ac2, 0., 1.) : 0.;
            return vec3(1. - t, 0., t);
        }
        float t = bc2 > 0. ? clamp(dot(point - b, c - b) / bc2, 0., 1.) : 0.;
        return vec3(0., 1. - t, t);
    }
    float d1 = dot(ab, ap), d2 = dot(ac, ap);
    if (d1 <= 0. && d2 <= 0.) return vec3(1., 0., 0.);
    vec3 bp = point - b;
    float d3 = dot(ab, bp), d4 = dot(ac, bp);
    if (d3 >= 0. && d4 <= d3) return vec3(0., 1., 0.);
    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0. && d1 >= 0. && d3 <= 0.) {
        float v = d1 / (d1 - d3); return vec3(1. - v, v, 0.);
    }
    vec3 cp = point - c;
    float d5 = dot(ab, cp), d6 = dot(ac, cp);
    if (d6 >= 0. && d5 <= d6) return vec3(0., 0., 1.);
    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0. && d2 >= 0. && d6 <= 0.) {
        float w = d2 / (d2 - d6); return vec3(1. - w, 0., w);
    }
    float va = d3 * d6 - d5 * d4;
    if (va <= 0. && d4 - d3 >= 0. && d5 - d6 >= 0.) {
        float w = (d4 - d3) / ((d4 - d3) + (d5 - d6)); return vec3(0., 1. - w, w);
    }
    float denominator = va + vb + vc, v = vb / denominator, w = vc / denominator;
    return vec3(1. - v - w, v, w);
}

float sharedNodeConcentration(int index) {
    vec2 history=texelFetch1D(columnSharedHistory,uint(index)).xy;
    return mix(history.y,history.x,displayAlpha);
}
bool sharedJunctionHasPath(int index,int a,int b) {
    for(int arm=0;arm<3;arm++) {
        int path=int(sharedRecord(uint(index*12+3+arm*3)).w);
        if(path==a||path==b)return true;
    }
    return false;
}
float sharedArmConcentration(int index,int arm,float distance,float node) {
    uint at=uint(index*12+3+arm*3);
    vec4 anchor=sharedRecord(at), direction=sharedRecord(at+1u);
    vec4 coordinate=sharedRecord(at+2u);
    float d=clamp(distance,0.,direction.w);
    float radius=sharedRecord(uint(index*12)).w;
    float localValue=columnSample(int(anchor.w),coordinate.x+coordinate.y*max(coordinate.z,d)).x;
    return mix(node,localValue,smoothstep(0.,max(coordinate.z,radius*.8),d));
}
vec2 sharedJunctionConcentration(int index,vec3 point) {
    uint at=uint(index*12);
    vec4 center=sharedRecord(at),axis=sharedRecord(at+1u);
    vec3 delta=point-center.xyz;float axial=dot(delta,axis.xyz);
    float metric=length(vec2(axial/(center.w*1.2),sqrt(max(0.,dot(delta,delta)-axial*axial))/(center.w*2.)));
    float blend=1.-smoothstep(1.,2.,metric);if(blend<=0.)return vec2(0.);
    vec3 a=sharedRecord(at+3u).xyz,b=sharedRecord(at+6u).xyz,c=sharedRecord(at+9u).xyz;
    vec3 weights=sharedTriangleWeights(point,a,b,c),hub=sharedRecord(at+2u).xyz;
    float hubWeight=1.;for(int i=0;i<3;i++)if(hub[i]>1e-6)hubWeight=min(hubWeight,weights[i]/hub[i]);
    float node=sharedNodeConcentration(index);
    float value=hubWeight*(axial>0.?sharedArmConcentration(index,0,axial,node):node);
    for(int arm=0;arm<3;arm++){
        vec3 direction=sharedRecord(at+uint(4+arm*3)).xyz;
        float projected=dot(delta,direction);bool parent=arm>0&&projected<0.&&axial>0.;
        value+=max(0.,weights[arm]-hubWeight*hub[arm])*sharedArmConcentration(index,parent?0:arm,parent?-projected:projected,node);
    }
    return vec2(value,blend);
}
`;
