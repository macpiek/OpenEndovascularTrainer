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
export function createColumnJunctionMixing(network,integral,territories,junctionField,spatial) {
    const junctions=junctionField?.junctions ?? [], indexByParent=new Map();
    const headers=new Float32Array(Math.max(1,junctions.length)*48);
    const values=new Float32Array(Math.max(1,junctions.length)*4);
    const items=junctions.map((j,index)=>{
        const at=index*48,anchors=j.anchors.map(a=>a.point),hub=closestTriangleWeights(j.center,anchors);
        indexByParent.set(j.edges[0],index);
        headers.set([...j.center.toArray(),j.radius,...j.axis.toArray(),0,...hub,0],at);
        const arms=j.anchors.map((anchor,arm)=>{
            const edge=network.edges[anchor.opticalEdgeIndex ?? j.edges[arm]],sign=arm===0?-1:1;
            const path=territories[edge.index],s=integral.edgeStart[edge.index]+(arm===0?edge.length:0)-sign*(anchor.opticalStartDistance??0),half=edge.cellLength*.5;
            const support=anchor.supportLength??0;
            headers.set([...anchor.point.toArray(),path,...anchor.direction.toArray(),anchor.travelled,s,sign,half,support],at+12+arm*12);
            return {path,s,sign,half,direction:anchor.direction,reach:anchor.travelled,support};
        });
        const armCount=new Set(arms.map(a=>a.path)).size;
        const bounded=j.boundedToArms||(j.territories?.size??armCount)>armCount;
        headers[at+7]=bounded?1:0;
        return {junction:j,arms,anchors,hub,index,bounded};
    });
    // A compact static list shares ownership across adjacent local junction
    // volumes. Concentration sampling is still bounded by each field's own
    // spatial support; this does not mix whole vascular components.
    const membership=[], lists=new Map();
    for(const item of items){
        const paths=item.junction.territories??new Set(item.arms.map(a=>a.path));
        item.paths=paths;
        if(!lists.has(paths)){
            lists.set(paths,(headers.length+membership.length)/4);
            membership.push(paths.size,...paths);
            while(membership.length%4)membership.push(-1);
        }
        headers[item.index*48+11]=lists.get(paths);
    }
    const records=new Float32Array(headers.length+membership.length);
    records.set(headers);records.set(membership,headers.length);
    const history=texture(values);
    const nearest={};
    function update(current,previous=current) {
        for(const item of items){const at=item.index*4,node=item.junction.nodeSample;
            history.image.data[at]=Math.max(0,current[node]);history.image.data[at+1]=Math.max(0,previous[node]);}
        history.needsUpdate=true;
    }
    function sample(index,point) {
        const item=items[index],j=item.junction,delta=point.clone().sub(j.center),axial=delta.dot(j.axis);
        const metric=Math.sqrt((axial/(j.radius*1.2))**2+Math.max(0,delta.lengthSq()-axial*axial)/(j.radius*2)**2);
        let blend=1-smooth(metric-1);
        for(const arm of item.arms)if(arm.support>0)
            blend*=1-smooth(2*delta.dot(arm.direction)/arm.support-1);
        if(blend<=0)return {value:0,blend:0};
        if(item.bounded){
            let coverage=0;
            const feather=Math.max(.08,Math.min(1,j.radius*.12));
            for(const arm of item.arms){
                const end=arm.s+arm.sign*arm.reach,lo=Math.min(arm.s,end),hi=Math.max(arm.s,end);
                const seed=arm.s+arm.sign*Math.max(0,delta.dot(arm.direction));
                spatial.nearest(arm.path,point,seed,3,nearest);
                spatial.sample(arm.path,Math.max(lo,Math.min(hi,nearest.s)),nearest);
                const score=point.distanceTo(nearest.point)-nearest.radius;
                coverage=Math.max(coverage,1-smooth((score+feather)/feather));
                if(coverage===1)break;
            }
            blend*=coverage;
            if(blend<=0)return {value:0,blend:0};
        }
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
    const hasPath = (index, a, b = a) => items[index].paths.has(a) || items[index].paths.has(b);
    return {indexByParent,items,records,history,update,sample,hasPath,
        memoryBytes:history.image.data.byteLength,
        dispose(){history.dispose();}};
}

export const columnJunctionMixingGLSL=/* glsl */`
uniform int columnSharedOffset;
uniform sampler2D columnSharedHistory;
float columnNearestRefined(int path,vec3 point,float seed,int iterations,out float score);
vec4 columnGeometryAt(vec4 range,float s,out vec4 derivative);
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
    uint first=uint(sharedRecord(uint(index*12+2)).w);
    vec4 paths=sharedRecord(first);
    int count=int(paths.x);
    for(int i=0;i<count;i++) {
        int component=(i+1)%4;
        if(component==0)paths=sharedRecord(first+uint((i+1)/4));
        int path=int(paths[component]);
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
    float blend=1.-smoothstep(1.,2.,metric);
    for(int arm=0;arm<3;arm++){
        vec3 direction=sharedRecord(at+uint(4+arm*3)).xyz;
        float support=sharedRecord(at+uint(5+arm*3)).w;
        if(support>0.)blend*=1.-smoothstep(support*.5,support,dot(delta,direction));
    }
    if(blend<=0.)return vec2(0.);
    if(axis.w>0.){
        // Shared wall labels do not enlarge the physical blood volume of a
        // fork. In the arch an unrelated clear branch can pass through its
        // broad ellipsoidal support; only this fork's local curved arms may
        // receive its mixture. The gate is identical for every wall owner.
        float coverage=0.,feather=clamp(center.w*.12,.08,1.);
        for(int arm=0;arm<3;arm++){
            uint base=at+uint(3+arm*3);
            vec4 anchor=sharedRecord(base),direction=sharedRecord(base+1u),coordinate=sharedRecord(base+2u);
            int path=int(anchor.w);
            float end=coordinate.x+coordinate.y*direction.w,score;
            float seed=coordinate.x+coordinate.y*max(0.,dot(delta,direction.xyz));
            float s=columnNearestRefined(path,point,seed,3,score);
            s=clamp(s,min(coordinate.x,end),max(coordinate.x,end));
            vec4 derivative,local=columnGeometryAt(texelFetch1D(columnGeometryRanges,uint(path)),s,derivative);
            score=length(point-local.xyz)-local.w;
            coverage=max(coverage,1.-smoothstep(-feather,0.,score));
            if(coverage>=1.)break;
        }
        blend*=coverage;
        if(blend<=0.)return vec2(0.);
    }
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
