import * as THREE from 'three';

const BUCKETS=4096,CELL=8;
const hash=(x,y,z)=>((x*17+y*31+z*47)%BUCKETS+BUCKETS)%BUCKETS;
// Sparse spatial buckets keep vertex work local even on the multi-million
// triangle anatomy. Every accepted contact is retained; there is no top-N cut.
export const vesselDeformationGLSL=`
uniform sampler2D vesselBuckets;
uniform sampler2D vesselReferences;
uniform sampler2D vesselPatches;
uniform vec2 vesselReferenceSize;
uniform vec2 vesselPatchSize;
uniform vec3 vesselDeformMin;
uniform vec3 vesselDeformMax;
uniform bool vesselDeformActive;
vec4 vesselRead(sampler2D data,vec2 size,float index) {
    return texture2D(data,(vec2(mod(index,size.x),floor(index/size.x))+.5)/size);
}
vec3 vesselDisplacement(vec3 p) {
    if(!vesselDeformActive || any(lessThan(p,vesselDeformMin)) || any(greaterThan(p,vesselDeformMax)))return vec3(0.);
    vec3 cell=floor(p/${CELL}.);
    float hash=mod(dot(cell,vec3(17.,31.,47.)),${BUCKETS}.);
    vec2 bucket=vesselRead(vesselBuckets,vec2(64.),hash).xy;
    vec3 direction=vec3(0.);float depth=0.;
    for(int i=0;i<int(bucket.y);i++) {
        float index=vesselRead(vesselReferences,vesselReferenceSize,bucket.x+float(i)).x;
        vec4 center=vesselRead(vesselPatches,vesselPatchSize,2.*index);
        vec4 displacement=vesselRead(vesselPatches,vesselPatchSize,2.*index+1.);
        vec3 offset=p-center.xyz;
        float q=dot(offset,offset)/(center.w*center.w);
        // Do not drag the opposite side of a narrow lumen along with contact.
        float inward=dot(offset,displacement.xyz)/max(1e-12,displacement.w);
        float weight=pow(max(0.,1.-q),3.)*smoothstep(-.5,-.1,inward);
        float d=weight*displacement.w;
        depth=max(depth,d);direction+=weight*displacement.xyz;
    }
    float magnitude=length(direction);
    return magnitude>1e-12?direction*(depth/magnitude):vec3(0.);
}
`;

function texture(values) {
    const count=Math.max(1,values.length/4),width=Math.min(1024,count),height=Math.ceil(count/width);
    const data=new Float32Array(width*height*4);data.set(values);
    const result=new THREE.DataTexture(data,width,height,THREE.RGBAFormat,THREE.FloatType);
    result.needsUpdate=true;return result;
}

export class VesselDeformation {
    constructor() {
        this.revision=0;this.patches=[];this.signature='';this.updateListeners=new Set();
        this.buckets=new Array(BUCKETS);this.usedBuckets=[];
        this.uniforms={vesselBuckets:{value:texture(new Float32Array(BUCKETS*4))},
            vesselReferences:{value:texture([])},vesselPatches:{value:texture([])},
            vesselReferenceSize:{value:new THREE.Vector2(1,1)},vesselPatchSize:{value:new THREE.Vector2(1,1)},
            vesselDeformMin:{value:new THREE.Vector3()},vesselDeformMax:{value:new THREE.Vector3()},vesselDeformActive:{value:false},
            vesselDeformMaxDisplacement:{value:0}};
        // The hash table has a fixed 64x64 layout in the shader.
        const buckets=this.uniforms.vesselBuckets.value;
        buckets.image.width=64;buckets.image.height=64;
    }
    update(patches) {
        const signature=JSON.stringify(patches);
        if(signature===this.signature)return false;
        this.signature=signature;this.patches=patches;this.revision++;
        this.uniforms.vesselDeformActive.value=patches.length>0;
        this.uniforms.vesselDeformMaxDisplacement.value=patches.reduce((maximum,patch)=>Math.max(maximum,patch.displacement),0);
        if(!patches.length){for(const listener of this.updateListeners)listener();return true;}
        const buckets=this.buckets;
        for(const i of this.usedBuckets)buckets[i].clear();
        this.usedBuckets.length=0;
        const data=[],min=[Infinity,Infinity,Infinity],max=min.map(v=>-v);
        patches.forEach((p,i)=>{
            data.push(...p.center,p.width,...p.normal.map(n=>n*p.displacement),p.displacement);
            const lo=p.center.map((v,k)=>{min[k]=Math.min(min[k],v-p.width);return Math.floor((v-p.width)/CELL);});
            const hi=p.center.map((v,k)=>{max[k]=Math.max(max[k],v+p.width);return Math.floor((v+p.width)/CELL);});
            for(let x=lo[0];x<=hi[0];x++)for(let y=lo[1];y<=hi[1];y++)for(let z=lo[2];z<=hi[2];z++) {
                const at=hash(x,y,z),bucket=buckets[at]??=new Set();
                if(!bucket.size)this.usedBuckets.push(at);
                bucket.add(i);
            }
        });
        const table=this.uniforms.vesselBuckets.value.image.data,references=[];
        for(let i=0;i<BUCKETS;i++) {
            const bucket=buckets[i];table[4*i]=references.length/4;table[4*i+1]=bucket?.size??0;
            if(bucket)for(const index of bucket)references.push(index,0,0,0);
        }
        this.uniforms.vesselBuckets.value.needsUpdate=true;
        for(const [name,values,size] of [['vesselReferences',references,'vesselReferenceSize'],['vesselPatches',data,'vesselPatchSize']]) {
            // Reuse GPU allocation while the capacity is sufficient.
            let t=this.uniforms[name].value;
            if(t.image.data.length<values.length){t.dispose();t=this.uniforms[name].value=texture(values);}
            else {t.image.data.fill(0);t.image.data.set(values);t.needsUpdate=true;}
            this.uniforms[size].value.set(t.image.width,t.image.height);
        }
        this.uniforms.vesselDeformMin.value.fromArray(min);this.uniforms.vesselDeformMax.value.fromArray(max);
        for(const listener of this.updateListeners)listener();
        return true;
    }
    addUpdateListener(listener) {this.updateListeners.add(listener);return ()=>this.updateListeners.delete(listener);}
    // CPU counterpart of vesselDisplacement for device-to-deformed-wall queries.
    displacementAt(point,out=new THREE.Vector3()) {
        out.set(0,0,0);if(!this.patches.length)return out;
        const bucket=hash(Math.floor(point.x/CELL),Math.floor(point.y/CELL),Math.floor(point.z/CELL));
        const table=this.uniforms.vesselBuckets.value.image.data,refs=this.uniforms.vesselReferences.value.image.data;
        let depth=0;
        for(let j=0;j<table[4*bucket+1];j++) {
            const p=this.patches[refs[4*(table[4*bucket]+j)]],dx=point.x-p.center[0],dy=point.y-p.center[1],dz=point.z-p.center[2];
            const q=(dx*dx+dy*dy+dz*dz)/(p.width*p.width);
            if(q>=1)continue;
            const inward=dx*p.normal[0]+dy*p.normal[1]+dz*p.normal[2],t=Math.max(0,Math.min(1,(inward+.5)/.4));
            const weight=(1-q)**3*t*t*(3-2*t),d=weight*p.displacement;
            depth=Math.max(depth,d);out.x+=d*p.normal[0];out.y+=d*p.normal[1];out.z+=d*p.normal[2];
        }
        return out.lengthSq()>1e-24?out.setLength(depth):out.set(0,0,0);
    }
    attach(material) {
        if(material.userData?.attachVesselDeformation) {
            material.userData.attachVesselDeformation(this,material);
            return;
        }
        if(material.isShaderMaterial) {
            Object.assign(material.uniforms,this.uniforms);
            material.vertexShader=vesselDeformationGLSL+material.vertexShader.replace(/vec4\(position,\s*1\.0?\)/g,'vec4(position+vesselDisplacement(position),1.0)');
        } else {
            const original=material.onBeforeCompile;
            material.onBeforeCompile=(shader,renderer)=>{
                original.call(material,shader,renderer);Object.assign(shader.uniforms,this.uniforms);
                shader.vertexShader=vesselDeformationGLSL+shader.vertexShader.replace('#include <begin_vertex>',
                    '#include <begin_vertex>\ntransformed+=vesselDisplacement(position);');
            };
            material.customProgramCacheKey=()=> 'elastic-vessel-v1';
        }
        material.needsUpdate=true;
    }
    dispose(){this.updateListeners.clear();for(const key of ['vesselBuckets','vesselReferences','vesselPatches'])this.uniforms[key].value.dispose();}
}
