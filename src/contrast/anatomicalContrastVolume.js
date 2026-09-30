import * as THREE from 'three';
import { buildAnatomicalVolumeAtlas } from './anatomicalVolumeAtlas.js';
import { FlowConcentrationField } from './flowConcentrationField.js';

const vertexShader = `
in vec3 brickOrigin;
in vec3 atlasOrigin;
out vec3 point;
out vec3 direction;
flat out vec3 origin;
flat out vec3 atlas;
uniform float brickSize;
void main(){
    point=brickOrigin+(position+.5)*brickSize;
    origin=brickOrigin;atlas=atlasOrigin;
    vec4 view=modelViewMatrix*vec4(point,1.);
    vec3 ray=isOrthographic ? vec3(0.,0.,-1.) : view.xyz;
    direction=vec3(dot(modelViewMatrix[0].xyz,ray),dot(modelViewMatrix[1].xyz,ray),dot(modelViewMatrix[2].xyz,ray));
    gl_Position=projectionMatrix*view;
}`;
const fragmentShader = `
precision highp sampler3D;
in vec3 point;
in vec3 direction;
flat in vec3 origin;
flat in vec3 atlas;
out vec4 result;
uniform sampler3D anatomy;
uniform sampler3D concentration;
uniform sampler3D previousConcentration;
uniform vec3 atlasDimensions;
uniform vec3 fieldDimensions;
uniform float atlasTileSize;
uniform float voxelSize;
uniform float brickSize;
uniform float distanceQuantization;
uniform float displayAlpha;
uniform float signalGain;
uniform bool debugMode;
uniform vec3 debugColor;
void main(){
    vec3 ray=normalize(direction);
    vec3 safeRay=vec3(ray.x<0.?-1.:1.,ray.y<0.?-1.:1.,ray.z<0.?-1.:1.)*max(abs(ray),vec3(.0000001));
    vec3 t0=(origin-point)/safeRay,t1=(origin+brickSize-point)/safeRay;
    float lo=max(0.,max(max(min(t0.x,t1.x),min(t0.y,t1.y)),min(t0.z,t1.z)));
    float hi=min(min(max(t0.x,t1.x),max(t0.y,t1.y)),max(t0.z,t1.z));
    if(hi<=lo)discard;
    float n=ceil((hi-lo)/(voxelSize*.65)),ds=(hi-lo)/n;
    float depth=0.;
    for(int i=0;i<64;i++){
        if(float(i)>=n)break;
        vec3 p=point+ray*(lo+(float(i)+.5)*ds);
        vec3 uv=(atlas+vec3(1.5)+(p-origin)/voxelSize)/atlasDimensions;
        float distance=(texture(anatomy,uv).r*255.-128.)*distanceQuantization;
        // The collision field measures unsigned distance to ALL mesh faces,
        // then applies the lumen sign. Exterior wall faces can consequently
        // have a distance close to zero too. Never give negative distances
        // iodine coverage: a symmetric ramp paints a second, speckled shell
        // outside the actual blood volume. Soften from the lumen side only.
        float coverage=smoothstep(0.,voxelSize*.35,distance);
        if(coverage>0.){
            vec3 fieldUv=(atlas*(2./atlasTileSize)+.5+clamp((p-origin)/brickSize,0.,1.))/fieldDimensions;
            float c=mix(texture(previousConcentration,fieldUv).r,texture(concentration,fieldUv).r,displayAlpha);
            depth+=.5*c*coverage*ds;
        }
    }
    depth*=signalGain;
    result=vec4((debugMode?debugColor:vec3(1.))*depth,depth);
}`;
function texture3D(data, dims, type) {
    const t = new THREE.Data3DTexture(data, ...dims);
    t.format = THREE.RedFormat;
    t.type = type;
    t.minFilter = t.magFilter = THREE.LinearFilter;
    t.unpackAlignment = 1;
    t.needsUpdate = true;
    return t;
}
export class AnatomicalContrastVolume {
    constructor(network, offsets, contactField) {
        this.network = network;
        this.offsets = offsets;
        this.weights = new Float32Array(network.edges.length).fill(1);
        this.atlas = buildAnatomicalVolumeAtlas(contactField);
        const a = this.atlas,
            total = network.edges.reduce((n, e) => n + e.cellCount, 0);
        this.field = new FlowConcentrationField(network, offsets, total);
        this.cellValues = new Float32Array(total);
        this.current = new Float32Array(a.vertices.length);
        this.previous = this.current.slice();
        const texelCount = a.concentrationDims.reduce((x, y) => x * y, 1);
        this.currentTexels = new Float32Array(texelCount);
        this.previousTexels = new Float32Array(texelCount);
        this.texelIndices = new Uint32Array(a.brickCount * 8);
        for (let i = 0; i < a.brickCount; i++) {
            const ax = (a.atlasOrigins[i * 3] * 2) / a.tileSize,
                ay = (a.atlasOrigins[i * 3 + 1] * 2) / a.tileSize,
                az = (a.atlasOrigins[i * 3 + 2] * 2) / a.tileSize;
            for (let z = 0; z < 2; z++)
                for (let y = 0; y < 2; y++)
                    for (let x = 0; x < 2; x++)
                        this.texelIndices[i * 8 + x + 2 * y + 4 * z] =
                            ax +
                            x +
                            a.concentrationDims[0] *
                                (ay + y + a.concentrationDims[1] * (az + z));
        }
        const sampleEdges = new Uint32Array(a.vertices.length),
            sampleT = new Float32Array(a.vertices.length),
            location = {};
        for (let i = 0; i < a.vertices.length; i++) {
            const v = a.vertices[i],
                x = v % a.vertexDims[0],
                y = Math.floor(v / a.vertexDims[0]) % a.vertexDims[1],
                z = Math.floor(v / (a.vertexDims[0] * a.vertexDims[1]));
            network.findNearestLocationCoordinates(
                a.origin[0] + x * a.step,
                a.origin[1] + y * a.step,
                a.origin[2] + z * a.step,
                location
            );
            sampleEdges[i] = location.edgeIndex;
            sampleT[i] = location.t;
        }
        this.stencil = this.field.compileSamples(sampleEdges, sampleT);
        this.textures = [
            texture3D(a.data, a.atlasDims, THREE.UnsignedByteType),
            texture3D(this.currentTexels, a.concentrationDims, THREE.FloatType),
            texture3D(this.previousTexels, a.concentrationDims, THREE.FloatType)
        ];
        const box = new THREE.BoxGeometry(1, 1, 1),
            geometry = new THREE.InstancedBufferGeometry();
        geometry.index = box.index;
        geometry.setAttribute('position', box.attributes.position);
        this.origins = new Float32Array(a.brickOrigins.length);
        this.locations = this.origins.slice();
        geometry.setAttribute(
            'brickOrigin',
            new THREE.InstancedBufferAttribute(this.origins, 3)
        );
        geometry.setAttribute(
            'atlasOrigin',
            new THREE.InstancedBufferAttribute(this.locations, 3)
        );
        geometry.instanceCount = 0;
        const material = new THREE.ShaderMaterial({
            glslVersion: THREE.GLSL3,
            vertexShader,
            fragmentShader,
            uniforms: {
                anatomy: { value: this.textures[0] },
                concentration: { value: this.textures[1] },
                previousConcentration: { value: this.textures[2] },
                atlasDimensions: { value: new THREE.Vector3(...a.atlasDims) },
                fieldDimensions: {
                    value: new THREE.Vector3(...a.concentrationDims)
                },
                atlasTileSize: { value: a.tileSize },
                voxelSize: { value: a.voxelSize },
                brickSize: { value: a.step },
                distanceQuantization: { value: a.quantization },
                displayAlpha: { value: 1 },
                signalGain: { value: 0.14 },
                debugMode: { value: false },
                debugColor: { value: new THREE.Color(0x14b8ff) }
            },
            transparent: true,
            premultipliedAlpha: true,
            blending: THREE.AdditiveBlending,
            depthTest: false,
            depthWrite: false,
            toneMapped: false
        });
        this.mesh = new THREE.Mesh(geometry, material);
        this.mesh.name = 'anatomical-contrast-volume';
        this.mesh.frustumCulled = false;
        this.mesh.renderOrder = 6;
        this.mesh.visible = false;
    }
    update(plume, stock, advanceHistory, graftReplacesLumen = false) {
        if (advanceHistory) {
            this.previous.set(this.current);
            this.previousTexels.set(this.currentTexels);
        }
        for (const e of this.network.edges)
            for (let cell = 0; cell < e.cellCount; cell++) {
                const i = this.offsets[e.index] + cell;
                this.cellValues[i] =
                    e.transportExcluded ||
                    e.renderExcluded ||
                    (graftReplacesLumen && e.graftSections?.[cell])
                        ? 0
                        : (e.massMg[cell] + plume[i]) / e.volumes[cell] / stock;
            }
        this.field.update(this.cellValues);
        this.field.sampleInto(this.stencil, this.current);
        const a = this.atlas;

        let active = 0;
        for (let brick = 0; brick < a.brickCount; brick++) {
            let visible = false;
            for (let j = 0; j < 8; j++) {
                const v = a.brickVertices[brick * 8 + j];
                this.currentTexels[this.texelIndices[brick * 8 + j]] =
                    this.current[v];
                if (this.current[v] > 0 || this.previous[v] > 0) visible = true;
            }
            if (!visible) continue;
            for (let j = 0; j < 3; j++) {
                this.origins[active * 3 + j] = a.brickOrigins[brick * 3 + j];
                this.locations[active * 3 + j] = a.atlasOrigins[brick * 3 + j];
            }
            active++;
        }
        this.mesh.geometry.instanceCount = active;
        this.mesh.visible = active > 0;
        this.mesh.geometry.attributes.brickOrigin.needsUpdate = true;
        this.mesh.geometry.attributes.atlasOrigin.needsUpdate = true;
        this.textures[1].needsUpdate = true;
        this.textures[2].needsUpdate = true;
    }
    resetHistory() {
        this.current.fill(0);
        this.previous.fill(0);
        this.currentTexels.fill(0);
        this.previousTexels.fill(0);
        this.textures[1].needsUpdate = true;
        this.textures[2].needsUpdate = true;
        this.mesh.visible = false;
        this.mesh.geometry.instanceCount = 0;
    }
    dispose() {
        this.mesh.geometry.dispose();
        this.mesh.material.dispose();
        for (const t of this.textures) t.dispose();
    }
}
