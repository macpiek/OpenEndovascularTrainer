import * as THREE from 'three';
import {RENAL_HALF_SIZE,RENAL_BODY_GLSL} from '../renalShape.js';

// Broad subcortical bases and individual papillae along the collecting tree.
// The compressed depth metric gives elliptical sections in the actual kidney.
const PYRAMID_METRIC = new THREE.Vector3(30,55,14);
const PYRAMID_COUNT=9;
// Dilute transported iodine over 120 ml of tissue per kidney, represented by
// five equal perfusion territories. This is an illustrative distribution volume.
const TISSUE_VOLUME_ML=120;
const TISSUE_SIGNAL_PER_MG_ML=18/350;
function createPyramids(side){
    // Fixed variation: anatomy must not flicker between frames or injections.
    let seed=side>0?371:829;
    const random=()=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed/4294967296;};
    const plans=[-1.98,-1.39,-.77,-.11,.48,1.15,1.91].map(angle=>{
        angle+=(random()-.5)*.18;
        return {
            base:new THREE.Vector3((.69+random()*.06)*Math.cos(angle),(.72+random()*.07)*Math.sin(angle),(random()-.5)*.24),
            apex:new THREE.Vector3(-.25-random()*.10,.43*Math.sin(angle)+(random()-.5)*.06,(random()-.5)*.12),
            radius:6.8+random()*3.2
        };
    });
    for(const depth of [-1,1])plans.push({
        base:new THREE.Vector3(.25+random()*.18,(random()-.5)*.33,depth*(.57+random()*.10)),
        apex:new THREE.Vector3(-.26,(random()-.5)*.14,depth*.10),
        radius:6.6+random()*1.6
    });
    return plans.map(({base,apex,radius})=>{
        const tip=apex.clone().multiply(PYRAMID_METRIC);
        const axis=base.clone().multiply(PYRAMID_METRIC).sub(tip);
        const length=axis.length();axis.normalize();
        const bend=new THREE.Vector3((random()-.5)*5,(random()-.5)*7,(random()-.5)*3);
        bend.addScaledVector(axis,-bend.dot(axis));
        return {tip:new THREE.Vector4(tip.x,tip.y,tip.z,length),
            axis:new THREE.Vector4(axis.x,axis.y,axis.z,radius),
            bend:new THREE.Vector4(bend.x,bend.y,bend.z,.78+random()*.36)};
    });
}

const vertexShader=`
varying vec3 point;
varying vec3 ray;
void main(){
    point=position;
    vec3 view=(modelViewMatrix*vec4(position,1.)).xyz;
    vec3 direction=isOrthographic?vec3(0.,0.,-1.):normalize(view);
    // Box geometry is normalized; undo the nonuniform kidney scale.
    ray=vec3(dot(modelViewMatrix[0].xyz,direction)/dot(modelViewMatrix[0].xyz,modelViewMatrix[0].xyz),
        dot(modelViewMatrix[1].xyz,direction)/dot(modelViewMatrix[1].xyz,modelViewMatrix[1].xyz),
        dot(modelViewMatrix[2].xyz,direction)/dot(modelViewMatrix[2].xyz,modelViewMatrix[2].xyz));
    gl_Position=projectionMatrix*vec4(view,1.);
}`;
const fragmentShader=`
varying vec3 point;
varying vec3 ray;
uniform float density[5];
uniform float side;
uniform float signalGain;
uniform bool debugMode;
uniform vec4 pyramidTips[${PYRAMID_COUNT}];
uniform vec4 pyramidAxes[${PYRAMID_COUNT}];
uniform vec4 pyramidBends[${PYRAMID_COUNT}];
${RENAL_BODY_GLSL}
float collectingBranch(vec3 p,vec3 start,vec3 end){
    vec3 axis=end-start;
    float t=clamp(dot(p-start,axis)/dot(axis,axis),0.,1.);
    float radius=mix(3.6,2.1,t);
    return 1.-smoothstep(radius*.72,radius*1.18,length(p-mix(start,end,t)));
}
vec2 renalCompartments(vec3 point,float body){
    vec3 p=point*vec3(side*30.,55.,14.);
    float medulla=0.;
    // The pelvis/calyces have no parenchymal enhancement. This is a volume
    // exclusion, so vessels and superimposed cortex remain visible in X-ray.
    float collecting=1.-smoothstep(.78,1.08,length((p-vec3(-12.,-1.5,0.))/vec3(8.,18.,8.5)));
    for(int j=0;j<${PYRAMID_COUNT};j++){
        vec3 delta=p-pyramidTips[j].xyz;
        vec3 axis=pyramidAxes[j].xyz;
        float along=dot(delta,axis);
        float t=along/pyramidTips[j].w;
        float u=clamp(t,0.,1.);
        vec3 lateral=normalize(cross(axis,vec3(0.,0.,1.)));
        vec3 transverse=cross(axis,lateral);
        vec3 offset=delta-along*axis-pyramidBends[j].xyz*(4.*u*(1.-u));
        float across=dot(offset,lateral);
        float through=dot(offset,transverse)/pyramidBends[j].w;
        // Bound the asymmetry even far from the axis. An unbounded across /
        // radius term can invert the radius near a papilla and create sharp
        // wedges through reversed smoothstep edges.
        float radius=.9+pyramidAxes[j].w*pow(u,.63);
        float asymmetry=across/sqrt(radius*radius+across*across);
        radius*=1.+.10*sin(t*5.+float(j)*2.3)+.08*asymmetry;
        // Gaussian shoulders blend gradually into cortical columns instead
        // of tracing the outline of a cone. Rounded ends and a smooth union
        // avoid both flat caps and creases where neighboring lobes overlap.
        float radialSquared=across*across+through*through;
        float pyramid=exp(-1.35*radialSquared/(radius*radius))
            *smoothstep(-.16,.20,t)*(1.-smoothstep(.58,1.28,t));
        medulla+=pyramid*(1.-medulla);
        vec3 calyxRoot=vec3(-12.5,pyramidTips[j].y*.65,0.);
        collecting=max(collecting,collectingBranch(p,calyxRoot,pyramidTips[j].xyz));
    }
    // Retain the cortical mantle over the broad pyramid bases.
    return vec2(medulla*(1.-smoothstep(.50,.94,body)),collecting);
}
void main(){
    vec3 d=normalize(ray),p=point+d*.001;
    float depth=0.;
    for(int i=0;i<72;i++){
        if(any(greaterThan(abs(p),vec3(1.001))))break;
        // Ellipsoid with a medial hilar indentation. No screen-space billboard.
        float body=renalBodyLevel(p,side);
        float coverage=1.-smoothstep(.84,1.,body);
        float band=clamp((p.y*.5+.5)*4.,0.,4.);
        float c=0.;
        for(int j=0;j<5;j++)c+=density[j]*max(0.,1.-abs(band-float(j)));
        // Low enhancement is confined to individual pyramids. Cortical
        // columns between them retain the same enhancement as outer cortex.
        vec2 compartments=renalCompartments(p,body);
        float tissueEnhancement=mix(1.4,.28,compartments.x)*(1.-compartments.y);
        depth+=coverage*c*tissueEnhancement*.045;
        p+=d*.045;
    }
    depth*=signalGain;
    // A diffuse nephrogram should remain translucent even with a large bolus.
    // Compress only the displayed signal; never discard transported iodine.
    depth=.10*(1.-exp(-depth/.10));
    if(depth<.00001)discard;
    gl_FragColor=vec4((debugMode?vec3(.3,1.,.8):vec3(1.))*depth,depth);
}`;

export class RenalParenchymaRenderer {
    constructor(perfusion) {
        this.perfusion=perfusion;
        this.group=new THREE.Group();this.group.name='renal-parenchymal-contrast';
        for(const kidney of perfusion.kidneys) {
            const pyramids=createPyramids(kidney.sign);
            const material=new THREE.ShaderMaterial({vertexShader,fragmentShader,uniforms:{
                density:{value:new Float32Array(5)},side:{value:kidney.sign},signalGain:{value:1},debugMode:{value:false},
                pyramidTips:{value:pyramids.map(p=>p.tip)},pyramidAxes:{value:pyramids.map(p=>p.axis)},
                pyramidBends:{value:pyramids.map(p=>p.bend)}},
                transparent:true,premultipliedAlpha:true,toneMapped:false,depthTest:false,depthWrite:false,blending:THREE.AdditiveBlending});
            const mesh=new THREE.Mesh(new THREE.BoxGeometry(2,2,2),material);
            mesh.position.copy(kidney.center);mesh.scale.set(...RENAL_HALF_SIZE);
            mesh.frustumCulled=false;mesh.userData.kidney=kidney;this.group.add(mesh);
        }
    }
    update(debugMode,gain) {
        let visible=false;
        for(const mesh of this.group.children) {
            const kidney=mesh.userData.kidney,u=mesh.material.uniforms;
            const meanConcentration=kidney.tissue.reduce((sum,m)=>sum+m,0)/TISSUE_VOLUME_ML;
            for(let i=0;i<5;i++){
                const localConcentration=kidney.tissue[i]/(TISSUE_VOLUME_ML/5);
                // Broad spatial averaging retains regional variation without
                // displaying a small arterial bolus as a dense tissue stripe.
                const concentration=.65*meanConcentration+.35*localConcentration;
                u.density.value[i]=concentration*TISSUE_SIGNAL_PER_MG_ML;
            }
            u.debugMode.value=debugMode;u.signalGain.value=gain;
            mesh.visible=kidney.tissue.some(m=>m>0.001);visible ||= mesh.visible;
        }
        this.group.visible=visible;
        return visible;
    }
    dispose(){for(const mesh of this.group.children){mesh.geometry.dispose();mesh.material.dispose();}}
}
