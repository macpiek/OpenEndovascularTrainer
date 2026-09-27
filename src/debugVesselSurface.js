import * as THREE from 'three';

// Resolve depth within the vessel itself without writing into the scene depth
// buffer: instruments inside its translucent wall must remain visible.
export function createDebugVesselSurface(geometry, color = 0x4f8dff) {
    const target = new THREE.WebGLRenderTarget(1, 1);
    target.depthTexture = new THREE.DepthTexture(1, 1, THREE.UnsignedIntType);
    const material = new THREE.ShaderMaterial({
        uniforms: {color:{value:new THREE.Color(color)},vesselDepth:{value:target.depthTexture},resolution:{value:new THREE.Vector2(1,1)},shadingEnabled:{value:true}},
        transparent:true,depthWrite:false,side:THREE.FrontSide,
        vertexShader:`varying vec3 viewNormal;
            void main(){viewNormal=normalize(normalMatrix*normal);gl_Position=projectionMatrix*modelViewMatrix*vec4(position,1.0);}`,
        fragmentShader:`uniform vec3 color;uniform sampler2D vesselDepth;uniform vec2 resolution;uniform bool shadingEnabled;varying vec3 viewNormal;
            void main(){
                float nearest=texture2D(vesselDepth,gl_FragCoord.xy/resolution).x;
                if(gl_FragCoord.z>nearest+0.0000001)discard;
                float light=max(0.0,dot(normalize(viewNormal),normalize(vec3(-0.35,0.55,1.0))));
                gl_FragColor=vec4(color*(shadingEnabled ? 0.24+0.76*light : 1.0),0.48);
                #include <tonemapping_fragment>
                #include <colorspace_fragment>
            }`
    });
    const mesh=new THREE.Mesh(geometry,material);
    const legacyMaterial=new THREE.MeshBasicMaterial({color,side:THREE.DoubleSide,transparent:true,opacity:0.18,depthWrite:false});
    const depthMaterial=new THREE.MeshDepthMaterial({side:THREE.FrontSide,colorWrite:false,polygonOffset:true,polygonOffsetFactor:1,polygonOffsetUnits:1});
    const depthMesh=new THREE.Mesh(geometry,depthMaterial);depthMesh.matrixAutoUpdate=false;
    const depthScene=new THREE.Scene();depthScene.add(depthMesh);
    const size=new THREE.Vector2();let previousKey='';
    return {mesh,
        setShadingEnabled(enabled){material.uniforms.shadingEnabled.value=!!enabled;},
        setLegacyViewEnabled(enabled){mesh.material=enabled?legacyMaterial:material;},
        prepare(renderer,camera){
            if(mesh.material===legacyMaterial)return;
            mesh.updateWorldMatrix(true,false);camera.updateMatrixWorld(true);
            renderer.getDrawingBufferSize(size);
            const key=[size.x,size.y,...camera.matrixWorld.elements,...camera.projectionMatrix.elements,...mesh.matrixWorld.elements].join(',');
            if(key===previousKey)return;
            target.setSize(size.x,size.y);material.uniforms.resolution.value.copy(size);
            depthMesh.matrix.copy(mesh.matrixWorld);
            const previousTarget=renderer.getRenderTarget();
            try{renderer.setRenderTarget(target);renderer.clear();renderer.render(depthScene,camera);}
            finally{renderer.setRenderTarget(previousTarget);}
            previousKey=key;
        },
        dispose(){target.dispose();material.dispose();legacyMaterial.dispose();depthMaterial.dispose();}
    };
}
