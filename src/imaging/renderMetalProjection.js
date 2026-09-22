// Draw all metal into one MSAA pass. A second renderer.render() can clear a
// color-background scene even with autoClear=false, and some backends discard
// multisample storage after resolving each render call.
export function renderMetalProjection(renderer,scene,camera,{wireGroups,graftGroup,wireMaterial,graftMaterial}) {
    const hidden=[],materials=[],previousOverride=scene.overrideMaterial;
    const roots=new Set([...wireGroups,graftGroup]);
    try {
        for(const child of scene.children) {
            if(!child.isCamera&&!roots.has(child)&&child.visible){hidden.push(child);child.visible=false;}
        }
        const assign=(root,material)=>root?.traverse(object=>{
            if(object.material){materials.push([object,object.material]);object.material=object.userData?.projectionMaterial??material;}
        });
        for(const root of wireGroups)assign(root,wireMaterial);
        assign(graftGroup,graftMaterial);
        scene.overrideMaterial=null;
        renderer.render(scene,camera);
    } finally {
        scene.overrideMaterial=previousOverride;
        for(const [object,material] of materials)object.material=material;
        for(const object of hidden)object.visible=true;
    }
}
