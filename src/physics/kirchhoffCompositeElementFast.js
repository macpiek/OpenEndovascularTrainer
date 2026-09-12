import { compositeElementFastKernelBytes } from './kirchhoffCompositeElementFastKernelBytes.js';
export { captureCompositeReferenceFrames, compileCompositeMaterial } from './kirchhoffCompositeElement.js';

// Same constitutive law as kirchhoffCompositeElement. The generated fixed
// stencil contains all position/spin derivatives of transported directors,
// curvature and reference twist. Only identically zero derivatives disappear
// at build time; no state-dependent derivative is dropped or approximated.
let compiledModule;
const OUTPUT_INDEX = 64;
const dot = (a,b) => a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
function finiteVector(values,length,name) {
    if(!values||values.length!==length)throw new TypeError(`${name} must contain ${length} finite values`);
    for(let i=0;i<length;i++)if(!Number.isFinite(values[i]))throw new TypeError(`${name} must contain ${length} finite values`);
}

export function createCompositeElementWorkspace(toolCount=2) {
    if(!Number.isInteger(toolCount)||toolCount<1||toolCount>2)throw new RangeError('An element has one or two tools');
    compiledModule??=new WebAssembly.Module(compositeElementFastKernelBytes);
    const kernel=new WebAssembly.Instance(compiledModule,{math:{sin:Math.sin,cos:Math.cos,atan2:Math.atan2,round:Math.round}}).exports;
    const buffer=kernel.memory.buffer,n=9+2*toolCount;
    const view=(index,count)=>new Float64Array(buffer,8*(OUTPUT_INDEX+index),count);
    return {toolCount,dofCount:n,kernel,input:new Float64Array(buffer,0,56),
        output:view(0,277),energy:0,toolEnergy:view(1,toolCount),strain:view(3,3*toolCount),
        moments:view(9,3*toolCount),jacobian:view(15,3*toolCount*n),
        gradient:view(93,n),hessian:view(106,n*n),referenceTwists:view(275,toolCount)};
}

/** All outputs borrow a stable single-page workspace. Input frames remain
 * frozen, material spins remain independent, and the Hessian is exactly the
 * same PSD Gauss-Newton tangent J^T K J as the JavaScript oracle. */
export function evaluateCompositeElement({positions,reference,referenceLength,tools,referenceTwist:twistAnchor=0},
    workspace=createCompositeElementWorkspace(tools.length)) {
    if(!Number.isFinite(referenceLength)||referenceLength<=0)throw new RangeError('referenceLength must be positive');
    if(positions.length!==3||reference.length!==2||tools.length!==workspace.toolCount)
        throw new RangeError('A hinge needs three points, two reference frames, and the workspace tool count');
    const input=workspace.input;
    for(let i=0;i<3;i++) {
        finiteVector(positions[i],3,'position');
        for(let j=0;j<3;j++)input[3*i+j]=positions[i][j];
    }
    for(let i=0;i<2;i++) {
        const {tangent,director}=reference[i];
        finiteVector(tangent,3,'reference tangent');finiteVector(director,3,'reference director');
        if(Math.abs(dot(tangent,tangent)-1)>1e-10||Math.abs(dot(director,director)-1)>1e-10||Math.abs(dot(tangent,director))>1e-10)
            throw new RangeError('Reference frame must be orthonormal');
        for(let j=0;j<3;j++) {input[9+6*i+j]=tangent[j];input[12+6*i+j]=director[j];}
    }
    input[21]=referenceLength;
    for(let i=0;i<tools.length;i++) {
        const tool=tools[i],dsDx=tool.dsDx??1,material=tool.material,base=22+17*i;
        const anchor=tool.referenceTwist??twistAnchor;
        if(!Number.isFinite(anchor))throw new TypeError('referenceTwist anchor must be finite');
        if(!Number.isFinite(dsDx)||dsDx<=0||material?.stiffness?.length!==9||material?.intrinsic?.length!==3)
            throw new TypeError('A positive dsDx and compiled material are required');
        finiteVector(tool.angles,2,'angles');
        input[base]=tool.angles[0];input[base+1]=tool.angles[1];input[base+2]=dsDx;
        for(let j=0;j<9;j++)input[base+3+j]=material.stiffness[j];
        for(let j=0;j<3;j++)input[base+12+j]=material.intrinsic[j];
        input[base+15]=material.energyOffset??0;input[base+16]=anchor;
    }
    const status=workspace.kernel[`evaluate${tools.length}`](0,OUTPUT_INDEX*8);
    if(status===1)throw new RangeError('A centerline edge cannot collapse');
    if(status===2)throw new RangeError('Antiparallel tangents require refinement/rebasing');
    if(status===3)throw new RangeError('A reversed hinge requires refinement');
    workspace.energy=workspace.output[0];
    if(!Number.isFinite(workspace.energy)||[workspace.gradient,workspace.hessian,workspace.strain,
        workspace.jacobian,workspace.moments,workspace.toolEnergy,workspace.referenceTwists]
        .some(values=>!values.every(Number.isFinite)))throw new RangeError('Nonfinite composite response');
    return workspace;
}
