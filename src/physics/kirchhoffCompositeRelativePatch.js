import {createCompositeElementWorkspace,evaluateCompositeElement} from './kirchhoffCompositeElementFast.js';
import {solveCompositeClearanceCell,condenseCompositeClearanceCell} from './kirchhoffCompositeClearance.js';

/** One hierarchical transverse node mode on the existing common mesh.
 * ALL incident wire hinges contribute, including their complete position/
 * spin derivatives. Its support covers up to five nodes, not the full rod.
 * basis is frozen during this local linearization. Surface feet, nonlinear
 * re-evaluation, adjacent-mode coupling and reduction admission belong to
 * the caller. This function does not certify a common-axis approximation.
 * relativeGradient/Stiffness describe additional relative terms. Physical
 * inertia of q+B*rho ALSO requires its common-relative Hessian, supplied as
 * additionalCoupling rows {dof,values:[C0,C1]} in the global Chain layout.
 * Its base common energy/gradient/Hessian must already be assembled in Chain.
 */
export function assembleCompositeRelativePatch({data,layout,node,toolId='wire',basis,
    relativeGradient=[0,0],relativeStiffness=[0,0,0],additionalCoupling=[]}) {
    if(!Number.isInteger(node)||node<=0||node>=layout.nodeCount-1) throw new RangeError('An interior mesh node is required');
    if(!basis || basis.length!==2 || basis.some(v=>v.length!==3||!v.every(Number.isFinite)))
        throw new TypeError('Two finite world-space transverse basis vectors are required');
    const dot=(a,b)=>a.reduce((sum,x,i)=>sum+x*b[i],0);
    if(Math.abs(dot(basis[0],basis[0])-1)>1e-10||Math.abs(dot(basis[1],basis[1])-1)>1e-10||Math.abs(dot(basis[0],basis[1]))>1e-10)
        throw new RangeError('The transverse basis must be orthonormal');
    const chord=data.positions[node+1].map((v,i)=>v-data.positions[node-1][i]),length=Math.hypot(...chord);
    if(!(length>0)||!Number.isFinite(length)||basis.some(v=>Math.abs(dot(v,chord)/length)>1e-10))
        throw new RangeError('The local basis must be transverse to the centered catheter chord');
    for(const [values,n] of [[relativeGradient,2],[relativeStiffness,3]])
        if(values.length!==n||!values.every(Number.isFinite)) throw new TypeError('Finite relative force and stiffness are required');
    const tool=data.tools.find(t=>t.id===toolId),spin=layout.spins.get(toolId);
    if(!tool||!spin||spin[node-1]<0||spin[node]<0) throw new RangeError('The internal mode requires material on both sides');
    const hinges=layout.hinges.filter(h=>h.tools.includes(toolId)&&Math.abs(h.vertex-node)<=1);
    const elements=hinges.map(({vertex:i})=>({vertex:i,dofs:[i-1,i,i+1].flatMap(j=>[layout.positions[j],layout.positions[j]+1,layout.positions[j]+2])
        .concat([spin[i-1],spin[i]])}));
    const dofs=Int32Array.from([...new Set(elements.flatMap(e=>e.dofs))].sort((a,b)=>a-b));
    const localIndex=new Map([...dofs].map((d,i)=>[d,i])),n=dofs.length;
    const gradient=new Float64Array(n),hessian=new Float64Array(n*n),coupling=new Float64Array(2*n);
    if(!Array.isArray(additionalCoupling)) throw new TypeError('Additional coupling must be a list of local Chain rows');
    for(const {dof,values} of additionalCoupling) {
        if(!localIndex.has(dof)) throw new RangeError('Additional coupling must stay inside the local patch support');
        if(!values||values.length!==2||!values.every(Number.isFinite)) throw new TypeError('Two finite common-relative coefficients are required');
        const index=2*localIndex.get(dof);coupling[index]+=values[0];coupling[index+1]+=values[1];
    }
    const modeGradient=Float64Array.from(relativeGradient),modeStiffness=Float64Array.from(relativeStiffness);
    const workspace=createCompositeElementWorkspace(1);
    let energy=0;
    for(const element of elements) {
        const i=element.vertex,coordinates=data.coordinates;
        const material=tool.materialAt?tool.materialAt({vertex:i,coordinate:coordinates[i],
            start:(coordinates[i-1]+coordinates[i])/2,end:(coordinates[i]+coordinates[i+1])/2}):tool.material;
        const response=evaluateCompositeElement({positions:[data.positions[i-1],data.positions[i],data.positions[i+1]],
            reference:[data.reference[i-1],data.reference[i]],referenceLength:(coordinates[i+1]-coordinates[i-1])/2,
            tools:[{angles:[tool.angles[i-1],tool.angles[i]],referenceTwist:tool.referenceTwists?.[i-1]??0,
                dsDx:typeof tool.dsDx==='function'?tool.dsDx(coordinates[i]):tool.dsDx??1,material}]},workspace);
        const at=3*(node-(i-1)),H=response.hessian;
        energy+=response.energy;
        for(let row=0;row<11;row++) {
            const r=localIndex.get(element.dofs[row]);gradient[r]+=response.gradient[row];
            for(let col=0;col<11;col++) hessian[r*n+localIndex.get(element.dofs[col])]+=H[11*row+col];
            for(let axis=0;axis<2;axis++) for(let k=0;k<3;k++) coupling[2*r+axis]+=H[11*row+at+k]*basis[axis][k];
        }
        for(let axis=0;axis<2;axis++) for(let k=0;k<3;k++) modeGradient[axis]+=response.gradient[at+k]*basis[axis][k];
        for(let a=0;a<2;a++) for(let b=a;b<2;b++) for(let j=0;j<3;j++) for(let k=0;k<3;k++)
            modeStiffness[a===0?b:2]+=basis[a][j]*H[(at+j)*11+at+k]*basis[b][k];
    }
    if(!Number.isFinite(energy)||[gradient,hessian,coupling,modeGradient,modeStiffness].some(v=>!v.every(Number.isFinite)))
        throw new RangeError('Nonfinite relative patch operator');
    return {node,toolId,dofs,energy,gradient,hessian,coupling,modeGradient,modeStiffness,
        affectedHinges:elements.map(e=>e.vertex),scope:'common-axis-local-Gauss-Newton-patch',
        certified:false,eliminatedDofCount:2};
}

/** Condensed corrections are ADDITIONS to an already assembled common chain;
 * never add patch.energy/gradient/hessian again (they are already in Chain).
 * An active patch can widen its LOCAL band up to its five-node support.
 * Patches with intersecting energy supports require a coupled local solve.
 */
export function solveCompositeRelativePatch({patch,clearance,...options}) {
    const cell=solveCompositeClearanceCell({stiffness:patch.modeStiffness,gradient:patch.modeGradient,clearance,...options});
    if(!cell.converged) return {cell,converged:false,certified:false};
    const correction=condenseCompositeClearanceCell({cell,coupling:patch.coupling,commonDofCount:patch.dofs.length});
    return {cell,correction,dofs:patch.dofs,converged:true,certified:false,
        scope:'local-quadratic-correction-only'};
}
