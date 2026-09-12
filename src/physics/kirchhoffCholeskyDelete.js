/** Delete one row/column of A=L L^T from a positive Cholesky factor.
 * Input uses lower diagonal-first band storage; output uses dense lower
 * diagonal-first storage with stride count-1. The retained subfactor receives
 * the exact rank-one contribution of the deleted column. No shift is added.
 * Temporary storage is reused; input/output are untouched on failure.
 */
export function deleteKirchhoffCholeskyRow(factor,count,band,index,out,workspace={}) {
    if(!Number.isInteger(count)||count<1||!Number.isInteger(band)||band<1||band>count||
        !Number.isInteger(index)||index<0||index>=count||!(factor instanceof Float64Array)||
        factor.length<count*band||!(out instanceof Float64Array)||out.length<(count-1)*(count-1))return false;
    const size=count-1,entries=size*size;
    if(!workspace.factor||workspace.factor.length<entries)workspace.factor=new Float64Array(entries);
    if(!workspace.vector||workspace.vector.length<size)workspace.vector=new Float64Array(size);
    const target=workspace.factor,vector=workspace.vector;
    // Scratch must never alias either public input/output: failed arithmetic
    // must leave the caller's previous usable factor intact.
    if(target.buffer===factor.buffer||target.buffer===out.buffer||vector.buffer===factor.buffer||vector.buffer===out.buffer||target.buffer===vector.buffer)return false;
    for(let i=0;i<count;i++){
        if(!(factor[i*band]>0)||!Number.isFinite(factor[i*band]))return false;
        for(let j=Math.max(0,i-band+1);j<i;j++)if(!Number.isFinite(factor[i*band+i-j]))return false;
    }
    target.fill(0,0,entries);vector.fill(0,0,size);
    for(let i=0;i<size;i++){
        const oldI=i<index?i:i+1;
        for(let j=0;j<=i;j++){
            const oldJ=j<index?j:j+1,distance=oldI-oldJ;
            target[i*size+i-j]=distance<band?factor[oldI*band+distance]:0;
        }
        if(i>=index&&oldI-index<band)vector[i]=factor[oldI*band+oldI-index];
    }
    for(let i=index;i<size;i++){
        const diagonal=target[i*size],value=vector[i],r=Math.hypot(diagonal,value);
        if(!(r>0)||!Number.isFinite(r))return false;
        const c=diagonal/r,s=value/r;target[i*size]=r;
        for(let j=i+1;j<size;j++){
            const offset=j*size+j-i,old=target[offset],x=vector[j];
            const next=c*old+s*x,remainder=c*x-s*old;
            if(!Number.isFinite(next)||!Number.isFinite(remainder))return false;
            target[offset]=next;vector[j]=remainder;
        }
    }
    for(let i=0;i<entries;i++)out[i]=target[i];
    return true;
}
