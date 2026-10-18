const identities=new WeakMap();

// Check every byte, including edits that did not bump BufferAttribute.version.
// Comparing aligned words is cheaper than repeating two serial hash chains for
// every byte of the large, usually unchanged anatomy during recovered steps.
function sameBytes(array,saved) {
    if(!array)return saved===null;
    if(!saved||array.byteLength!==saved.byteLength)return false;
    const bytes=new Uint8Array(array.buffer,array.byteOffset,array.byteLength);
    let end=0;
    if((array.byteOffset&3)===0) {
        const count=bytes.length>>>2,words=new Uint32Array(array.buffer,array.byteOffset,count);
        const previous=new Uint32Array(saved.buffer,0,count);
        for(let i=0;i<count;i++)if(words[i]!==previous[i])return false;
        end=count*4;
    }
    for(let i=end;i<bytes.length;i++)if(bytes[i]!==saved[i])return false;
    return true;
}

/** Replay identity, not a lumen classifier. Hash exact indexed mesh bytes so a
 * certificate cannot silently migrate to a different anatomy or transform.
 * Computed when capturing/restoring debug data, including recovered steps.
 * Read the bytes each time: direct attribute edits need not bump a version.
 * The weakly owned copy is metadata only and never used for collision queries. */
export function sharedAxisGeometryIdentity(geometry) {
    const position=geometry?.getAttribute?.('position');
    if(!position?.array)return null;
    let a=2166136261,b=0x9e3779b9;
    const arrays=[position.array,geometry.index?.array];
    const layout=`mesh-v1/${position.itemSize}/${arrays.map(x=>x?`${x.constructor.name}:${x.length}`:'none').join('/')}`;
    const saved=identities.get(geometry);
    if(saved?.layout===layout&&arrays.every((array,i)=>sameBytes(array,saved.bytes[i])))return saved.identity;
    for(const array of arrays)if(array) {
        const bytes=new Uint8Array(array.buffer,array.byteOffset,array.byteLength);
        // Indexed traversal avoids iterator overhead on large anatomical meshes
        // while preserving the exact v1 digest used by existing recordings.
        for(let i=0;i<bytes.length;i++){
            const value=bytes[i];a=Math.imul(a^value,16777619);b=Math.imul(b^value,2246822519);b=(b<<13)|(b>>>19);
        }
    }
    const identity=`${layout}/${(a>>>0).toString(16)}/${(b>>>0).toString(16)}`;
    identities.set(geometry,{layout,identity,bytes:arrays.map(array=>array?
        new Uint8Array(array.buffer,array.byteOffset,array.byteLength).slice():null)});
    return identity;
}
