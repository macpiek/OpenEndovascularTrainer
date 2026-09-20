/** Replay identity, not a lumen classifier. Hash exact indexed mesh bytes so a
 * certificate cannot silently migrate to a different anatomy or transform.
 * Computed only when capturing/restoring debug data, outside the step loop. */
export function sharedAxisGeometryIdentity(geometry) {
    const position=geometry?.getAttribute?.('position');
    if(!position?.array)return null;
    let a=2166136261,b=0x9e3779b9;
    const arrays=[position.array,geometry.index?.array];
    for(const array of arrays)if(array) {
        const bytes=new Uint8Array(array.buffer,array.byteOffset,array.byteLength);
        for(const value of bytes){a=Math.imul(a^value,16777619);b=Math.imul(b^value,2246822519);b=(b<<13)|(b>>>19);}
    }
    return `mesh-v1/${position.itemSize}/${arrays.map(x=>x?`${x.constructor.name}:${x.length}`:'none').join('/')}/${(a>>>0).toString(16)}/${(b>>>0).toString(16)}`;
}
