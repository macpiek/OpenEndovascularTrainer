// A material-coordinate interval can expose both physical curves at a portal
// or where the tools separate. Sharing a chart edge does not make the wire's
// surface identical to the catheter's surface.
export function compositeJointWallSurfaces(entry) {
    if(entry?.walls!==undefined) {
        if(entry.wall!==undefined||!Array.isArray(entry.walls))throw new RangeError('Declare wall or walls for a Joint edge, never both');
        return entry.walls;
    }
    return entry?.wall==null?[]:[entry.wall];
}

export function compositeJointWallOwner(contactOwners,edge,owner) {
    return compositeJointWallSurfaces(contactOwners?.edges?.[edge]).find(w=>w.owner===owner);
}

export function expandCompositeJointWallOwners(contactOwners,layout) {
    if(contactOwners?.edges?.length!==layout.nodeCount-1)throw new RangeError('Wall ownership must cover actual edges');
    const records=[];
    contactOwners.edges.forEach((entry,edge)=>{
        if(entry?.edge!==edge)throw new RangeError('Wall owners must follow actual edge order');
        const seen=new Set();
        for(const wall of compositeJointWallSurfaces(entry)) {
            if(!wall||!layout.edgeToolIds[edge].includes(wall.owner)||seen.has(wall.owner))
                throw new RangeError('Each distinct wall owner must occupy its physical edge');
            if(!Number.isFinite(wall.radius)||wall.radius<=0)throw new RangeError('Owner radius must be finite and positive');
            seen.add(wall.owner);records.push({edge,wall});
        }
    });
    return records;
}
