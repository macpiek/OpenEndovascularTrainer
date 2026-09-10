import {createCompositeChainLayout} from './kirchhoffCompositeChain.js';
import {createCompositeContinuousGeometry} from './kirchhoffCompositeContinuousGeometry.js';
import {compositeContinuousFrameSupport} from './kirchhoffCompositeContinuousFrame.js';
const ownedGeometries=new WeakMap();

/** Own a serializable continuous-inertia chart for both physical materials.
 * Material starts/tips are physical boundaries, so interpolation cannot borrow
 * inactive nodes past them. Additional interfaces are explicitly supplied.
 * This reserves the common position band; it does not change the native rod
 * elasticity, length constraints or contact geometry of its caller.
 */
export function ownCompositeJointInertiaGeometry({layout,coordinates,inertiaGeometryByTool=null,elasticityGeometry='native-discrete-rod'}) {
    if(!['native-discrete-rod','continuous-material-frame'].includes(elasticityGeometry))throw new RangeError('Unknown elastic geometry');
    if(inertiaGeometryByTool===null){
        if(elasticityGeometry!=='native-discrete-rod')throw new RangeError('Continuous elasticity requires the same explicit continuous inertia geometry');
        return {layout:createCompositeChainLayout(layout.edgeToolIds,{positionSupports:layout.positionSupports??[],materialSupports:layout.materialSupports??[]}),inertiaGeometryByTool:null,elasticityGeometry};
    }
    const ids=[...layout.spins.keys()],n=layout.nodeCount;
    if(!(inertiaGeometryByTool instanceof Map)||inertiaGeometryByTool.size!==ids.length||ids.some(id=>!inertiaGeometryByTool.has(id)))
        throw new RangeError('Continuous inertia requires an explicit geometry for every material');
    const owned=new Map(),supports=new Map(),materialSupports=[];
    for(const id of ids) {
        const input=inertiaGeometryByTool.get(id);
        if(!input||!Array.isArray(input.interfaces??[])||new Set(input.interfaces??[]).size!==(input.interfaces??[]).length)
            throw new RangeError('Explicit distinct physical geometry interfaces are required');
        if(input.coordinates!==undefined&&(input.coordinates.length!==coordinates.length||input.coordinates.some((x,j)=>x!==coordinates[j])))
            throw new RangeError('Continuous inertia geometry must share the exact common coordinates');
        const active=layout.edgeToolIds.flatMap((tools,e)=>tools.includes(id)?[e]:[]),first=active[0],end=active.at(-1)+1,
            interfaces=[...new Set([...(input.interfaces??[]),...([first,end].filter(j=>j>0&&j<n-1))])],cached=ownedGeometries.get(input),
            geometry=cached?.first===first&&cached.end===end?input:createCompositeContinuousGeometry({coordinates,interfaces});
        if(elasticityGeometry==='continuous-material-frame'&&geometry.interfaces.some(node=>node>first&&node<end))
            throw new RangeError('Continuous elasticity needs orientation-continuous interface coupling; independent geometry traces are not rod joints');
        // Only reuse recursively frozen geometry created here. A serialized
        // clone or caller-owned lookalike is rebuilt, never trusted as a plan.
        ownedGeometries.set(geometry,{first,end});
        for(const e of active) {
            const nodes=geometry.edges[e].nodeIndices;
            if(nodes.some(node=>node<first||node>end))throw new RangeError('Continuous inertia cannot borrow inactive material nodes');
            supports.set(nodes.join(','),nodes.slice());
            if(elasticityGeometry==='continuous-material-frame'){
                const support=compositeContinuousFrameSupport(geometry,e);
                materialSupports.push({toolId:id,edge:e,positionNodes:support.positionIndices,angleEdges:support.angleIndices});
            }
        }
        owned.set(id,geometry);
    }
    return {layout:createCompositeChainLayout(layout.edgeToolIds,{positionSupports:[...supports.values()],materialSupports}),inertiaGeometryByTool:owned,elasticityGeometry};
}
