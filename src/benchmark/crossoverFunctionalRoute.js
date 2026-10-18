import {findAortoiliacDebugAnchors} from '../anatomyDebugLabels.js';

export const CROSSOVER_ROUTE_LABELS={
    aorta:'aorta',ipsilateral:'strona dostępu',
    'contralateral-common':'przeciwległa biodrowa wspólna',
    'contralateral-internal':'przeciwległa biodrowa wewnętrzna',
    'contralateral-external':'przeciwległa biodrowa zewnętrzna / dalsze naczynia nogi',
    unknown:'położenie nieustalone'
};

function tipPoint(shape,insertion) {
    const c=shape?.coordinates,p=shape?.positions;
    if(!Number.isFinite(insertion)||!c||c.length<2||p?.length!==c.length*3||insertion<c[0]||insertion>c.at(-1)+1e-8)return null;
    let i=0;while(i<c.length-2&&c[i+1]<insertion)i++;
    const t=(insertion-c[i])/(c[i+1]-c[i]);
    const point=[0,1,2].map(k=>p[i*3+k]+t*(p[(i+1)*3+k]-p[i*3+k]));
    return point.every(Number.isFinite)?point:null;
}

/** Coarse anatomical screening, independent of the old solver's shape.
 * This labels a nearest centerline branch, NOT containment in the STL lumen.
 * Wall contacts remain the solver's separate geometry gate. The external
 * subtree includes the femoral and more distal branches; exact femoral entry,
 * pigtail seating and freedom from jitter still require visual review.
 */
export function createCrossoverRouteCheck(flow,{accessSide='right'}={}) {
    const anchors=findAortoiliacDebugAnchors(flow);
    if(!anchors||!['right','left'].includes(accessSide))throw new Error('Brak anatomii rozwidlenia dla testu crossover.');
    const edges=flow.edges,labels=new Map(),opposite=accessSide==='right'?'left':'right';
    const descendants=(root,label)=>{
        const stack=[root],visited=new Set();
        while(stack.length){const index=stack.pop();if(visited.has(index))continue;visited.add(index);
            const edge=edges[index];if(!edge)throw new Error('Niepełna topologia naczyń.');
            labels.set(index,label);stack.push(...edge.childEdgeIndices);}
    };
    let parent=edges[anchors.aortoiliacParentEdgeIndex];const ancestors=new Set();
    while(parent&&!ancestors.has(parent.index)){ancestors.add(parent.index);labels.set(parent.index,'aorta');parent=edges[parent.parentEdgeIndex];}
    descendants(anchors[`${accessSide}IliacEdgeIndex`],'ipsilateral');
    descendants(anchors[`${opposite}IliacEdgeIndex`],'contralateral-common');
    let split=edges[anchors[`${opposite}IliacEdgeIndex`]];const visited=new Set();
    while(split?.childEdgeIndices.length===1&&!visited.has(split.index)){visited.add(split.index);split=edges[split.childEdgeIndices[0]];}
    if(split?.childEdgeIndices.length!==2)throw new Error('Nie rozpoznano podziału biodrowej wspólnej.');
    // Same dominant-outflow rule as the anatomy debug labels. Do not guess
    // when the branches cannot be ordered by the resolved flow tree.
    const branches=split.childEdgeIndices.map(i=>edges[i]).sort((a,b)=>b.meanFlowMm3PerS-a.meanFlowMm3PerS);
    if(!branches.every(e=>Number.isFinite(e.meanFlowMm3PerS))||branches[0].meanFlowMm3PerS<=branches[1].meanFlowMm3PerS)
        throw new Error('Niejednoznaczny podział biodrowej zewnętrznej i wewnętrznej.');
    descendants(branches[0].index,'contralateral-external');descendants(branches[1].index,'contralateral-internal');
    function locate(shape,insertion) {
        const point=tipPoint(shape,insertion);if(!point)return {region:'unknown',point:null};
        let nearest=null;
        for(const edge of edges){
            const dx=point[0]-edge.start.x,dy=point[1]-edge.start.y,dz=point[2]-edge.start.z;
            const along=Math.max(0,Math.min(edge.length,dx*edge.axis.x+dy*edge.axis.y+dz*edge.axis.z));
            const distance=Math.hypot(dx-along*edge.axis.x,dy-along*edge.axis.y,dz-along*edge.axis.z);
            if(!nearest||distance<nearest.distanceMm)nearest={edgeIndex:edge.index,distanceMm:distance,
                radiusMm:Math.max(edge.radiusStart,edge.radiusEnd)};
        }
        // Extracted centerlines underestimate some junction radii. A small
        // localization margin is allowed only for branch labeling; it does
        // not permit penetration or replace an actual surface query.
        const region=nearest&&nearest.distanceMm<=nearest.radiusMm+2?labels.get(nearest.edgeIndex)??'unknown':'unknown';
        return {region,point,...nearest};
    }
    return {
        locate,
        checkpoint(stage,shape,tools){
            const wire=locate(shape,tools?.wire),catheter=locate(shape,tools?.catheter);
            const crossed=p=>p.region.startsWith('contralateral-');
            const passed=[()=>catheter.region==='aorta',()=>catheter.region==='contralateral-common',
                ()=>crossed(wire)&&crossed(catheter),
                ()=>wire.region==='contralateral-external'&&catheter.region==='contralateral-external',
                ()=>wire.region==='contralateral-external'&&catheter.region==='contralateral-external'][stage]?.()??false;
            return {stage,passed,wire,catheter};
        }
    };
}

export function summarizeCrossoverRoute(checkpoints,{complete=false}={}) {
    const ordered=checkpoints.length===5&&checkpoints.every((c,i)=>c.stage===i);
    return {checkpoints,routeChecksPassed:complete&&ordered&&checkpoints.every(c=>c.passed),
        fullManeuverVerified:false,visualReviewRequired:true,
        scope:'Coarse tip branch checks at five stages. Does not certify wall containment, pigtail seating, femoral entry, jitter or freedom from blocking.'};
}
