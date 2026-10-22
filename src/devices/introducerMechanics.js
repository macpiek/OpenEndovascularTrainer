import {introducerDimensions, INTRODUCER_LENGTHS_MM} from './introducerSheath.js';
import {sampleSheathPath, sheathLength} from '../physics/sheathPath.js';

/** Stable access geometry is separate from the moving elastic implant. */
export function initializeElasticIntroducer(sheath) {
    const entryLength=sheath.entryLength??70;
    sheath.shortPath??=[{...sheath.start,s:0},{...sheath.end,s:entryLength}];
    sheath.length??=70;
    // Imported oblique endpoints can measure 70.00000000000003 mm.
    sheath.insertion=Math.max(0,Math.min(sheath.length,sheath.insertion??sheathLength(sheath)));
    Object.assign(sheath,introducerDimensions(sheath.french??6),{entryLength});
}

// Only the skin/access tract is a prescribed sleeve. The intravascular shaft
// participates in the shared bending energy and has its own vessel contacts.
export function introducerAccessBoundary(sheath,minimumRadius=0) {
    const path=sheath.shortPath;
    return {start:path[0],end:path.at(-1),path,entryLength:sheath.entryLength,
        innerRadius:Math.max(sheath.radius,minimumRadius)+.1,proximalExtension:40};
}

export function elasticIntroducerTool(sheath,body) {
    // Hollow tube second moment, relative to the 6F preset (0.1 mm wall).
    const reference=introducerDimensions(6);
    const scale=(sheath.radius**4-sheath.innerRadius**4)/(reference.radius**4-reference.innerRadius**4);
    body.radius=sheath.radius;
    return {id:'sheath',body,type:'introducer-sheath',length:sheath.length,
        insertion:sheath.insertion,rotation:0,shaftStiffness:scale,tipStiffness:scale,
        nodeCoordinates:Array.from({length:body.count},(_,i)=>
            -40+i/(body.count-1)*(sheath.insertion+40))};
}

/** Prepare feed only. Geometry is published exclusively after solver acceptance. */
export function prepareElasticIntroducer(sheath,{delta=0,selection=null,catheterProgress=0,catheterRadius=5/6}={}) {
    let length=sheath.length,french=sheath.french;
    if(selection) {
        if(sheath.insertion!==0)throw new Error('Całkowicie wycofaj koszulkę, aby zmienić jej długość lub średnicę.');
        ({length,french}=selection);
        if(!INTRODUCER_LENGTHS_MM.includes(length))throw new RangeError('Nieznana długość koszulki');
    }
    const dimensions=introducerDimensions(french);
    if(catheterProgress>0&&catheterRadius>dimensions.innerRadius+1e-9)
        throw new Error('Cewnik nie mieści się w wybranej koszulce. Wybierz większą średnicę.');
    const insertion=selection?0:Math.max(0,Math.min(length,sheath.insertion+delta));
    if(insertion===sheath.insertion&&length===sheath.length&&french===sheath.french)return false;
    Object.assign(sheath,dimensions,{length,insertion});
    return true;
}

/** The mechanical node view contains the whole deformable shaft, not just its tip. */
export function publishElasticIntroducer(sheath,body) {
    const view=body.jointStateView;
    if(!view)return false;
    const path=Array.from(view.coordinates,(s,i)=>({s,x:view.positions[3*i],y:view.positions[3*i+1],z:view.positions[3*i+2]}));
    const old=sheath.path;
    const changed=!old||old.length!==path.length||path.some((p,i)=>
        Math.abs(p.s-old[i].s)>1e-8||Math.hypot(p.x-old[i].x,p.y-old[i].y,p.z-old[i].z)>1e-7);
    if(!changed)return false;
    sheath.path=path;
    sheath.start={...sampleSheathPath(sheath,0)};
    sheath.end={...path.at(-1)};
    return true;
}
