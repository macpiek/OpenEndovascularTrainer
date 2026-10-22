import * as THREE from 'three';
import {renalSphereLevelBound} from './renalShape.js';

// Fixed, different vascular plans for each side. Keep the anatomy reproducible
// across the mesh build, collision field and runtime contrast network.
const RENAL_PLANS = {
    left: {seed:371, levels:[-31,-15,-1,17,30], fans:[[2,1],[1,2,2],[3,1],[2,2],[2]]},
    right:{seed:829, levels:[-29,-18,3,14,32], fans:[[1,2],[2,2],[3],[1,2,1],[2,3]]}
};

// Distal renal outlets in the supported anatomy (millimetres, native space).
// Match by position and terminal topology, never by asset-specific node IDs.
const RENAL_OUTLETS = [
    {side:'left', sign:1, target:[27.49,-194.19,-6.70], outlet:[38,-197,-8], radius:3.1},
    {side:'right', sign:-1, target:[-12.73,-189.20,-8.18], outlet:[-23.5,-190,-13.7], radius:3.7}
];

/** Shared geometry for the offline hollow vascular mesh and iodine transport. */
export function extendRenalArteries(sourceSegments) {
    const nodes = new Map();
    for (const segment of sourceSegments) for (const end of ['Start','End']) {
        const id=segment[`node${end}Id`], point=segment[end.toLowerCase()];
        const node=nodes.get(id) ?? {id,point,degree:0};
        node.degree++; nodes.set(id,node);
    }
    if(sourceSegments.some(s=>s.renalSide)) {
        const kidneys=RENAL_OUTLETS.map(({side,sign,target})=>({side,sign,center:new THREE.Vector3(sign*74,target[1]+3,-16),
            terminalNodes:sourceSegments.filter(s=>s.renalSide===side&&nodes.get(s.nodeEndId).degree===1).map(s=>({id:s.nodeEndId,sector:s.renalSector})),
            segmentIds:sourceSegments.filter(s=>s.renalSide===side).map(s=>s.id)}));
        return {segments:sourceSegments,kidneys};
    }
    let nextNode=Math.max(-1,...nodes.keys())+1;
    let nextId=Math.max(-1,...sourceSegments.map(s=>Number(s.id)||0))+1;
    const segments=sourceSegments.slice(), kidneys=[];
    for (const definition of RENAL_OUTLETS) {
        const target=new THREE.Vector3(...definition.target);
        const root=[...nodes.values()].find(n=>n.degree===1 && Math.hypot(n.point.x-target.x,n.point.y-target.y,n.point.z-target.z)<2);
        if (!root) continue; // Fixture/toy networks have no renal bed.
        const {side,sign}=definition;
        const center=new THREE.Vector3(sign*74,root.point.y+3,-16);
        const kidney={side,sign,center,terminalNodes:[],segmentIds:[]};
        kidneys.push(kidney);
        const plan=RENAL_PLANS[side];
        const lateralSources=[];
        let seed=plan.seed;
        const variation=amplitude=>{
            seed=(Math.imul(seed,1664525)+1013904223)>>>0;
            return (seed/4294967296*2-1)*amplitude;
        };
        const inside=(x,y,z,radius)=>{
            const point=new THREE.Vector3(sign*x,center.y+y,center.z+z);
            // Reserve clearance for the real mesh wall, not just the axis.
            while(renalSphereLevelBound(point,radius+.5,kidney)>.73)
                point.lerp(center,.04);
            return point;
        };
        const average=points=>points.reduce((sum,p)=>sum.add(p),new THREE.Vector3()).multiplyScalar(1/points.length);
        const toward=(parent,points,fraction,radius)=>{
            const point=parent.point.clone().lerp(average(points),fraction);
            return inside(point.x*sign+variation(.7),point.y-center.y+variation(.9),
                point.z-center.z+variation(.8),radius);
        };
        function branch(parent,end,endTangent,r0,r1,sector=-1,departure=0.36,hilar=false,lateral=false) {
            const firstSegment=segments.length;
            const {point:start,id,tangent}=parent;
            const handle=start.distanceTo(end)*0.36;
            // Both daughters leave a junction along the parent's tangent. The
            // turn develops over the branch, rather than at a polygonal elbow.
            const curve=new THREE.CubicBezierCurve3(start,
                start.clone().addScaledVector(tangent,start.distanceTo(end)*departure),
                end.clone().addScaledVector(endTangent,-handle),end);
            let steps=Math.max(16,Math.ceil(curve.getLength()/(r0<.5?.22:.5)));
            let points=curve.getSpacedPoints(steps);
            // Resolve tight small-vessel curves without polygonal elbows.
            for(let pass=0;pass<5;pass++){
                let previousDirection=tangent,largest=0;
                for(let i=1;i<points.length;i++){
                    const direction=points[i].clone().sub(points[i-1]).normalize();
                    largest=Math.max(largest,previousDirection.angleTo(direction));
                    previousDirection=direction;
                }
                if(largest<10*Math.PI/180)break;
                steps*=2;points=curve.getSpacedPoints(steps);
            }
            const radius=t=>{
                // A side ostium joins at the parent's calibre, then quickly
                // narrows to a fine branch instead of creating another trunk.
                // Finish narrowing close to the ostium, mostly inside the
                // parent lumen, rather than carrying its width into the tissue.
                const u=lateral?Math.min(1,t*curve.getLength()/1.25):t;
                return THREE.MathUtils.lerp(r0,r1,u*u*(3-2*u));
            };
            let previous=start, previousId=id;
            for(let i=1;i<=steps;i++) {
                const point=points[i],nodeId=nextNode++;
                const axis=point.clone().sub(previous),length=axis.length(); axis.normalize();
                const a=radius((i-1)/steps),b=radius(i/steps);
                const segment={id:nextId++,start:previous.clone(),end:point.clone(),axis,length,
                    radiusStart:a,radiusEnd:b,safeRadius:Math.min(a,b),nodeStartId:previousId,nodeEndId:nodeId,
                    renalSide:side,renalSector:sector,renalHilar:hilar,renalLateral:lateral,source:'renal-perfusion'};
                segments.push(segment);kidney.segmentIds.push(segment.id);
                previous=point;previousId=nodeId;
            }
            return {point:end,id:previousId,tangent:endTangent,path:segments.slice(firstSegment)};
        }
        // Match the native outlet tangent and calibre before the hilar fan.
        // Shared junction radii avoid a step in the hollow lumen.
        const incoming=sourceSegments.find(s=>s.nodeEndId===root.id);
        const tangent=incoming?incoming.end.clone().sub(incoming.start).normalize():new THREE.Vector3(sign,0,0);
        const outward=new THREE.Vector3(sign,0,0);
        // The atlas centerline stops inside its capped distal stump. Sweep
        // through the true mesh outlet before turning toward the hilum; a
        // direct shortcut would leave an angular blind pouch beside the join.
        const outlet=new THREE.Vector3(...definition.outlet);
        const outletTangent=outlet.clone().sub(root.point).normalize();
        const patentOutlet=branch({...root,tangent},outlet,outletTangent,definition.radius,definition.radius);
        // Divide before the parenchymal outline. Separate hilar branches then
        // enter the kidney, instead of carrying one trunk into the tissue.
        const hilum=new THREE.Vector3(sign*46,center.y,center.z);
        const trunk=branch(patentOutlet,hilum,outward,definition.radius,2.2);
        // After the extrarenal hilar fan, keep the smaller branches and their
        // walls inside the tissue, with room near the poles and indentation.
        for(let sector=0;sector<5;sector++) {
            const y=plan.levels[sector],z=variation(4);
            const r1=1.05+variation(.14);
            // Allocate irregular territories along the entire cortical arc,
            // including the medial sides of both poles. Grow the tree toward
            // these territories instead of repeating short lateral tufts.
            const boundaries=[-2.18,-1.12,-.36,.35,1.10,2.18];
            const surfaceFans=plan.fans[sector],fans=[...surfaceFans,2];
            const twigCount=surfaceFans.reduce((a,b)=>a+b,0);
            let twigIndex=0;
            const territories=fans.map((count,fan)=>Array.from({length:count},(_,twig)=>{
                const deep=fan===surfaceFans.length;
                const interval=boundaries[sector+1]-boundaries[sector];
                const start=boundaries[sector]+interval*(deep?twig/count:twigIndex++/twigCount);
                const width=interval/(deep?count:twigCount);
                const daughterCount=variation(1)>.55?3:2;
                const counts=Array.from({length:daughterCount},()=>variation(1)>.15?3:2);
                const total=counts.reduce((a,b)=>a+b,0);
                let leafIndex=0;
                return counts.map(leafCount=>Array.from({length:leafCount},()=>{
                    const angle=start+width*(leafIndex+++.5+variation(.34))/total;
                    // Anterior/posterior cortical branches project between the
                    // peripheral fans rather than leaving empty interior wedges.
                    const reach=deep?.60+variation(.11):.83+variation(.065);
                    const depth=deep?((sector%2===0?1:-1)*sign*(13+variation(2))):variation(8)*(.7+.3*Math.cos(angle));
                    return inside(74+35*reach*Math.cos(angle),55*reach*Math.sin(angle),
                        depth,.28);
                }));
            }));
            // The proximal fan is extrarenal; its distal ends and all smaller
            // branches remain within the parenchyma.
            const segmental=inside(70-.18*Math.abs(y)+variation(1.5),y,z,2.2);
            const sectorDirection=average(territories.flat(3)).sub(segmental).normalize();
            const first=branch(trunk,segmental,sectorDirection,2.2,r1,sector,.20,true);
            lateralSources.push({path:first.path,sector,count:3});
            for(let fan=0;fan<fans.length;fan++) {
                const interlobar=toward(first,territories[fan].flat(2),.34+variation(.07),r1);
                const r2=.71+variation(.10);
                const second=branch(first,interlobar,interlobar.clone().sub(first.point).normalize(),r1,r2,sector);
                lateralSources.push({path:second.path,sector,count:2});
                for(let twig=0;twig<fans[fan];twig++) {
                    const territory=territories[fan][twig];
                    const end=toward(second,territory.flat(),.42+variation(.10),r2);
                    const r3=.40+variation(.05);
                    const arcuate=branch(second,end,end.clone().sub(second.point).normalize(),r2,r3,sector);
                    // Two further generations distribute fine cortical vessels
                    // across each territory, including anterior/posterior depth.
                    // These are connected lumens, shared by transport and tools.
                    for(const leaves of territory){
                        const smallEnd=toward(arcuate,leaves,.48+variation(.12),r3);
                        const r4=.24+variation(.035);
                        const small=branch(arcuate,smallEnd,smallEnd.clone().sub(end).normalize(),r3,r4,sector);
                        for(const leafEnd of leaves){
                            const terminal=branch(small,leafEnd,leafEnd.clone().sub(smallEnd).normalize(),
                                r4,.13+variation(.025),sector);
                            kidney.terminalNodes.push({id:terminal.id,sector});
                        }
                    }
                }
            }
        }
        // Attach side branches to existing interior graph nodes, leaving the
        // parent artery continuous beyond each takeoff. Never attach a visual
        // tube to the middle of an edge without a matching flow/collision node.
        for(const {path,sector,count} of lateralSources){
            const candidates=path.slice(2,-3).filter(s=>
                s.end.distanceTo(path[0].start)>2&&s.end.distanceTo(path.at(-1).end)>1.5&&
                renalSphereLevelBound(s.end,s.radiusEnd+.5,kidney)<.72);
            let previousSite=null;
            const sites=Array.from({length:count+(variation(1)>.45?1:0)},()=>.08+(variation(1)+1)*.44).sort((a,b)=>a-b);
            for(const position of sites){
                if(!candidates.length)break;
                const index=Math.min(candidates.length-1,Math.floor(candidates.length*position));
                const source=candidates[index];
                if(previousSite&&source.end.distanceTo(previousSite)<1.6)continue;
                const parent={point:source.end,id:source.nodeEndId,tangent:source.axis};
                const normal=new THREE.Vector3(-source.axis.y,source.axis.x,0).normalize();
                const binormal=new THREE.Vector3().crossVectors(source.axis,normal).normalize();
                // Independent azimuths and oblique courses avoid alternating,
                // coplanar crossbars between neighbouring segmental arteries.
                const azimuth=variation(Math.PI);
                const spreadDirection=normal.clone().multiplyScalar(Math.cos(azimuth))
                    .addScaledVector(binormal,Math.sin(azimuth));
                const target=source.end.clone().addScaledVector(source.axis,6.8+variation(2.8))
                    .addScaledVector(spreadDirection,3.4+variation(1.7));
                const clipToTissue=(start,target,radius)=>{
                    let lo=0,hi=1;
                    for(let i=0;i<20;i++){
                        const mid=(lo+hi)/2;
                        if(renalSphereLevelBound(start.clone().lerp(target,mid),radius+.5,kidney)<.78)lo=mid;
                        else hi=mid;
                    }
                    return start.clone().lerp(target,lo);
                };
                const radius=.205+variation(.035);
                const end=clipToTissue(source.end,target,radius);
                const direction=end.clone().sub(source.end);
                if(direction.length()<3||direction.clone().normalize().dot(source.axis)<.15)continue;
                previousSite=source.end;
                const stem=branch(parent,end,direction.normalize(),source.radiusEnd,radius,sector,.38,false,true);
                const leaves=variation(1)>.65?3:2;
                let addedLeaves=0;
                for(let leaf=0;leaf<leaves;leaf++){
                    // Unequal daughter lengths and independent directions form
                    // an asymmetric fine spray, rather than repeated tridents.
                    const spread=variation(2.1)+(leaf===0?-.65:.65);
                    const target=end.clone().addScaledVector(stem.tangent,4.2+variation(2.2))
                        .addScaledVector(spreadDirection,spread)
                        .addScaledVector(binormal,variation(1.6));
                    // Stop at the capsule along the intended course. Pulling a
                    // near-polar endpoint toward the center can fold a tiny
                    // branch back onto itself and pinch its swept lumen.
                    const leafEnd=clipToTissue(end,target,radius);
                    if(leafEnd.distanceTo(end)<.6)continue;
                    const terminal=branch(stem,leafEnd,leafEnd.clone().sub(end).normalize(),
                        radius,.13+variation(.025),sector,.36,false,true);
                    kidney.terminalNodes.push({id:terminal.id,sector});
                    addedLeaves++;
                }
                if(!addedLeaves)kidney.terminalNodes.push({id:stem.id,sector});
            }
        }
    }
    return {segments,kidneys};
}
