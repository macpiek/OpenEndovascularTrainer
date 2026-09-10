import {locateKirchhoffDistalLumenBranch,isKirchhoffDistalLumenWitness} from './kirchhoffToolContactOwnership.js';
import {createCompositeExternalCapsuleGeometryWorkspace,evaluateCompositeExternalCapsuleContact} from './kirchhoffCompositeExternalCapsuleGeometry.js';

const preparations=new WeakMap(),signature=JSON.stringify,clamp=(x,a,b)=>Math.max(a,Math.min(b,x)),
    fail=(message,details={})=>{const e=new RangeError(message);e.code='joint-world-tool-contact-adapter-required';e.details=details;throw e;},
    positive=(v,name)=>{if(!(v>0)||!Number.isFinite(v))fail(`${name} must be positive`);return v;},
    nonnegative=(v,name)=>{if(!(v>=0)||!Number.isFinite(v))fail(`${name} must be nonnegative`);return v;};

function snapshot(world,state,bindings) {
    if(!Array.isArray(world?.toolContacts)||!Array.isArray(bindings)||state?.elasticityGeometry!=='native-discrete-rod'||state.inertiaGeometryByTool!=null)
        fail('World external contact needs actual bodies and native affine tools');
    const byBody=new Map();for(const b of bindings){if(byBody.has(b.body)||!world.bodies.includes(b.body)||!state.layout.spins.has(b.toolId))fail('Distinct actual body bindings required');byBody.set(b.body,b);}
    const cache=new Map();
    function bodyData(body) {
        if(cache.has(body))return cache.get(body);const binding=byBody.get(body);if(!binding)fail('Every enabled external body needs an own Joint binding');
        const nodeMap=new Map(binding.nodes.map(n=>[n.node,n.jointNode])),sourceStart=body.activeStart,sourceEnd=body.activeEnd,
            first=nodeMap.get(sourceStart),last=nodeMap.get(sourceEnd),positions=state.toolPositions.get(binding.toolId);
        if(!Number.isInteger(first)||!Number.isInteger(last)||!(last>first))fail('External source needs active native edges');
        const proxy={id:body.id,toolId:binding.toolId,activeStart:first,activeEnd:last,count:last+1,segmentCount:last,
            x:Array(last+1).fill(0),y:Array(last+1).fill(0),z:Array(last+1).fill(0),restLength:Array(last).fill(0),edges:[],sourceStart,sourceEnd,nodeMap:[...nodeMap]};
        for(let node=first;node<=last;node++) {const p=positions?.[node];if(p?.length!==3||!Array.from(p).every(Number.isFinite))fail('Own native external positions must be finite');
            proxy.x[node]=p[0];proxy.y[node]=p[1];proxy.z[node]=p[2];}
        for(let source=sourceStart;source<sourceEnd;source++) {
            const a=nodeMap.get(source),b=nodeMap.get(source+1);if(!Number.isInteger(a)||!Number.isInteger(b)||!(b>a))fail('External original body edges need ordered union bindings');
            const span=positive(state.coordinates[b]-state.coordinates[a],'Source coordinate span'),rest=positive(body.restLength[source],'Source material edge length'),
                radius=Math.max(nonnegative(body.nodeRadius[source],'Source capsule radius'),nonnegative(body.nodeRadius[source+1],'Source capsule radius')),
                materialId=body.materialCoordinate?.[source]??source;
            if(!Number.isFinite(materialId))fail('Original external material identity must be finite');
            for(let edge=a;edge<b;edge++) {
                if(!state.layout.edgeToolIds[edge]?.includes(binding.toolId))fail('Every native union child must belong to the actual external tool');
                const fraction=(state.coordinates[edge]-state.coordinates[a])/span;
                proxy.restLength[edge]=rest*(state.coordinates[edge+1]-state.coordinates[edge])/span;
                proxy.edges.push({edge,source,radius,materialId:b===a+1?materialId:signature(['native-union-child',materialId,fraction])});
            }
        }
        cache.set(body,proxy);return proxy;
    }
    const mapped=(body,node)=>new Map(body.nodeMap).get(clamp(node,body.sourceStart,body.sourceEnd));
    return world.toolContacts.flatMap((c,index)=>{
        if(!c.enabled||c.endSegmentA<c.startSegmentA||c.endSegmentB<c.startSegmentB)return [];
        if(c.bodyA===c.bodyB||c.compliance!==0)fail('External contact requires two tools and the actual hard normal law');
        if(typeof c.openDistalB!=='boolean')fail('Original external distal opening must be explicit');
        for(const key of ['startSegmentA','endSegmentA','startSegmentB','endSegmentB'])if(!Number.isInteger(c[key])||c[key]<0)fail('Original external windows must be nonnegative integer segments');
        const inner=bodyData(c.bodyA),outer=bodyData(c.bodyB),innerStart=clamp(c.startSegmentA,inner.sourceStart,inner.sourceEnd-1),
            innerEnd=clamp(c.endSegmentA,innerStart,inner.sourceEnd-1),outerStart=clamp(c.startSegmentB,outer.sourceStart,outer.sourceEnd-1),
            outerEnd=clamp(c.endSegmentB,outerStart,outer.sourceEnd-1),joints=[];
        for(const joint of world.containments??[])if(joint.enabled&&joint.innerBody===c.bodyA&&joint.outerBody===c.bodyB) {
            const own={enabled:true,innerBody:inner,outerBody:outer,enforceDistalPortal:joint.enforceDistalPortal,openDistal:joint.openDistal,
                containedLength:joint.containedLength,innerRadius:joint.innerRadius,innerArcOffset:joint.innerArcOffset,
                startNode:mapped(inner,joint.startNode),endNode:mapped(inner,joint.endNode+1)-1,sourceSlidingSegment:joint._slidingPortalState?.segment??null};
            const branch=locateKirchhoffDistalLumenBranch(own),parent=inner.edges.find(e=>e.edge===branch.segment)?.source;
            if(parent===own.sourceSlidingSegment)own._slidingPortalState={segment:branch.segment};
            joints.push({...own,branch});
        }
        return [{index,inner,outer,innerStart,innerEnd,outerStart,outerEnd,openDistalB:c.openDistalB,friction:nonnegative(c.friction,'Actual external Coulomb coefficient'),joints}];
    });
}

/** Prepare the existing finite external source windows on native union
 * children. All unowned window pairs remain declared, including open gaps;
 * the row manager refreshes each closest witness during the common solve.
 * Original distal/lumen ownership is frozen only for this preparation.
 */
export function prepareCompositeJointWorldToolContact({world,state,bindings,forcePerLength=1,frictionForcePerLength=50}={}) {
    positive(forcePerLength,'External forcePerLength');positive(frictionForcePerLength,'External frictionForcePerLength');
    const sources=snapshot(world,state,bindings),pairs=[],exclusions=[],inactivePairs=[],muByPair={},ws=createCompositeExternalCapsuleGeometryWorkspace();
    for(const s of sources) {
        const inner=s.inner.edges.filter(e=>e.source>=s.innerStart&&e.source<=s.innerEnd),outer=s.outer.edges.filter(e=>e.source>=s.outerStart&&e.source<=s.outerEnd),
            point=(b,node)=>[b.x[node],b.y[node],b.z[node]],tool={bodyA:s.inner,bodyB:s.outer,openDistalB:s.openDistalB};
        for(const a of inner)for(const b of outer) {
            const id=signature(['world-external',s.index,a.edge,b.edge]),openDistalB=s.openDistalB&&b.edge===s.outer.activeEnd-1,
                input={innerStart:point(s.inner,a.edge),innerEnd:point(s.inner,a.edge+1),outerStart:point(s.outer,b.edge),outerEnd:point(s.outer,b.edge+1),
                    innerRadius:a.radius,outerRadius:b.radius,openDistalB},g=evaluateCompositeExternalCapsuleContact({input,order:'gradient'},ws),
                detail={constraint:s.index,innerEdge:a.edge,outerEdge:b.edge,sourceInnerEdge:a.source,sourceOuterEdge:b.source};
            if(s.joints.some(j=>isKirchhoffDistalLumenWitness(j,tool,a.edge,b.edge,g.innerT,...g.innerPoint,j.branch))) {
                exclusions.push({...detail,reason:'lumen-ownership'});continue;
            }
            if(g.openDistalExcluded)inactivePairs.push({...detail,reason:'open-distal'});
            if(!g.supported) {
                // Keep the actual candidate available for contact birth. The
                // manager selects the original absent, zero-load branch at a
                // coincident witness and does not claim geometric clearance.
                if(g.reason==='coincident-native-capsule-witness')inactivePairs.push({...detail,reason:g.reason});
                else fail('Unsupported native external source geometry',{...detail,reason:g.reason});
            }
            pairs.push({id,innerToolId:s.inner.toolId,outerToolId:s.outer.toolId,innerEdge:a.edge,outerEdge:b.edge,
                innerMaterialSegmentId:a.materialId,outerMaterialSegmentId:b.materialId,innerRadius:a.radius,outerRadius:b.radius,
                feature:'external-capsule',openDistalB,endpointDerivative:'clamped-one-sided'});
            muByPair[id]=[s.friction,s.friction];
        }
    }
    const withFriction=Object.values(muByPair).some(mu=>mu[0]>0),friction=withFriction?{law:'coulomb',mu:Object.values(muByPair)[0],muByPair,
        forcePerLength:frictionForcePerLength,materialPath:'linear-affine-maps',rateMode:'backward-euler-grid',slipModel:'implicit-backward-euler-surface-rate',finiteStepSlipKnown:false}:'none',
        contacts=pairs.length?{mode:withFriction?'lumen-coulomb':'lumen-normal',friction,chartId:'world-native-external',forcePerLength,pairs}:'none',
        proof=Object.freeze({scope:'actual-world-native-external-capsule-windows',constraints:sources.length,pairs:pairs.length,excludedPairs:exclusions.length,initiallyInactivePairs:inactivePairs.length,
            continuumClearanceCertified:false,includesLegacyPostPass:false,sourceFrictionPreserved:true});
    preparations.set(proof,{world,state,bindings,source:signature(sources),numeric:signature(contacts)});
    return {contacts,proof,pairs:structuredClone(pairs),exclusions,inactivePairs};
}

export function assertCompositeJointWorldToolContact({proof,world,state,bindings,contacts}={}) {
    const saved=preparations.get(proof);
    if(!saved||saved.world!==world||saved.state!==state||saved.bindings!==bindings||saved.source!==signature(snapshot(world,state,bindings))||saved.numeric!==signature(contacts))
        fail('Actual external tools, material windows, radii, geometry, ownership or prepared law changed');
    return proof;
}
