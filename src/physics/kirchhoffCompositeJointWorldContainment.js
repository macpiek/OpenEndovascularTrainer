import {evaluateKirchhoffLumenSegmentContact} from './kirchhoffLumenContact.js';

const preparations=new WeakMap(),signature=v=>JSON.stringify(v),dot=(a,b)=>a.reduce((sum,v,k)=>sum+v*b[k],0),
    fail=(message,details={})=>{const e=new RangeError(message);e.code='joint-world-containment-adapter-required';e.details=details;throw e;},
    finite=(v,name)=>{if(!Number.isFinite(v))fail(`${name} must be finite`);return v;},
    nonnegative=(v,name)=>{if(!(finite(v,name)>=0))fail(`${name} must be nonnegative`);return v;},
    point=(v,name)=>{if(v?.length!==3||!Array.from(v).every(Number.isFinite))fail(`${name} needs three finite values`);return Array.from(v);};
const SIDE_SAMPLES=Object.freeze([.125,.375,.625,.875]),TIP_SAMPLES=Object.freeze([0,.25,.5,.75,1]);

function snapshot(world,state,bindings) {
    if(!Array.isArray(world?.containments)||!Array.isArray(bindings)||state.elasticityGeometry!=='native-discrete-rod'||state.inertiaGeometryByTool!=null)
        fail('World containment adapter requires actual bodies and native affine tools');
    const bindingByBody=new Map();
    for(const b of bindings){if(!world.bodies.includes(b.body)||bindingByBody.has(b.body)||!state.layout.spins.has(b.toolId))fail('Containment needs distinct actual body bindings');bindingByBody.set(b.body,b);}
    function bodyData(body) {
        const binding=bindingByBody.get(body);if(!binding)fail('Both actual containment bodies need own Joint bindings');
        const nodeMap=new Map(binding.nodes.map(n=>[n.node,n.jointNode])),sourceStart=body.activeStart,sourceEnd=body.activeEnd;
        if(!Number.isInteger(sourceStart)||!Number.isInteger(sourceEnd)||sourceStart<0||sourceEnd>=body.count||sourceEnd<=sourceStart)fail('Containment body needs an active physical edge');
        const sourceNodes=Array.from({length:sourceEnd-sourceStart+1},(_,j)=>{const node=sourceStart+j,jointNode=nodeMap.get(node);
            if(!Number.isInteger(jointNode)||![jointNode-1,jointNode].some(e=>state.layout.edgeToolIds[e]?.includes(binding.toolId)))fail('Every containment node needs its actual own Joint node');
            if(j&&jointNode<=nodeMap.get(node-1))fail('Containment source needs ordered native union bindings');
            return {node,jointNode,radius:nonnegative(body.nodeRadius[node],'Actual inner rod radius'),position:point(state.toolPositions.get(binding.toolId)?.[jointNode],'Own Joint position')};});
        const start=sourceNodes[0].jointNode,end=sourceNodes.at(-1).jointNode,edges=[];
        for(let parent=0;parent<sourceNodes.length-1;parent++) {
            const a=sourceNodes[parent],b=sourceNodes[parent+1],x0=state.coordinates[a.jointNode],x1=state.coordinates[b.jointNode],L=x1-x0,
                parentRest=finite(body.restLength[a.node],'Actual material edge length'),materialId=body.materialCoordinate?.[a.node]??a.node;
            if(!(L>0&&parentRest>0))fail('Containment material lengths must be positive');
            for(let edge=a.jointNode;edge<b.jointNode;edge++) {
                if(!state.layout.edgeToolIds[edge]?.includes(binding.toolId))fail('Every union child must belong to its own physical tool');
                const lo=state.coordinates[edge],hi=state.coordinates[edge+1],fraction=(lo-x0)/L,childSamples=samples=>samples.flatMap(f=>{
                    const x=x0+f*L;if(x<lo||x>hi||x===hi&&edge+1<b.jointNode)return [];
                    return [{fraction:x===lo?0:x===hi?1:(x-lo)/(hi-lo),sourceFraction:f}];});
                edges.push({node:edge,jointEdge:edge,sourceNode:a.node,restLength:parentRest*((hi-lo)/L),sourceRadius:Math.max(a.radius,b.radius),
                    materialId:a.jointNode+1===b.jointNode?materialId:signature(['native-union-child',materialId,fraction]),
                    sideSamples:childSamples(SIDE_SAMPLES),tipSamples:childSamples(TIP_SAMPLES)});
            }
        }
        const nodes=Array.from({length:end-start+1},(_,j)=>({node:start+j,jointNode:start+j,position:point(state.toolPositions.get(binding.toolId)?.[start+j],'Own union child position')}));
        return {toolId:binding.toolId,bodyId:body.id,start,end,sourceStart,sourceEnd,sourceNodes,nodes,edges};
    }
    return world.containments.flatMap((c,index)=>{
        if(!c.enabled)return [];
        if(c.innerBody===c.outerBody||c.model!=='kirchhoff'||c.innerResponse!==1||c.outerResponse!==1||c.compliance!==0)
            fail('Containment requires two reciprocal Kirchhoff tools and the source hard normal law');
        const inner=bodyData(c.innerBody),outer=bodyData(c.outerBody);
        if(inner.toolId!==state.relativeToolId)fail('Containment inner body must own the full relative coordinates');
        for(const key of ['startNode','endNode','outerStartNode','searchWindow'])if(!Number.isInteger(c[key])||c[key]<0)fail('Actual containment windows must use nonnegative integer nodes');
        if(typeof c.openProximal!=='boolean'||typeof c.openDistal!=='boolean')fail('Containment open-end policy must be explicit');
        const containedLength=c.containedLength===Infinity?'unbounded':nonnegative(c.containedLength,'Contained material length');
        const mapped=(body,node)=>body.sourceNodes.find(n=>n.node===Math.max(body.sourceStart,Math.min(body.sourceEnd,node))).jointNode;
        return [{index,inner,outer,startNode:mapped(inner,c.startNode),endNode:mapped(inner,c.endNode+1)-1,outerStartNode:mapped(outer,c.outerStartNode),searchWindow:c.searchWindow,
            innerArcOffset:nonnegative(c.innerArcOffset,'Inner material offset'),containedLength,innerRadius:nonnegative(c.innerRadius,'Actual catheter inner radius'),
            openProximal:c.openProximal,openDistal:c.openDistal,enforceDistalPortal:c.enforceDistalPortal,distalPortalModel:c.distalPortalModel,
            portalFilletRadius:nonnegative(c.portalFilletRadius,'Actual distal fillet'),axialFriction:nonnegative(c.axialFriction,'Actual axial lumen friction'),
            torsionalFriction:nonnegative(c.torsionalFriction,'Actual torsional lumen friction')}];
    });
}

/** Source preparation for the declared NATIVE affine containment model.
 * Retains actual material overlap, four interior side samples, original tip
 * samples and exact portal crossing, plus source radii and local search range.
 * Pair/feature selection is frozen for this preparation. JointLumenRows
 * re-queries every emitted original inequality at every trial; a changed
 * support requires a new source preparation. No continuum/CCD claim is made.
 */
export function prepareCompositeJointWorldContainment({world,state,bindings,forcePerLength=1,frictionForcePerLength=50}={}) {
    if(!(finite(forcePerLength,'Containment numerical forcePerLength')>0))fail('Containment numerical forcePerLength must be positive');
    if(!(finite(frictionForcePerLength,'Containment numerical frictionForcePerLength')>0))fail('Containment numerical frictionForcePerLength must be positive');
    const sources=snapshot(world,state,bindings),pairs=[],sampleProofs=[],pairMap=new Map(),exclusions=[];
    const mu=sources.length?[sources[0].axialFriction,sources[0].torsionalFriction]:[0,0];
    if(sources.some(s=>s.axialFriction!==mu[0]||s.torsionalFriction!==mu[1]))
        fail('Different actual containment friction pairs require per-pair Coulomb coefficients',
            {required:'per-pair-native-Coulomb-coefficients',coefficients:sources.map(s=>[s.axialFriction,s.torsionalFriction])});
    const withFriction=mu.some(v=>v>0),friction=withFriction?{law:'coulomb',mu,forcePerLength:frictionForcePerLength,
        materialPath:'linear-affine-maps',rateMode:'backward-euler-grid',slipModel:'implicit-backward-euler-surface-rate',finiteStepSlipKnown:false}:'none';
    for(const s of sources) {
        const innerStart=Math.max(s.inner.start,Math.min(s.inner.end-1,s.startNode)),innerEnd=Math.min(s.inner.end-1,Math.max(innerStart,s.endNode)),
            outerStart=Math.max(s.outer.start,Math.min(s.outer.end-1,s.outerStartNode)),outerLast=s.outer.end-1,outerEdges=s.outer.edges.filter(e=>e.node>=outerStart),
            arc=new Map([[outerStart,0]]),limit=s.containedLength==='unbounded'?Infinity:s.containedLength;
        outerEdges.forEach(e=>arc.set(e.node+1,arc.get(e.node)+e.restLength));const outerLength=arc.get(outerLast+1);
        const inNode=n=>s.inner.nodes[n-s.inner.start],outNode=n=>s.outer.nodes[n-s.outer.start];
        function add(innerEdge,outerEdge,feature,quadrature,conditionalPortal=false) {
            const ie=s.inner.edges[innerEdge-s.inner.start],oe=s.outer.edges[outerEdge-s.outer.start],radius=ie.sourceRadius;
            if(s.innerRadius<radius)fail('Actual guidewire radius exceeds the source lumen');
            const key=signature([s.index,ie.jointEdge,oe.jointEdge,feature]);let pair=pairMap.get(key);
            if(!pair){pair={id:key,innerToolId:s.inner.toolId,outerToolId:s.outer.toolId,innerEdge:ie.jointEdge,outerEdge:oe.jointEdge,
                innerMaterialSegmentId:ie.materialId,outerMaterialSegmentId:oe.materialId,lumenRadius:s.innerRadius,innerRadius:radius,
                openDistal:s.openDistal&&outerEdge===outerLast,portalFilletRadius:s.portalFilletRadius,feature,endpointDerivative:'clamped-one-sided',
                ...(withFriction&&s.openDistal&&outerEdge===outerLast&&s.portalFilletRadius>1e-12&&['side','distal-fillet'].includes(feature)?{featurePolicy:'native-side-fillet'}:{}),
                ...(feature==='distal-rim'?{}:{quadrature:[]})};pairMap.set(key,pair);pairs.push(pair);}
            if(quadrature!==undefined) {
                if(!pair.quadrature.includes(quadrature)) {
                    pair.quadrature.push(quadrature);
                    if(conditionalPortal)(pair.conditionalPortalSamples??=[]).push(quadrature);
                } else if(!conditionalPortal&&pair.conditionalPortalSamples) {
                    pair.conditionalPortalSamples=pair.conditionalPortalSamples.filter(s=>s!==quadrature);
                    if(!pair.conditionalPortalSamples.length)delete pair.conditionalPortalSamples;
                }
            }
        }
        function original(innerEdge,outerEdge,quadrature) {
            return evaluateKirchhoffLumenSegmentContact({innerStart:inNode(innerEdge).position,innerEnd:inNode(innerEdge+1).position,
                outerStart:outNode(outerEdge).position,outerEnd:outNode(outerEdge+1).position,lumenRadius:s.innerRadius,
                innerRadius:s.inner.edges[innerEdge-s.inner.start].sourceRadius,innerMaterialSegmentId:s.inner.edges[innerEdge-s.inner.start].materialId,
                outerMaterialSegmentId:s.outer.edges[outerEdge-s.outer.start].materialId,openDistal:s.openDistal&&outerEdge===outerLast,
                portalFilletRadius:s.portalFilletRadius,quadrature:[quadrature],activationDistance:Number.MAX_VALUE});
        }
        let innerArc=s.innerArcOffset;
        for(let innerEdge=innerStart;innerEdge<=innerEnd;innerEdge++) {
            const innerChild=s.inner.edges[innerEdge-s.inner.start],rest=innerChild.restLength;
            for(const {fraction,sourceFraction} of innerChild.sideSamples) {
                const materialArc=innerArc+rest*fraction;
                if(materialArc>limit+1e-12||materialArc>outerLength+1e-12){exclusions.push({constraint:s.index,innerEdge,fraction,reason:'outside-contained-material-span'});continue;}
                let expected=outerStart;while(expected<outerLast&&materialArc>arc.get(expected+1))expected++;
                const expectedSource=s.outer.edges[expected-s.outer.start].sourceNode,window=Math.max(1,s.searchWindow),
                    search=outerEdges.filter(e=>e.sourceNode>=expectedSource-window&&e.sourceNode<=expectedSource+window),lo=search[0].node,hi=search.at(-1).node,
                    p=inNode(innerEdge).position.map((v,k)=>v+fraction*(inNode(innerEdge+1).position[k]-v));let winner=null;
                for(let outerEdge=lo;outerEdge<=hi;outerEdge++) {
                    const C=outNode(outerEdge).position,D=outNode(outerEdge+1).position,d=D.map((v,k)=>v-C[k]),length2=dot(d,d);
                    if(!(length2>1e-12))fail('Degenerate native containment search edge');
                    const rawT=dot(p.map((v,k)=>v-C[k]),d)/length2,t=Math.max(0,Math.min(1,rawT)),distance=Math.hypot(...p.map((v,k)=>v-C[k]-t*d[k]));
                    if(!winner||distance<winner.distance)winner={outerEdge,distance,rawT};
                }
                const raw=original(innerEdge,winner.outerEdge,fraction),feature=raw.side?'side':raw.fillet?'distal-fillet':null;
                if(feature){add(innerEdge,winner.outerEdge,feature,fraction);sampleProofs.push({constraint:s.index,innerEdge,fraction,sourceInnerEdge:innerChild.sourceNode,sourceFraction,
                    outerEdge:winner.outerEdge,search:[lo,hi],sourceSearch:[expectedSource-window,expectedSource+window],feature});}
                else if(s.openDistal&&winner.outerEdge===outerLast||s.openProximal&&winner.outerEdge===outerStart&&winner.rawT<0)
                    exclusions.push({constraint:s.index,innerEdge,fraction,reason:'open-end-owned-sample'});
                else fail('A native containment sample needs another local pair/feature chart',{constraint:s.index,innerEdge,fraction,outerEdge:winner.outerEdge});
            }
            if(s.openDistal&&innerArc<=Math.min(limit,outerLength)+rest) {
                // Preserve original distal quadrature and the exact geometric
                // crossing independently of side sample ownership.
                for(const {fraction} of innerChild.tipSamples){const raw=original(innerEdge,outerLast,fraction);
                    if(raw.fillet)add(innerEdge,outerLast,'distal-fillet',fraction);
                    if(raw.side&&raw.portal.crosses)add(innerEdge,outerLast,'side',fraction,true);
                    if(raw.portal.crosses)add(innerEdge,outerLast,'distal-rim');}
                // Every actual native child can carry the geometric aperture
                // crossing even when no inherited source quadrature lies in it.
                if(original(innerEdge,outerLast,.5).portal.crosses)add(innerEdge,outerLast,'distal-rim');
            }
            innerArc+=rest;
        }
    }
    const contacts=pairs.length?{mode:withFriction?'lumen-coulomb':'lumen-normal',friction,...(withFriction?{historyUpdate:'preserve-and-append'}:{}),chartId:'world-native-containment',forcePerLength,pairs}:'none',
        proof=Object.freeze({scope:'actual-world-native-affine-containment-samples',constraints:sources.length,pairs:pairs.length,samples:sampleProofs.length,
            continuumClearanceCertified:false,includesLegacyPostPass:false,sourceFrictionPreserved:true});
    preparations.set(proof,{world,state,bindings,source:signature(sources),numeric:signature(contacts),sampleProofs,exclusions});
    return {contacts,proof,samples:structuredClone(sampleProofs),exclusions:structuredClone(exclusions)};
}

export function assertCompositeJointWorldContainment({proof,world,state,bindings,contacts}={}) {
    const p=preparations.get(proof);
    if(!p||p.world!==world||p.state!==state||p.bindings!==bindings||p.source!==signature(snapshot(world,state,bindings))||p.numeric!==signature(contacts))
        fail('Actual containment bodies, material windows, radii, geometry or prepared law changed');
    return proof;
}
