// Contact birth adds unknowns to the NEXT physical step. Previous reactions
// are initial guesses; preserve every existing sample and its law exactly,
// while new independent normal/tangent unknowns start at zero. This is not a
// remesh, coefficient change, reaction removal or tangential-anchor reset.
const same=(a,b)=>JSON.stringify(a)===JSON.stringify(b);
export function compositeJointContactAddition({previousSignature,currentSignature,previousSampleIds,currentSampleIds}) {
    if(typeof previousSignature!=='string'||typeof currentSignature!=='string'||!Array.isArray(previousSampleIds)||!Array.isArray(currentSampleIds)||
        currentSampleIds.length<=previousSampleIds.length||new Set(previousSampleIds).size!==previousSampleIds.length||new Set(currentSampleIds).size!==currentSampleIds.length)return null;
    let before,after;try {before=JSON.parse(previousSignature);after=JSON.parse(currentSignature);}catch{return null;}
    const {provenance:oldPairs,...oldChart}=before,{provenance:newPairs,...newChart}=after;
    if(!same(oldChart,newChart)||!Array.isArray(oldPairs)||!Array.isArray(newPairs))return null;
    const byId=new Map(newPairs.map(p=>[p.id,p]));if(byId.size!==newPairs.length)return null;
    for(const old of oldPairs) {
        const next=byId.get(old.id);if(!next)return null;
        const {quadrature:oldQ,conditionalPortalSamples:oldConditional=[],...oldLaw}=old,
            {quadrature:newQ,conditionalPortalSamples:newConditional=[],...newLaw}=next;
        if(!same(oldLaw,newLaw)||!Array.isArray(oldQ)||!Array.isArray(newQ)||oldQ.some(s=>!newQ.includes(s)||oldConditional.includes(s)!==newConditional.includes(s)))return null;
    }
    const incoming=new Map(previousSampleIds.map((id,i)=>[id,i]));
    if(previousSampleIds.some(id=>!currentSampleIds.includes(id)))return null;
    return Object.freeze({previousSignature,previousSampleIds:Object.freeze(previousSampleIds.slice()),
        sourceIndices:Object.freeze(currentSampleIds.map(id=>incoming.get(id)??-1))});
}

export function compositeJointFrictionAdditionCompatible({previousSignature,currentSignature,normalAddition}) {
    if(!normalAddition)return false;
    let old,next;try{old=JSON.parse(previousSignature);next=JSON.parse(currentSignature);}catch{return false;}
    if(old.normal!==normalAddition.previousSignature)return false;
    const {normal:a,muByPair:oldMu,...oldLaw}=old,{normal:b,muByPair:newMu,...newLaw}=next;
    if(!same(oldLaw,newLaw)||!!oldMu!==!!newMu)return false;
    return !oldMu||Object.entries(oldMu).every(([id,mu])=>Object.hasOwn(newMu,id)&&same(mu,newMu[id]));
}
