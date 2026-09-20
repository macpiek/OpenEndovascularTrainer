/** Compare full-cycle trajectories at common arc coordinates in world space.
 * Usage: node scripts/physics/compare-shared-axis-cycles.mjs REFERENCE CANDIDATE OUTPUT.json */
import {readFileSync,writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
const [referenceDirectory,candidateDirectory,output]=process.argv.slice(2);
if(!output)throw new Error('Reference directory, candidate directory and output JSON required');
const json=(dir,name)=>JSON.parse(readFileSync(`${dir}/${name}.json`));
const rows=dir=>readFileSync(`${dir}/steps.jsonl`,'utf8').trim().split('\n').map(line=>JSON.parse(line));
const reference=rows(referenceDirectory),candidate=rows(candidateDirectory);
assert.ok(json(referenceDirectory,'summary').completed&&json(candidateDirectory,'summary').completed,'Both complete cycles required');
assert.equal(candidate.length,reference.length);
for(let i=0;i<candidate.length;i++)for(const key of ['index','phase','wire','catheter'])
    assert.equal(candidate[i][key],reference[i][key],`${key} must match at step ${i}`);
const baseline=new Map(json(referenceDirectory,'shapes').map(s=>[s.index,s])),phases={};
let compared=0;
for(const s of json(candidateDirectory,'shapes')) {
    const r=baseline.get(s.index);assert.ok(r,`Missing reference snapshot ${s.index}`);compared++;
    const xs=r.coordinates,ys=r.positions,stats=phases[s.phase]??={samples:0,squaredError:0,maxMm:0};
    let i=0;
    for(let n=0;n<s.coordinates.length;n++) {
        const x=s.coordinates[n];while(i+2<xs.length&&xs[i+1]<x)i++;
        assert.ok(x>=xs[0]-1e-6&&x<=xs.at(-1)+1e-6,'Comparable arc range required');
        const t=(x-xs[i])/(xs[i+1]-xs[i]);
        const delta=s.positions[n].map((v,k)=>v+s.origin[k]-(ys[i][k]*(1-t)+ys[i+1][k]*t+r.origin[k]));
        const distance=Math.hypot(...delta);stats.samples++;stats.squaredError+=distance*distance;stats.maxMm=Math.max(stats.maxMm,distance);
    }
}
assert.equal(compared,baseline.size,'Every reference snapshot must be compared');
const trajectory=Object.fromEntries(Object.entries(phases).map(([phase,s])=>[phase,{samples:s.samples,rmsMm:Math.sqrt(s.squaredError/s.samples),maxMm:s.maxMm}]));
const quality={allFinite:candidate.every(r=>r.quality?.finite===true),
    missingQuality:candidate.filter(r=>!r.quality).length,
    maxPenetrationMm:Math.max(...candidate.map(r=>r.quality?.maxPenetration??Infinity)),
    maxRelativeLengthError:Math.max(...candidate.flatMap(r=>r.quality?.bodies.map(b=>b.maxLengthError)??[Infinity])),
    maxCertifiedResidual:Math.max(...candidate.map(r=>r.certificateBound??Infinity)),
    totalFactorizations:candidate.reduce((sum,r)=>sum+r.factorizations,0),
    totalIterations:candidate.reduce((sum,r)=>sum+r.iterations,0)};
const report={referenceDirectory,candidateDirectory,comparedSnapshots:compared,
    comparison:'Common arc coordinates with linear interpolation, world coordinates including each snapshot origin',trajectory,quality};
writeFileSync(output,JSON.stringify(report,null,2));console.log(JSON.stringify(report,null,2));
