// Frozen existing mechanics, not browser FPS or a dynamic insertion replay.
import fs from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {solveCoulombNewton} from '../../src/physics/kirchhoffCoulombNewtonSolver.js';
import {createCoulombBandLayout} from '../../src/physics/kirchhoffCoulombBandLU.js';
import {condenseKirchhoffTwoChannelSystem} from '../../src/physics/kirchhoffTwoChannelCondensation.js';
import {measureCoupledLoadKKT} from '../../src/physics/kirchhoffCoupledLoadSolver.js';
const f=JSON.parse(gunzipSync(fs.readFileSync(new URL('../../tests/fixtures/kirchhoff-two-channel-condensation.json.gz',import.meta.url))),
    (_,v)=>v==='Infinity'?Infinity:v==='-Infinity'?-Infinity:v);
const lower=[],upper=[],physical=[];let n=0;
f.native.order.forEach((original,sorted)=>{
    physical[sorted]=n++;lower.push(f.native.lower[sorted]);upper.push(f.native.upper[sorted]);
    const b=f.channels[original].bias;if(b){n++;lower.push(b.lower-b.lambda);upper.push(b.upper-b.lambda);}
});
const groups=f.native.groups.map(g=>({...g,rows:g.rows.map(i=>physical[i]),normalRow:g.normalRow==null?undefined:physical[g.normalRow]}));
const a=Float64Array.from(f.oracle.matrix),rhs=Float64Array.from(f.oracle.rhs),layout=createCoulombBandLayout(a,n,n,groups,'row-major'),
    compact={starts:layout.starts,ends:layout.ends,offsets:layout.offsets,values:new Float64Array(layout.entries)};
for(let i=0;i<n;i++)for(let j=layout.starts[i];j<=layout.ends[i];j++)compact.values[layout.offsets[i]+j]=a[i*n+j];
const variants=['condensed','full-dense','full-band'],samples=Object.fromEntries(variants.map(v=>[v,[]])),last={};
for(let pass=0;pass<9;pass++)for(const name of pass%2?variants.slice().reverse():variants){
    const start=performance.now();let result,x,entries;
    if(name==='condensed'){
        const c=condenseKirchhoffTwoChannelSystem(f.native,f.channels);
        if(c.status!=='condensed')throw new Error(c.reason);
        result=solveCoulombNewton(c.matrix,c.rhs,c.lower,c.upper,c.count,c.count,c.groups,{matrixFormat:'row-major',tolerance:2e-4});
        x=c.recover(result.increment).fullIncrement;entries=c.matrix.length;
    }else{
        result=solveCoulombNewton(name==='full-band'?compact:a,rhs,lower,upper,n,n,groups,
            {matrixFormat:name==='full-band'?'general-band':'row-major',tolerance:2e-4});x=result.increment;entries=name==='full-band'?compact.values.length:a.length;
    }
    const ms=performance.now()-start;if(pass>=2)samples[name].push(ms);
    const residual=Float64Array.from(rhs,(v,i)=>v-x.reduce((sum,value,j)=>sum+a[i*n+j]*value,0)),actualGroups=groups.map(g=>({...g,
        radii:g.normalRow==null?g.radii:g.mu.map(mu=>mu*Math.max(0,g.normalLambda+x[g.normalRow]))})),kkt=measureCoupledLoadKKT(residual,x,lower,upper,actualGroups);
    last[name]={converged:result.diagnostics.converged,originalKkt:kkt.maximumResidual,operatorEntries:entries,diagnostics:result.diagnostics};
}
const result={scope:'saved-155-native-298-expanded-existing-mechanics',limitation:'Frozen Node microbenchmark; condensed includes condensation and recovery; excludes geometry and browser.',variants:{}};
for(const name of variants){const sorted=samples[name].slice().sort((a,b)=>a-b);result.variants[name]={...last[name],samplesMs:samples[name],medianMs:sorted[Math.floor(sorted.length/2)]};}
const text=JSON.stringify(result,null,2);if(process.argv[2])fs.writeFileSync(process.argv[2],text+'\n');console.log(text);
