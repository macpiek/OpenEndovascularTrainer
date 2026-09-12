// Independent affine mechanics oracle. No application imports or anatomy replay.
import assert from 'node:assert/strict';
import fs from 'node:fs';
const dt=1/120, targetGap=.01632478, tolerance=.0002;
const near=(a,b,t=1e-10)=>assert.ok(Math.abs(a-b)<=t,`${a} != ${b}`);
function solve(A,rhs){
  A=A.map((r,i)=>[...r,rhs[i]]);const n=rhs.length;
  for(let i=0;i<n;i++){
    let p=i;for(let k=i+1;k<n;k++)if(Math.abs(A[k][i])>Math.abs(A[p][i]))p=k;
    assert.ok(Math.abs(A[p][i])>1e-14,'singular affine block');[A[i],A[p]]=[A[p],A[i]];
    const f=A[i][i];for(let j=i;j<=n;j++)A[i][j]/=f;
    for(let k=0;k<n;k++)if(k!==i){const f=A[k][i];for(let j=i;j<=n;j++)A[k][j]-=f*A[i][j];}
  }
  return A.map(r=>r[n]);
}
function residuals(c,u,b,p,beta){
  const {m,K,xPred,rest,vPred=0,step=dt}=c,H=m+K,q=xPred+u+b;
  const lambda=-K*(q-rest),betaMaterial=-K*b,h=Math.max(0,c.xStart??xPred)+step*vPred+u;
  const physicalMomentum=m*u-lambda-p,biasMomentum=m*b-betaMaterial-beta;
  return {u,b,p,beta,q,lambda,betaMaterial,vPhysical:vPred+u/step,
    geometryViolation:Math.max(0,-q),physicalNormalViolation:p>1e-10?Math.abs(h):Math.max(0,-h),
    physicalMomentum,biasMomentum,materialResidual:K>0?q-rest+lambda/K:0,
    biasMaterialResidual:K>0?b+betaMaterial/K:0,
    kinetic:.5*m*(vPred+u/step)**2,elastic:K>0?.5*K/(step*step)*(q-rest)**2:0,H};
}
function coupled(c){
  const {m,K,xPred,rest,vPred=0,step=dt}=c,H=m+K,hPred=Math.max(0,c.xStart??xPred)+step*vPred;
  for(const loaded of [false,true])for(const biasLoaded of [false,true]){
    // Unknowns [u,b,p,beta]. Physical momentum after eliminating material;
    // compliant bias metric; physical normal and geometric complementarity.
    const A=[[H,K,-1,0],[0,H,0,-1],loaded?[1,0,0,0]:[0,0,1,0],biasLoaded?[1,1,0,0]:[0,0,0,1]];
    const rhs=[-K*(xPred-rest),0,loaded?-hPred:0,biasLoaded?-xPred:0];
    const [u,b,p,beta]=solve(A,rhs),s=residuals(c,u,b,p,beta),h=hPred+u;
    if(p< -1e-9||beta< -1e-9||h< -1e-9||s.q< -1e-9||Math.abs(p*h)>1e-9||Math.abs(beta*s.q)>1e-9)continue;
    const algebraMaximum=Math.max(...A.map((r,i)=>Math.abs(r.reduce((v,a,j)=>v+a*[u,b,p,beta][j],0)-rhs[i])));
    near(s.physicalMomentum,0);near(s.biasMomentum,0);near(s.materialResidual,0);near(s.biasMaterialResidual,0);
    near(s.geometryViolation,0);near(s.physicalNormalViolation,0);
    return {...s,algebraMaximum,loaded,biasLoaded};
  }
  throw Error('No feasible normal active set');
}
function friction(normalLambda,biasLambda,{slide=3,spin=-2,r=.5,m=1,I=.25,mu=.3,step=dt}={}){
  // Axial and circumferential contact components have equal mobility here.
  near(1/m,r*r/I);const w=1/m,slip=[step*slide,step*r*spin],length=Math.hypot(...slip),cap=mu*normalLambda;
  const f=length?Math.min(1/w,cap/length):0,t=slip.map(v=>-f*v);
  const outSlide=slide+t[0]/(m*step),outSpin=spin+r*t[1]/(I*step);
  near(m*(outSlide-slide),t[0]/step);near(I*(outSpin-spin),r*t[1]/step);
  assert.ok(Math.hypot(...t)<=cap+1e-12);
  const before=.5*m*slide*slide+.5*I*spin*spin,after=.5*m*outSlide*outSlide+.5*I*outSpin*outSpin;
  assert.ok(after<=before+1e-12);
  return {normalLambda,biasLambda,cap,tangentLambda:t,slide,spin,outSlide,outSpin,kineticLoss:before-after};
}
const stiff={m:1,K:99,xPred:-200*targetGap,rest:100*targetGap/99};
const H=stiff.m+stiff.K,physicalU=-stiff.K*(stiff.xPred-stiff.rest)/H;
near(stiff.xPred+physicalU,-targetGap);
const onePassB=-(stiff.xPred+physicalU),oldLambda=stiff.m*physicalU;
const onePass={u:physicalU,b:onePassB,q:stiff.xPred+physicalU+onePassB,
  materialResidual:stiff.xPred+physicalU+onePassB-stiff.rest+oldLambda/stiff.K};
near(onePass.materialResidual,targetGap);assert.ok(onePass.materialResidual>tolerance);
const exact=coupled(stiff);near(exact.q,0);near(exact.p,0);assert.ok(exact.vPhysical>0);
near(exact.u,100*targetGap);near(exact.b,100*targetGap);
// Independently assemble the proposed native row adapter, without eliminating
// material rows. Unknowns [lambdaMaterialPhysical,p,betaMaterial,betaNormal].
const W=1/stiff.m,A=1/stiff.K;
const rowMatrix=[[W+A,W,W,W],[0,1,0,0],[0,0,W+A,W],[W,W,W,W]];
const rowRhs=[-(stiff.xPred-stiff.rest),0,0,-stiff.xPred];
const [lambdaRow,pRow,betaMaterialRow,betaNormalRow]=solve(rowMatrix,rowRhs);
const nativeRowAdapter={unknowns:[lambdaRow,pRow,betaMaterialRow,betaNormalRow],matrix:rowMatrix,rhs:rowRhs,
  u:W*(lambdaRow+pRow),b:W*(betaMaterialRow+betaNormalRow)};
near(nativeRowAdapter.u,exact.u);near(nativeRowAdapter.b,exact.b);near(betaNormalRow,exact.beta);
assert.notEqual(rowMatrix[0][2],rowMatrix[2][0],'Cross blocks must not be symmetrized');
let b=0,iterationsToTolerance=null;const checkpoints=[];
for(let iteration=1;iteration<=1000;iteration++){
  // Exact block Gauss-Seidel for the SAME equations, with fixed active set.
  const u=(-stiff.K*(stiff.xPred-stiff.rest)-stiff.K*b)/H;
  const lambda=stiff.m*u,nextB=-stiff.xPred-u;
  const q=stiff.xPred+u+nextB,material=q-stiff.rest+lambda/stiff.K;
  const expected=targetGap*(stiff.K/H)**(iteration-1);near(material,expected,2e-13);
  if(iterationsToTolerance===null&&Math.abs(material)<=tolerance)iterationsToTolerance=iteration;
  if([1,2,30,64,128,256,440,1000].includes(iteration))checkpoints.push({iteration,u,b:nextB,geometryViolation:Math.max(0,-q),materialResidual:material});
  b=nextB;
}
assert.ok(checkpoints.find(v=>v.iteration===64).materialResidual>.001);
near(b,exact.b,8e-5);
const unloaded=[];
for(const step of [1/60,1/120]){
  const state=coupled({m:1,K:0,xPred:-targetGap,rest:0,step});
  near(state.u,0);near(state.p,0);near(state.b,targetGap);
  const channels=friction(state.p,state.beta,{step});near(channels.outSlide,3);near(channels.outSpin,-2);
  unloaded.push({step,state,channels});
}
const loaded=coupled({m:1,K:2,xPred:-targetGap,rest:-targetGap});
near(loaded.p,2*targetGap);near(loaded.u,0);assert.ok(loaded.beta>0);
const loadedFriction=friction(loaded.p,loaded.beta);
const changedBiasFriction=friction(loaded.p,loaded.beta*1000);
near(loadedFriction.outSlide,changedBiasFriction.outSlide);near(loadedFriction.outSpin,changedBiasFriction.outSpin);
const recoil=coupled({m:1,K:2,xPred:1,rest:0});near(recoil.b,0);near(recoil.p,0);near(recoil.u,-2/3);
const result={status:'PASS',scope:'Synthetic affine oracle only; no native rod/anatomy acceptance claim',dt,tolerance,
  parameters:stiff,physicalOnly:{u:physicalU,geometryGap:stiff.xPred+physicalU},
  preserveStrain:{E:[[1]],rank:1,mobileDofs:1,requiredRepair:targetGap,zeroStrainBiasSolution:0,feasible:false},
  onePass,exact,nativeRowAdapter,alternating:{contraction:stiff.K/H,iterationsToTolerance,checkpoints},
  unloadedBiasWithIndependentSlideSpin:unloaded,loadedElasticReaction:{state:loaded,friction:loadedFriction,biasScaled1000:changedBiasFriction},freeElasticRecoil:recoil};
fs.writeFileSync(new URL('./split-compliant-reclosure-oracle.json',import.meta.url),JSON.stringify(result,null,2));
console.log(JSON.stringify({status:result.status,onePassMaterial:onePass.materialResidual,blockResidual:exact.algebraMaximum,
  exactU:exact.u,exactBias:exact.b,alternating64:checkpoints.find(v=>v.iteration===64),iterationsToTolerance,
  physicalLoad:loaded.p,biasLoad:loaded.beta,loadedFriction},null,2));
