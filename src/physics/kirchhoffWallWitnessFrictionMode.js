import {kirchhoffComponentBodies} from './kirchhoffComponentBodies.js';
import {evaluateKirchhoffSurfaceFrictionKKT} from './kirchhoffSurfaceFriction.js';
const axes=['fx','fy','fz','mx','my','mz'];
function profile(body){
 const value=[body.wallStaticFriction,body.wallKineticFriction];
 if(!value.every(Number.isFinite)||value[1]<0||value[0]<value[1])throw new RangeError('Wall static/kinetic profile requires finite static >= kinetic >= 0');
 return value;
}
function key(w){return JSON.stringify([w.side,w.node,w.materialA,w.materialB,w.t,w.face,w.radius,w.triangleKey]);}
function state(component,dt){
 const s=component._wallWitnessFrictionModes,bodies=kirchhoffComponentBodies(component);
 if(!s||s.committed||s.dt!==dt||s.bodies.length!==bodies.length||bodies.some((b,i)=>b!==s.bodies[i]||profile(b).some((v,j)=>v!==s.profiles[i][j])))
  throw new Error('Wall friction requires matching uncommitted static-candidate controller');
 return s;
}
/** Position-history law: each new physical step first tries the static cone.
 * No split velocity history or arbitrary stiction-speed threshold is used.
 * Caller snapshots the whole predicted step before any contact is applied. */
export function beginKirchhoffWallWitnessFrictionModes(component,{dt,step,displacementToleranceMm,coneTolerance=1e-9,maximumAttempts=8}={}){
 if(!(Number.isFinite(dt)&&dt>0&&Number.isInteger(step)&&step>=0&&Number.isFinite(displacementToleranceMm)&&displacementToleranceMm>0&&Number.isFinite(coneTolerance)&&coneTolerance>=0&&Number.isInteger(maximumAttempts)&&maximumAttempts>0))throw new RangeError('Explicit finite wall friction step and tolerances required');
 const bodies=kirchhoffComponentBodies(component);
 return component._wallWitnessFrictionModes={bodies,profiles:bodies.map(profile),dt,step,displacementToleranceMm,coneTolerance,maximumAttempts,attempt:1,overrides:new Set(),committed:false};
}
export function selectKirchhoffWallWitnessFrictionCoefficient(component,witness,dt){
 const [staticMu,kineticMu]=profile(witness.body);
 if(staticMu===kineticMu)return {mu:staticMu,key:null,mode:'equal'};
 const s=state(component,dt),identity=key(witness),mode=s.overrides.has(identity)?'slide':'stick';
 return {mu:mode==='stick'?staticMu:kineticMu,key:identity,mode};
}
/** A failed mechanics solve is never evidence of breakaway. Only an accepted
 * whole closure using the static cone can request a clean kinetic retry. */
export function evaluateKirchhoffWallWitnessFrictionCandidate(component,batch,{converged=false}={}){
 const s=state(component,batch?.dt);
 const result={status:'unconverged',accepted:false,restart:false,dt:s.dt,step:s.step,attempt:s.attempt,overrides:[...s.overrides],contacts:[]};
 if(!converged)return result;
 if(batch.component!==component||batch.committed)throw new Error('Fresh unapplied wall friction measurement batch required');
 let ambiguous=false,invalid=false;const overrides=new Set(s.overrides);
 for(const e of batch.entries){
  const selected=selectKirchhoffWallWitnessFrictionCoefficient(component,e.witness,s.dt);
  if(selected.key===null)continue;
  if(e.modeKey!==selected.key||e.mode!==selected.mode||e.surface.group.mu.some(mu=>mu!==selected.mu))throw new Error('Stale wall friction mode batch');
  const lambda=e.surface.rows.map(row=>row.lambda),displacement=e.surface.rows.map(row=>row.strain),normal=e.contact.normalLambda;
  const residual=evaluateKirchhoffSurfaceFrictionKKT(lambda,displacement,normal,[selected.mu,selected.mu]);
  const slip=Math.hypot(...displacement),force=Math.hypot(...lambda),radius=selected.mu*normal;
  const valid=residual.residualMm<=s.displacementToleranceMm&&residual.coneViolation<=s.coneTolerance;
  const stopped=valid&&(slip===0||radius>0&&force<radius*(1-s.coneTolerance)&&slip<=s.displacementToleranceMm);
  result.contacts.push({key:selected.key,mode:selected.mode,mu:selected.mu,normalLambda:normal,lambda,displacement,stopped,residualMm:residual.residualMm,coneViolation:residual.coneViolation});
  if(!valid){invalid=true;continue;}
  if(normal===0||stopped)continue;
  if(selected.mode==='stick'){
   if(slip>s.displacementToleranceMm)overrides.add(selected.key);else ambiguous=true;
  }else if(radius>0&&slip<=s.displacementToleranceMm)ambiguous=true;
 }
 if(invalid)return result;
 if(ambiguous)return {...result,status:'ambiguous'};
 if(overrides.size>s.overrides.size){
  if(s.attempt>=s.maximumAttempts)return {...result,status:'exhausted'};
  return {...result,status:'restart',restart:true,attempt:s.attempt+1,overrides:[...overrides]};
 }
 return {...result,status:'accepted',accepted:true};
}
/** Must follow whole-step rollback. Never clip or overwrite static reactions
 * at the rejected pose, and never add a second kinetic kick to that state. */
export function prepareKirchhoffWallWitnessFrictionRetry(component,decision){
 const s=state(component,decision?.dt);
 if(decision.status!=='restart'||decision.step!==s.step||decision.attempt<=s.attempt||decision.attempt>s.maximumAttempts||!Array.isArray(decision.overrides)||decision.overrides.length<=s.overrides.size||decision.overrides.length<decision.attempt-1||decision.overrides.some(k=>typeof k!=='string')||new Set(decision.overrides).size!==decision.overrides.length||[...s.overrides].some(k=>!decision.overrides.includes(k)))throw new Error('Invalid wall friction retry plan');
 for(const w of component._wallWitnessRows?.witnesses??[])if(w.ledger.lambda!==0||w.ledger.tangentLambda.some(v=>v!==0)||w.ledger.wrenches.some(v=>axes.some(a=>v[a]!==0)))throw new Error('Restore whole step with zero applied wall reactions before retry');
 s.overrides=new Set(decision.overrides);s.attempt=decision.attempt;return s;
}
export function commitKirchhoffWallWitnessFrictionModes(component,decision){
 const s=state(component,decision?.dt);
 if(!decision.accepted||decision.status!=='accepted'||decision.step!==s.step||decision.attempt!==s.attempt)throw new Error('Accepted wall friction closure required');
 s.committed=true;return true;
}
