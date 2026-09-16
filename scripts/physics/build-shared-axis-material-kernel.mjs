// Build-only dependency: wabt@1.0.37 (or OET_WABT_PATH). Arithmetic is f64,
// with the same parentheses and accumulation order as the JS reference.
import fs from 'node:fs/promises';
import {createRequire} from 'node:module';
const require=createRequire(import.meta.url);
const g=n=>`(local.get $${n})`,set=(n,x)=>`(local.set $${n} ${x})`,i=x=>`(i32.const ${x})`,f=x=>`(f64.const ${x})`;
const op=(name,...xs)=>`(${name} ${xs.join(' ')})`,ia=(a,b)=>op('i32.add',a,b),im=(a,b)=>op('i32.mul',a,b);
const add=(a,b)=>op('f64.add',a,b),sub=(a,b)=>op('f64.sub',a,b),mul=(a,b)=>op('f64.mul',a,b),div=(a,b)=>op('f64.div',a,b),neg=a=>op('f64.neg',a);
const eq=(a,b)=>op('i32.eq',a,b),lt=(a,b)=>op('i32.lt_s',a,b),and=(a,b)=>op('i32.and',a,b),or=(a,b)=>op('i32.or',a,b);
const choose=(condition,a,b)=>`(if (result f64) ${condition} (then ${a}) (else ${b}))`;
const load=(base,index)=>op('f64.load',ia(g(base),im(typeof index==='number'?i(index):index,i(8))));
const store=(base,index,x)=>op('f64.store',ia(g(base),im(typeof index==='number'?i(index):index,i(8))),x);
const r=k=>load('record',k),q=k=>load('jacobian',k),phi=k=>r(44+k),A=k=>r(47+k),R=k=>r(typeof k==='number'?56+k:ia(i(56),k)),p=k=>r(65+k),u=k=>r(68+k),torque=k=>r(71+k);
const sum=xs=>xs.reduce(add),dot=(a,b,n=3)=>sum(Array.from({length:n},(_,k)=>mul(a(k),b(k))));
let serial=0;
const loop=(name,end,body)=>{const id=serial++;return `${set(name,i(0))}(block $end${id}(loop $loop${id}(br_if $end${id}(i32.ge_s ${g(name)} ${end}))${body}${set(name,ia(g(name),i(1)))}(br $loop${id})))`;};
const names=(prefix,n)=>Array.from({length:n},(_,k)=>prefix+k);
const floats=['x','dot','dx','dy','dz',...names('dphi',3),...names('dp',3),...names('dJ',9),...names('dA',9),...names('du',3),...names('spun',3),
    ...names('angular',6),...names('dLength',2),...names('dTorque',6),...names('column',11),'wx','wy','wz','dd1','dd2','df'];

// Local 6x6 derivative of the existing material torque law.
let derivative=set('x',dot(phi,phi));
derivative+=loop('col',i(6),`
 ${set('axis',op('i32.rem_s',g('col'),i(3)))}
 ${['dx','dy','dz'].map((v,k)=>set(v,choose(lt(g('col'),i(3)),choose(eq(g('axis'),i(k)),f(-1),f(0)),R(ia(i(k*3),g('axis')))))).join('\n')}
 ${Array.from({length:3},(_,k)=>set('dphi'+k,sum([mul(A(k*3),g('dx')),mul(A(k*3+1),g('dy')),mul(A(k*3+2),g('dz'))]))+set('dp'+k,div(g('dphi'+k),r(86+k)))).join('\n')}
 ${set('dot',dot(phi,k=>g('dphi'+k)))}
 ${Array.from({length:9},(_,k)=>{
    const a=Math.floor(k/3),b=k%3;
    const skew=a===b?f(0):a===0?(b===1?neg(g('dphi2')):g('dphi1')):a===1?(b===0?g('dphi2'):neg(g('dphi0'))):b===0?neg(g('dphi1')):g('dphi0');
    return set('dJ'+k,add(add(mul(f(-.5),skew),mul(mul(mul(f(2),r(90)),g('dot')),sub(mul(phi(a),phi(b)),a===b?g('x'):f(0)))),
        mul(r(89),sub(add(mul(g('dphi'+a),phi(b)),mul(phi(a),g('dphi'+b))),a===b?mul(f(2),g('dot')):f(0)))));
 }).join('\n')}
 ${Array.from({length:9},(_,k)=>{
    const a=Math.floor(k/3),b=k%3;
    return set('dA'+k,sum([f(0),...Array.from({length:3},(_,j)=>mul(g('dJ'+(a*3+j)),r(77+b*3+j)))]));
 }).join('\n')}
 ${Array.from({length:3},(_,k)=>set('du'+k,add(dot(j=>g('dA'+(j*3+k)),p),dot(j=>A(j*3+k),j=>g('dp'+j))))).join('\n')}
 ${Array.from({length:3},(_,k)=>{
    const j=(k+1)%3,l=(k+2)%3;
    return set('spun'+k,add(g('du'+k),choose(lt(g('col'),i(3)),sub(choose(eq(g('axis'),i(j)),u(l),f(0)),choose(eq(g('axis'),i(l)),u(j),f(0))),f(0))));
 }).join('\n')}
 ${Array.from({length:3},(_,k)=>{
    const j=(k+1)%3,l=(k+2)%3,spin=choose(op('i32.ge_s',g('col'),i(3)),sub(choose(eq(g('axis'),i(j)),torque(3+l),f(0)),choose(eq(g('axis'),i(l)),torque(3+j),f(0))),f(0));
    return store('jacobian',ia(i(k*6),g('col')),neg(g('du'+k)))+store('jacobian',ia(i((k+3)*6),g('col')),sub(dot(j=>R(j*3+k),j=>g('spun'+j)),spin));
 }).join('\n')}
`);

// Pull back all columns to the global spatial band. No symmetrization or
// omission of small entries: both triangles follow the reference scatter.
const spatial=loop('col',i(11),`
 ${[0,1].map(j=>{
    const at=3*j,sign=choose(and(op('i32.ge_s',g('col'),i(at)),lt(g('col'),i(at+3))),f(-1),choose(and(op('i32.ge_s',g('col'),i(at+3)),lt(g('col'),i(at+6))),f(1),f(0)));
    return ['dx','dy','dz'].map((v,k)=>set(v,choose(eq(op('i32.rem_s',g('col'),i(3)),i(k)),sign,f(0)))).join('\n')+
        set('dLength'+j,dot(k=>r(32+at+k),k=>g(['dx','dy','dz'][k])))+
        set('angular'+at,div(neg(dot(k=>r(24+at+k),k=>g(['dx','dy','dz'][k]))),r(30+j)))+
        set('angular'+(at+1),div(dot(k=>r(18+at+k),k=>g(['dx','dy','dz'][k])),r(30+j)))+
        set('angular'+(at+2),choose(eq(g('col'),i(9+j)),f(1),f(0)));
 }).join('\n')}
 ${Array.from({length:6},(_,k)=>set('dTorque'+k,sum([f(0),...Array.from({length:6},(_,j)=>mul(q(k*6+j),g('angular'+j)))]))).join('\n')}
 ${names('column',11).map(n=>set(n,f(0))).join('\n')}
 ${[0,1].map(j=>{
    const at=3*j,m=9*j;
    return ['wx','wy','wz'].map((v,k)=>set(v,dot(a=>r(m+3*k+a),a=>g('angular'+(at+a))))).join('\n')+
        [0,1,2].map(a=>{
            const cross=offset=>a===0?sub(mul(g('wy'),r(offset+at+2)),mul(g('wz'),r(offset+at+1))):a===1?sub(mul(g('wz'),r(offset+at)),mul(g('wx'),r(offset+at+2))):sub(mul(g('wx'),r(offset+at+1)),mul(g('wy'),r(offset+at)));
            return set('dd1',cross(18))+set('dd2',cross(24))+
                set('df',sub(div(add(sub(add(mul(neg(g('dd2')),torque(at)),mul(g('dd1'),torque(at+1))),mul(r(24+at+a),g('dTorque'+at))),mul(r(18+at+a),g('dTorque'+(at+1)))),r(30+j)),div(mul(r(38+at+a),g('dLength'+j)),r(30+j))))+
                set('column'+(at+a),sub(g('column'+(at+a)),g('df')))+set('column'+(at+3+a),add(g('column'+(at+3+a)),g('df')));
        }).join('\n')+set('column'+(9+j),g('dTorque'+(at+2)));
 }).join('\n')}
 ${set('columnDof',op('i32.load',ia(g('indices'),im(g('col'),i(4)))))}
 ${Array.from({length:11},(_,row)=>`${set('rowDof',op('i32.load',ia(g('indices'),i(row*4))))}
    ${set('entry',ia(op('i32.sub',ia(im(g('rowDof'),g('width')),g('columnDof')),g('rowDof')),g('half')))}
    ${store('hessian',g('entry'),add(load('hessian',g('entry')),g('column'+row)))}`).join('\n')}
`);
const wat=`(module(import "env" "memory" (memory 1))
 (func(export "assemble") ${['records','dofs','count','hessian','width','half','jacobian'].map(n=>`(param $${n} i32)`).join(' ')}
 ${['hinge','record','indices','col','axis','columnDof','rowDof','entry'].map(n=>`(local $${n} i32)`).join(' ')}
 ${floats.map(n=>`(local $${n} f64)`).join(' ')}
 ${loop('hinge',g('count'),set('record',ia(g('records'),im(g('hinge'),i(91*8))))+set('indices',ia(g('dofs'),im(g('hinge'),i(11*4))))+derivative+spatial)}))`;
const wabt=await require(process.env.OET_WABT_PATH||'wabt')(),parsed=wabt.parseWat('sharedAxisMaterialKernel.wat',wat);
parsed.resolveNames();parsed.validate();const {buffer}=parsed.toBinary({canonicalize_lebs:true});parsed.destroy();
await fs.writeFile(new URL('../../src/physics/kirchhoffSharedAxisMaterialKernelBytes.js',import.meta.url),
    `// Generated by scripts/physics/build-shared-axis-material-kernel.mjs. Do not edit.\nexport const sharedAxisMaterialKernelBytes=new Uint8Array([${Array.from(buffer)}]);\n`);
console.log(`Generated ${buffer.length} bytes`);
