// Compile the fixed centerline/spin stencil, including its exact first
// derivatives. Optional build dependency only: wabt, see OET_WABT_PATH.
import fs from 'node:fs/promises';
import { createRequire } from 'node:module';
const require = createRequire(import.meta.url);

function compile(toolCount) {
    const n = 9 + 2 * toolCount, nodes = [], cache = new Map(), checks = [];
    function node(op, args) {
        if (op === 'add') { if (args[0] === zero) return args[1]; if (args[1] === zero) return args[0]; }
        if (op === 'mul') {
            if (args.includes(zero)) return zero;
            if (args[0] === one) return args[1]; if (args[1] === one) return args[0];
        }
        if (op === 'sub' && args[1] === zero) return args[0];
        const key = op + ':' + args.join(',');
        if (cache.has(key)) return cache.get(key);
        const id = nodes.length; nodes.push({ op, args }); cache.set(key, id); return id;
    }
    let zero = -1, one = -1;
    const constant = value => node('const', [value]);
    zero = constant(0); one = constant(1);
    const addV = (a,b) => node('add',[a,b]), subV = (a,b) => node('sub',[a,b]);
    const mulV = (a,b) => node('mul',[a,b]), divV = (a,b) => node('div',[a,b]);
    const scaleV = (a,s) => mulV(a,constant(s));
    const input = index => node('input',[index]);
    const jet = (v, index = -1) => ({ v, d: Array.from({length:n},(_,i)=>i===index?one:zero) });
    const unary = (a,v,f) => ({v,d:a.d.map(x=>mulV(f,x))});
    const binary = (a,b,v,da,db) => ({v,d:a.d.map((x,i)=>addV(mulV(da,x),mulV(db,b.d[i])))});
    const add = (a,b) => binary(a,b,addV(a.v,b.v),one,one);
    const sub = (a,b) => binary(a,b,subV(a.v,b.v),one,constant(-1));
    const mul = (a,b) => binary(a,b,mulV(a.v,b.v),b.v,a.v);
    const div = (a,b) => binary(a,b,divV(a.v,b.v),divV(one,b.v),divV(scaleV(a.v,-1),mulV(b.v,b.v)));
    const scale = (a,s) => unary(a,mulV(a.v,s),s);
    const sqrt = a => {const v=node('sqrt',[a.v]);return unary(a,v,divV(constant(.5),v));};
    const sin = a => unary(a,node('sin',[a.v]),node('cos',[a.v]));
    const cos = a => unary(a,node('cos',[a.v]),scaleV(node('sin',[a.v]),-1));
    const atan2 = (y,x) => {const d=addV(mulV(x.v,x.v),mulV(y.v,y.v));return binary(y,x,node('atan2',[y.v,x.v]),divV(x.v,d),divV(scaleV(y.v,-1),d));};
    const plus = (a,b) => a.map((x,i)=>add(x,b[i]));
    const minus = (a,b) => a.map((x,i)=>sub(x,b[i]));
    const times = (a,s) => a.map(x=>mul(x,s));
    const dot = (a,b) => add(add(mul(a[0],b[0]),mul(a[1],b[1])),mul(a[2],b[2]));
    const dotV = (a,b) => addV(addV(mulV(a[0],b[0]),mulV(a[1],b[1])),mulV(a[2],b[2]));
    const cross = (a,b) => [sub(mul(a[1],b[2]),mul(a[2],b[1])),sub(mul(a[2],b[0]),mul(a[0],b[2])),sub(mul(a[0],b[1]),mul(a[1],b[0]))];
    const unit = v => {const length=sqrt(dot(v,v)); checks.push({v:length.v,minimum:0,status:1});return v.map(x=>div(x,length));};
    const transport = (v,from,to) => {
        const axis=cross(from,to), denominator=add(jet(one),dot(from,to));
        checks.push({v:denominator.v,minimum:1e-10,status:2});
        const first=cross(axis,v),second=cross(axis,first).map(x=>div(x,denominator));
        return plus(plus(v,first),second);
    };
    const p=Array.from({length:3},(_,i)=>Array.from({length:3},(_,j)=>jet(input(3*i+j),3*i+j)));
    const t=[unit(minus(p[1],p[0])),unit(minus(p[2],p[1]))];
    const director=[0,1].map(i=>transport(Array.from({length:3},(_,j)=>jet(input(9+6*i+3+j))),
        Array.from({length:3},(_,j)=>jet(input(9+6*i+j))),t[i]));
    const perpendicular=t.map((v,i)=>cross(v,director[i]));
    const denominator=add(jet(one),dot(t[0],t[1]));
    checks.push({v:denominator.v,minimum:1e-10,status:3});
    const kb=cross(t[0],t[1]).map(v=>div(scale(v,constant(2)),denominator));
    const transported=transport(director[0],t[0],t[1]);
    const rawReferenceTwist=atan2(dot(t[1],cross(transported,director[1])),dot(transported,director[1]));
    const outputs=[], materialLengths=[]; let totalEnergy=zero;
    for(let tool=0;tool<toolCount;tool++) {
        const base=22+17*tool,theta=[0,1].map(i=>jet(input(base+i),9+2*tool+i));
        const tau=constant(2*Math.PI);
        const winding=mulV(tau,node('round',[divV(subV(input(base+16),rawReferenceTwist.v),tau)]));
        const referenceTwist=add(rawReferenceTwist,jet(winding));
        outputs.push([275+tool,referenceTwist.v]);
        const c=theta.map(cos),s=theta.map(sin);
        const m1=director.map((v,i)=>plus(times(v,c[i]),times(perpendicular[i],s[i])));
        const m2=director.map((v,i)=>minus(times(perpendicular[i],c[i]),times(v,s[i])));
        const length=mulV(input(21),input(base+2)),inverseLength=divV(one,length);
        materialLengths.push(length);
        const rates=[scale(dot(kb,plus(m1[0],m1[1])),scaleV(inverseLength,.5)),
            scale(dot(kb,plus(m2[0],m2[1])),scaleV(inverseLength,.5)),
            scale(add(sub(theta[1],theta[0]),referenceTwist),inverseLength)];
        const error=rates.map((r,i)=>subV(r.v,input(base+12+i)));
        const moment=error.map((_,i)=>dotV([input(base+3+3*i),input(base+4+3*i),input(base+5+3*i)],error));
        const energy=mulV(length,addV(scaleV(dotV(error,moment),.5),input(base+15)));
        totalEnergy=addV(totalEnergy,energy);outputs.push([1+tool,energy]);
        for(let row=0;row<3;row++) {
            outputs.push([3+3*tool+row,rates[row].v],[9+3*tool+row,moment[row]]);
            for(let i=0;i<n;i++)outputs.push([15+(3*tool+row)*n+i,rates[row].d[i]]);
        }
    }
    outputs.push([0,totalEnergy]);
    const expression=id=>{
        const {op,args}=nodes[id];
        if(op==='const')return `(f64.const ${args[0]})`;
        return `(local.get $v${id})`;
    };
    const locals=nodes.flatMap((o,i)=>o.op==='const'?[]:[`(local $v${i} f64)`]).join('\n');
    const instructions=nodes.flatMap(({op,args},i)=>{
        if(op==='const')return[];
        const value=op==='input'?`(f64.load offset=${8*args[0]} (local.get $input))`:
            ['sin','cos','atan2','round'].includes(op)?`(call $${op} ${args.map(expression).join(' ')})`:
            `(f64.${op} ${args.map(expression).join(' ')})`;
        return [`(local.set $v${i} ${value})`];
    }).join('\n');
    const guards=checks.map(({v,minimum,status})=>`(if (i32.eqz (f64.gt ${expression(v)} (f64.const ${minimum}))) (then (return (i32.const ${status}))))`).join('\n');
    const stores=outputs.map(([offset,id])=>`(f64.store offset=${offset*8} (local.get $output) ${expression(id)})`).join('\n');
    const accumulate=materialLengths.map((length,tool)=>`(call $accumulate (local.get $input) (local.get $output) (i32.const ${n}) (i32.const ${tool}) ${expression(length)})`).join('\n');
    return {nodes:nodes.length,wat:`(func (export "evaluate${toolCount}") (param $input i32) (param $output i32) (result i32)
        ${locals}
        ${instructions}
        ${guards}
        ${stores}
        (memory.fill (i32.add (local.get $output) (i32.const ${93*8})) (i32.const 0) (i32.const ${(13+169)*8}))
        ${accumulate}
        (call $mirror (local.get $output) (i32.const ${n}))
        (i32.const 0))`};
}

const kernels=[compile(1),compile(2)];
const wat=`(module
 (import "math" "sin" (func $sin (param f64) (result f64)))
 (import "math" "cos" (func $cos (param f64) (result f64)))
 (import "math" "atan2" (func $atan2 (param f64 f64) (result f64)))
 (import "math" "round" (func $round (param f64) (result f64)))
 (memory (export "memory") 1)
 (func $get (param $base i32) (param $index i32) (result f64)
  (f64.load (i32.add (local.get $base) (i32.shl (local.get $index) (i32.const 3)))))
 (func $addAt (param $base i32) (param $index i32) (param $value f64)
  (local $address i32)
  (local.set $address (i32.add (local.get $base) (i32.shl (local.get $index) (i32.const 3))))
  (f64.store (local.get $address) (f64.add (f64.load (local.get $address)) (local.get $value))))
 (func $accumulate (param $input i32) (param $output i32) (param $n i32) (param $tool i32) (param $length f64)
  (local $row i32) (local $i i32) (local $j i32) (local $c i32)
  (local $jac i32) (local $ki i32) (local $component i32)
  (local $ji f64) (local $wj f64) (local $moment f64)
  (loop $rows
   (local.set $jac (i32.add (i32.const 15) (i32.mul (i32.add (i32.mul (local.get $tool) (i32.const 3)) (local.get $row)) (local.get $n))))
   (local.set $ki (i32.add (i32.const 25) (i32.add (i32.mul (local.get $tool) (i32.const 17)) (i32.mul (local.get $row) (i32.const 3)))))
   (local.set $moment (call $get (local.get $output) (i32.add (i32.const 9) (i32.add (i32.mul (local.get $tool) (i32.const 3)) (local.get $row)))))
   (local.set $i (i32.const 0))
   (loop $cols
    (local.set $ji (call $get (local.get $output) (i32.add (local.get $jac) (local.get $i))))
    (call $addAt (local.get $output) (i32.add (i32.const 93) (local.get $i)) (f64.mul (f64.mul (local.get $length) (local.get $ji)) (local.get $moment)))
    (local.set $j (i32.const 0))
    (loop $hessian
     (local.set $wj (f64.const 0)) (local.set $c (i32.const 0))
     (loop $components
      (local.set $component (i32.add (i32.const 15) (i32.add (i32.mul (i32.add (i32.mul (local.get $tool) (i32.const 3)) (local.get $c)) (local.get $n)) (local.get $j))))
      (local.set $wj (f64.add (local.get $wj) (f64.mul (call $get (local.get $input) (i32.add (local.get $ki) (local.get $c))) (call $get (local.get $output) (local.get $component)))))
      (local.set $c (i32.add (local.get $c) (i32.const 1))) (br_if $components (i32.lt_s (local.get $c) (i32.const 3))))
     (call $addAt (local.get $output) (i32.add (i32.const 106) (i32.add (i32.mul (local.get $i) (local.get $n)) (local.get $j))) (f64.mul (f64.mul (local.get $length) (local.get $ji)) (local.get $wj)))
     (local.set $j (i32.add (local.get $j) (i32.const 1))) (br_if $hessian (i32.le_s (local.get $j) (local.get $i))))
    (local.set $i (i32.add (local.get $i) (i32.const 1))) (br_if $cols (i32.lt_s (local.get $i) (local.get $n))))
   (local.set $row (i32.add (local.get $row) (i32.const 1))) (br_if $rows (i32.lt_s (local.get $row) (i32.const 3)))))
 (func $mirror (param $output i32) (param $n i32) (local $i i32) (local $j i32)
  (local.set $i (i32.const 1))
  (loop $rows
   (local.set $j (i32.const 0))
   (loop $cols
    (f64.store (i32.add (local.get $output) (i32.mul (i32.add (i32.const 106) (i32.add (i32.mul (local.get $j) (local.get $n)) (local.get $i))) (i32.const 8)))
     (call $get (local.get $output) (i32.add (i32.const 106) (i32.add (i32.mul (local.get $i) (local.get $n)) (local.get $j)))))
    (local.set $j (i32.add (local.get $j) (i32.const 1))) (br_if $cols (i32.lt_s (local.get $j) (local.get $i))))
   (local.set $i (i32.add (local.get $i) (i32.const 1))) (br_if $rows (i32.lt_s (local.get $i) (local.get $n)))))
 ${kernels.map(k=>k.wat).join('\n')}
)`;
const wabt=await require(process.env.OET_WABT_PATH || 'wabt')();
const parsed=wabt.parseWat('kirchhoffCompositeElementFast.wat',wat,{bulk_memory:true});
parsed.validate();
const {buffer}=parsed.toBinary({canonicalize_lebs:true}); parsed.destroy();
const output=new URL('../../src/physics/kirchhoffCompositeElementFastKernelBytes.js',import.meta.url);
await fs.writeFile(output,`// Generated by scripts/physics/build-composite-element-fast.mjs. Do not edit.\nexport const compositeElementFastKernelBytes=new Uint8Array([${Array.from(buffer)}]);\n`);
console.log(JSON.stringify({bytes:buffer.length,symbolicNodes:kernels.map(k=>k.nodes)}));
