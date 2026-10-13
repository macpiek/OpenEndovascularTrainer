import test from 'node:test';
import assert from 'node:assert/strict';
import { createPatientMotion, createPatientSoftTissue } from '../src/imaging/patientSoftTissue.js';

test('patient motion freezes with accepted time and resets without a phase jump from old history', () => {
    const motion = createPatientMotion();
    const first = {...motion.update(1, {cardiacPhase:.2})};
    assert.deepEqual(motion.update(1, {cardiacPhase:.2}), first);
    assert.equal(motion.update(0, {cardiacPhase:0}).inspiration, 0);
    assert.equal(motion.state.contraction, 0);
    assert.throws(() => motion.update(NaN), /Invalid patient time/);
});

test('respiration has the same phase under different acquisition schedules and changing rate is continuous', () => {
    const a = createPatientMotion(), b = createPatientMotion();
    for(let i=0;i<=600;i++)a.update(i/60,{respiratoryRate:15});
    for(let i=0;i<=150;i++)b.update(i/15,{respiratoryRate:15});
    assert.ok(Math.abs(a.state.inspiration-b.state.inspiration)<1e-12);
    const prior=a.state.inspiration;
    a.update(10,{respiratoryRate:18});assert.equal(a.state.inspiration,prior);
});

test('organs move in world coordinates without changing geometry or allocating new meshes', () => {
    const tissue=createPatientSoftTissue();
    try {
        tissue.update(0,{cardiacPhase:0});
        const meshes=tissue.scene.children[0].children.slice();
        const geometry=meshes.map(m=>m.geometry), initial=tissue.heart.scale.clone();
        tissue.update(1,{cardiacPhase:.24});
        assert.ok(tissue.heart.scale.x<initial.x);
        assert.ok(tissue.rightLung.position.y<249);
        assert.deepEqual(tissue.scene.children[0].children,meshes);
        assert.deepEqual(meshes.map(m=>m.geometry),geometry);
        tissue.update(1,{cardiacPhase:.24});tissue.heart.updateMatrix();
        assert.ok(tissue.motion.state.contraction>0);
        for(const mesh of meshes) {
            assert.ok(Array.from(mesh.geometry.attributes.position.array).every(Number.isFinite));
            if(mesh.name!=='bowel-loop')continue;
            const positions=mesh.geometry.attributes.position, center=mesh.geometry.attributes.tubeCenter;
            const {tubularSegments,radialSegments}=mesh.geometry.parameters;
            for(const ring of [0,tubularSegments])for(let i=0;i<=radialSegments;i++){
                const at=ring*(radialSegments+1)+i;
                assert.equal(positions.getX(at),center.getX(at));
                assert.equal(positions.getY(at),center.getY(at));
                assert.equal(positions.getZ(at),center.getZ(at));
            }
        }
        // Matching wall/gas motion coordinates keep the nested volumes together
        // during peristalsis, including the longer colon and its closed caps.
        const bowel=meshes.filter(m=>m.name==='bowel-loop');
        for(let i=0;i<bowel.length;i+=2){
            const wall=bowel[i].geometry,gas=bowel[i+1].geometry;
            for(const name of ['tubeCenter','along','motility'])
                assert.deepEqual(wall.attributes[name].array,gas.attributes[name].array);
            const p=wall.attributes.position,g=gas.attributes.position,c=wall.attributes.tubeCenter;
            for(let j=0;j<p.count;j++){
                const outer=Math.hypot(p.getX(j)-c.getX(j),p.getY(j)-c.getY(j),p.getZ(j)-c.getZ(j));
                const inner=Math.hypot(g.getX(j)-c.getX(j),g.getY(j)-c.getY(j),g.getZ(j)-c.getZ(j));
                assert.ok(inner<=outer+1e-5,'gas must stay inside the bowel wall');
            }
        }
    } finally { tissue.dispose(); }
});
