import test from 'node:test';
import assert from 'node:assert/strict';
import {Vector3} from 'three';
import {fixture,place,finish} from './helpers/stentGraftFixture.js';
import {graftDockingPort} from '../src/devices/stentGraftConnections.js';
import {fabricPoint} from '../src/devices/stentGraftScaffold.js';
import {HybridContrastSystem} from '../src/contrast/hybridContrastSystem.js';
import {previewFixture} from './helpers/stentGraftPreviewFixture.js';

test('II completes its conduit with one contralateral extension and retains its native long outlet',()=>{
    const {system:s,device:body}=previewFixture('right','body',true,'ii-124');
    try {
        s.updateAccess('right',100,null,{deviceId:body.id,release:{sheath:1,tip:true}});
        assert.equal(body.phase,'deployed');assert.equal(s.surface.openGates.length,1);
        assert.equal(s.load('left','limb').ok,true);assert.equal(s.positionAtTarget('left').ok,true);
        const limb=s.accesses.left.device;limb.position=limb.target;
        assert.equal(s.deploy('left').ok,true);assert.equal(limb.parentPartIndex,2);
        s.updateAccess('left',100,null,{deviceId:limb.id,release:'sheath'});
        assert.equal(limb.phase,'deployed');assert.equal(s.surface.sealed,true);
        assert.equal(s.surface.portals.length,3);assert.equal(s.surface.openGates.length,0);
    } finally {s.dispose();}
});

test('IIs accepts both expanding extensions; overlap stays inside parent cloth and only two limbs seal the sac',t=>{
    const f=fixture(),s=f.system;
    try {
        place(s,'right','body');s.deploy('right');finish(s,'right');
        const body=s.implants[0];assert.equal(body.modelId,'iis-103');
        assert.equal(s.surface.openGates.length,2);
        const limbs=[];
        for(const side of ['left','right']) {
            if(side==='right') {
                body.position=body.target=0;
                assert.equal(s.removeDelivery('right').ok,true);
            }
            const port=s.availableGate(side);assert.equal(port.index,side==='right'?1:2);
            place(s,side,'limb');assert.equal(s.deploy(side).ok,true);
            const limb=s.accesses[side].device;assert.equal(limb.parentId,body.id);
            const part=limb.parts[0];
            // Uncover completely, then let the detached rings expand with a
            // stationary sleeve, as when the operator releases the key.
            while(!limb.limbReleased)s.updateAccess(side,1/30,null,{deviceId:limb.id,release:'sheath'});
            for(let i=0;i<300&&limb.phase!=='deployed';i++)s.updateAccess(side,1/30);
            assert.equal(limb.phase,'deployed');assert.ok(part.exposure.every(v=>v===1));
            const p=part.mesh.geometry.attributes.position;
            let overlapError=0;
            for(let i=0;i<p.count;i++) {
                const q=new Vector3().fromBufferAttribute(p,i),fitted=limb.wallFit.fitOverlap(q.clone());
                overlapError=Math.max(overlapError,q.distanceTo(fitted));
            }
            assert.ok(overlapError<.03,`extension crosses parent cloth by ${overlapError} mm`);
            let metalError=0,metalWorst=null;
            const metal=part.rings.geometry.attributes.instanceStart;
            for(let i=0;i<metal.count;i++) {
                const q=new Vector3().fromBufferAttribute(metal,i);
                const error=q.distanceTo(limb.wallFit.fitOverlap(q.clone()));
                if(error>metalError){metalError=error;metalWorst={side,i,p:q.toArray(),ring:Math.floor(i/288)};}
            }
            assert.ok(metalError<.1,`extension wire crosses parent cloth by ${metalError} mm: ${JSON.stringify(metalWorst)}`);
            const ends=part.rings.geometry.attributes.instanceEnd;let firstSegment=0;
            for(const ring of part.scaffoldRings) {
                let length=0;
                for(let i=firstSegment;i<firstSegment+ring.samples;i++)length+=new Vector3().fromBufferAttribute(metal,i).distanceTo(new Vector3().fromBufferAttribute(ends,i));
                assert.ok(Math.abs(length/ring.restLength-1)<1e-4,'parent contact cannot shorten the metal');
                firstSegment+=ring.samples;
            }
            const row=Math.min(2,part.rows-1),center=new Vector3();
            for(let j=0;j<part.sides;j++)center.add(new Vector3().fromBufferAttribute(p,row*part.sides+j));
            center.divideScalar(part.sides);
            let radius=0;for(let j=0;j<part.sides;j++)radius+=new Vector3().fromBufferAttribute(p,row*part.sides+j).distanceTo(center)/part.sides;
            limbs.push({side,radiusMm:radius,clothCrossingMm:overlapError,metalCrossingMm:metalError});
            assert.ok(radius>3.5,`released proximal limb remains crimped: radius ${radius}`);
            assert.equal(s.surface.sealed,side==='right');
            assert.equal(s.surface.openGates.length,side==='right'?0:1);
            assert.equal(s.availableGate(side),null);
            const snapshot=s.surface.paths.find(entry=>entry.part===part);
            assert.deepEqual(snapshot.positions,part.contactBasePositions??p.array,'flow uses the accepted expanded cloth');
        }
        assert.equal(s.surface.portals.length,3,'one inlet and two extended outlets');
        const part=body.parts[0],marker=part.flowDividerMarker,layout=part.markerLayout.find(m=>m.role==='flow-divider');
        assert.equal(marker.name,'flow-divider-marker');assert.equal(marker.visible,true);
        const b=marker.geometry.attributes.instanceEnd;
        assert.ok(new Vector3().fromBufferAttribute(b,0).distanceTo(fabricPoint(part,layout.end,layout.angle))<1e-4);
        assert.ok(marker.userData.projectionMaterial,'divider has its own radiopaque projection');
        assert.ok(graftDockingPort(body,1,s.routes.right).entry.distanceTo(graftDockingPort(body,2,s.routes.left).entry)>1);
        const h=new HybridContrastSystem({centerlineSegments:f.segments,contactField:f.contactField}),n=h.flowNetwork;
        h.setStentGraftSurface(s.surface);
        n.depositIodine(n.edges.find(e=>e.graftCovered&&!e.transportExcluded).index,0,100);
        for(let i=0;i<600;i++)n.update(1/30);
        assert.ok(n.totalIodineMassMg<.1,`sealed conduit retains ${n.totalIodineMassMg} mg`);
        assert.ok(Math.abs(n.totalIodineMassMg+n.outletIodineMassMg+n.stentGraftRemodeling.trappedIodineMassMg-100)<1e-7);
        t.diagnostic(JSON.stringify({limbs,remainingIodineMg:n.totalIodineMassMg,outletIodineMg:n.outletIodineMassMg}));
    } finally {f.dispose();}
});
