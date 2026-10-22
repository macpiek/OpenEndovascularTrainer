import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import * as THREE from 'three';
import { decodeCollisionAsset } from '../src/physics/collision/collisionAssetFormat.js';
import { HybridContrastSystem } from '../src/contrast/hybridContrastSystem.js';
import {buildJunctionRegions} from '../src/contrast/junctionContrastVolume.js';

function system(name='Aorta_infrarenal_aneurysm') {
    const bytes=fs.readFileSync(new URL(`../res/${name}.collision.bin`,import.meta.url));
    const asset=decodeCollisionAsset(bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength));
    const data=asset.arrays.centerlineSegments,links=asset.arrays.centerlineEdges;
    const segments=Array.from({length:links.length/2},(_,id)=>({
        id,nodeStartId:links[id*2],nodeEndId:links[id*2+1],
        start:new THREE.Vector3().fromArray(data,id*9),end:new THREE.Vector3().fromArray(data,id*9+3),
        radiusStart:data[id*9+6],radiusEnd:data[id*9+7],safeRadius:data[id*9+8]
    }));
    for(const r of asset.metadata.renal?.segments??[])Object.assign(segments[r.index],{
        renalSide:r.side,renalSector:r.sector,renalHilar:r.hilar,renalLateral:r.lateral
    });
    return new HybridContrastSystem({centerlineSegments:segments,localOptions:{capacity:16}});
}
function ostia(net) {
    // The two affected side branches in the supplied aneurysm atlas. Select
    // anatomically, without depending on the flow graph's edge numbering.
    return net.aorticBranchPrefixDiagnostics.paths.filter((path) => {
        const root = net.edges[path.rootEdgeIndex];
        return root.start.y < -253 && root.start.y > -254;
    });
}
function pathMass(net, paths) {
    return paths.reduce(
        (total, path) =>
            total +
            path.edgeIndices.reduce(
                (sum, i) => sum + net.edges[i].massMg.reduce((a, b) => a + b, 0),
                0
            ),
        0
    );
}

test('aneurysm ostia use downstream branch calibre, including a repaired parent-radius valley', () => {
    const net = system().flowNetwork;
    const paths = ostia(net);
    assert.equal(paths.length, 2, 'both enlarged technical branch prefixes must be corrected');
    assert.ok(
        paths.some(
            (path) => net.edges[net.edges[path.rootEdgeIndex].parentEdgeIndex].rawRadiusEnd < 10
        ),
        'the test must cover the raw-radius valley which used to bypass correction'
    );
    for (const path of paths) {
        assert.ok(path.branchRadiusMm > 2 && path.branchRadiusMm < 4);
        assert.ok(path.removedVolumeMm3 > 3000, 'do not leave a false multi-millilitre reservoir');
        const root = net.edges[path.rootEdgeIndex];
        assert.ok(root.radiusStart <= path.branchRadiusMm * 1.35 + 1e-6);
        assert.ok(root.meanFlowMm3PerS > 0, 'the branch must remain patent');
    }
    const corrected = new Set(paths.flatMap((path) => path.edgeIndices));
    const trunk = net.edges.filter(
        (e) => e.start.y < -255 && e.start.y > -270 && e.axis.y < -0.95 && e.rawRadiusStart > 8
    );
    assert.ok(trunk.length > 20);
    for (const edge of trunk) {
        assert.ok(!corrected.has(edge.index), 'do not constrict the continuing aortic trunk');
        assert.ok(edge.radiusStart >= edge.rawRadiusStart);
    }
});

test('a finite bolus traverses the aneurysm ostia and washes out without losing iodine', () => {
    const s = system(),
        net = s.flowNetwork,
        paths = ostia(net);
    assert.equal(paths.length, 2);
    const source = net.findNearestLocation(new THREE.Vector3(3, -212, 0), {});
    let peakMass = 0;
    // The continuous-chain dispersion model broadens transit through the
    // upstream sac. Isolated local ostial washout is checked at 6 s below;
    // allow the full upstream bolus to arrive before checking total clearance.
    for (let frame = 0; frame < 1200; frame++) {
        if (frame < 30) net.depositIodine(source.edgeIndex, source.cellIndex, 20);
        s.update(1 / 60);
        peakMass = Math.max(peakMass, pathMass(net, paths));
        const injected = Math.min(frame + 1, 30) * 20;
        assert.ok(Math.abs(net.totalIodineMassMg + net.outletIodineMassMg - injected) < 1e-7);
    }
    assert.ok(peakMass > 10, 'the bolus must actually enter both branches');
    assert.ok(
        pathMass(net, paths) < 0.6,
        'less than 0.1% may remain after the dispersed bolus has traversed the upstream aneurysm (20 s)'
    );
});

for(const name of ['Aorta_plain','Aorta_plain_renal','Aorta_infrarenal_aneurysm_renal'])
    test(`${name}: side ostia clear a finite bolus without narrowing the aortoiliac fork`,()=>{
        const s=system(name),net=s.flowNetwork,paths=ostia(net);
        assert.equal(paths.length,2,'correct both side ostia in the deployed anatomy too');
        for(const path of paths){
            const root=net.edges[path.rootEdgeIndex];
            assert.ok(root.radiusStart<=path.branchRadiusMm*1.35+1e-6);
            assert.ok(path.removedVolumeMm3>40,'remove the oversized technical pocket');
            assert.ok(root.meanFlowMm3PerS>0,'keep perfusion of the side branch');
            for(const index of path.edgeIndices){
                const e=net.edges[index];
                assert.ok(e.lumenMappingUsesCorrectedRadius,'port mapping must use the corrected ostium');
                assert.ok(e.radiusStart<=e.rawRadiusStart+1e-6);
            }
        }
        const corrected=new Set(net.aorticBranchPrefixDiagnostics.paths.flatMap(p=>p.edgeIndices));
        const fork=net.edges.find(e=>e.end.y< -291&&e.end.y> -292&&e.childEdgeIndices.length===2&&e.radiusEnd>6);
        assert.ok(fork,'locate the real aortoiliac bifurcation');
        for(const index of fork.childEdgeIndices){
            let e=net.edges[index],length=0;
            while(e&&length<25){
                assert.ok(!corrected.has(e.index),'do not taper either main iliac artery as a side ostium');
                length+=e.length;
                if(e.childEdgeIndices.length!==1)break;
                e=net.edges[e.childEdgeIndices[0]];
            }
        }
        // Isolate ostial clearance from the separate upstream aneurysm transit.
        const roots=paths.map(p=>net.edges[p.rootEdgeIndex]);
        let peak=0,injected=0;
        for(let frame=0;frame<360;frame++){
            net.clearFlowOverrides();
            if(frame<30)for(const root of roots){
                const flow=root.meanFlowMm3PerS;
                net._branchInletFlowMm3PerS[root.index]=flow;
                net._branchInletConcentrationMgPerMm3[root.index]=.3;
                net._branchInletEdgeIndices.add(root.index);
                injected+=flow*.3/60;
            }
            net.update(1/60);
            peak=Math.max(peak,pathMass(net,paths));
            assert.ok(Math.abs(net.totalIodineMassMg+net.outletIodineMassMg-injected)<1e-7,
                'clearance must conserve iodine rather than delete it');
        }
        assert.ok(peak>.01,'the bolus must reach the corrected ostia');
        assert.ok(pathMass(net,paths)<Math.min(.6,peak*.01),'clear over 99% of the ostial peak by six seconds');
    });

for(const name of ['Aorta_plain_renal','Aorta_infrarenal_aneurysm_renal'])
    test(`${name}: iliac side ostia keep the main limb, join continuously and share optical volume`,()=>{
        const s=system(name),net=s.flowNetwork;
        const forks=net.edges.filter(e=>e.end.y< -325&&e.end.y> -336&&e.childEdgeIndices.length===2&&e.radiusEnd>3);
        assert.equal(forks.length,2,'cover both iliac origins');
        const regions=buildJunctionRegions(net),paths=[];
        for(const fork of forks){
            const children=fork.childEdgeIndices.map(i=>net.edges[i]);
            const main=children.reduce((a,b)=>a.meanFlowMm3PerS>b.meanFlowMm3PerS?a:b);
            const smoothing=net.sideJunctionRadiusDiagnostics.find(p=>p.parentEdgeIndex===fork.index);
            assert.ok(smoothing?.removedVolumeMm3>0,'remove the spurious shared-mouth radius peaks');
            const trunk=smoothing.edgeIndices.map(i=>net.edges[i]);
            assert.equal(trunk[0].radiusStart,smoothing.startRadius,'preserve the upstream lumen');
            assert.equal(trunk.at(-1).radiusEnd,smoothing.endRadius,'preserve the downstream lumen');
            for(let i=1;i<trunk.length;i++)
                assert.ok(Math.abs(trunk[i-1].radiusEnd-trunk[i].radiusStart)<1e-6,
                    'the shared mouth and continuing artery must have continuous calibre');
            assert.ok(!net.aorticBranchPrefixDiagnostics.paths.some(p=>p.rootEdgeIndex===main.index),
                'a raw radius valley must not shrink the continuing iliac limb');
            const side=children.find(e=>e!==main);
            const path=net.aorticBranchPrefixDiagnostics.paths.find(p=>p.rootEdgeIndex===side.index);
            assert.ok(path,'correct the smaller iliac ostium');paths.push(path);
            assert.ok(side.radiusStart<=path.branchRadiusMm*1.15+1e-6,'avoid a bulbous side-branch mouth');
            const last=net.edges[path.edgeIndices.at(-1)],next=net.edges[last.childEdgeIndices[0]];
            assert.ok(next,'the corrected prefix must lead to a patent artery');
            assert.ok(Math.abs(last.radiusEnd-next.radiusStart)<1e-6,'no artificial calibre jump after the ostium');
            assert.ok(regions.some(r=>r.edgeIndices.has(fork.index)&&children.every(e=>r.edgeIndices.has(e.index))),
                'both limbs and the shared iliac junction must use one optical union');
        }
        let injected=0,peak=0;
        for(let frame=0;frame<360;frame++){
            net.clearFlowOverrides();
            if(frame<30)for(const p of paths){
                const root=net.edges[p.rootEdgeIndex],flow=root.meanFlowMm3PerS;
                net._branchInletFlowMm3PerS[root.index]=flow;
                net._branchInletConcentrationMgPerMm3[root.index]=.3;
                net._branchInletEdgeIndices.add(root.index);injected+=flow*.3/60;
            }
            net.update(1/60);peak=Math.max(peak,pathMass(net,paths));
            assert.ok(Math.abs(net.totalIodineMassMg+net.outletIodineMassMg-injected)<1e-7);
        }
        assert.ok(peak>1,'the bolus must actually fill the iliac side branches');
        assert.ok(pathMass(net,paths)<peak*.01,'iliac ostia must clear over 99% of their peak without deleting iodine');
    });

for (const name of ['Aorta_plain_renal', 'Aorta_infrarenal_aneurysm_renal'])
    test(`${name}: a sheath bolus cannot pool in a terminal strand inside the iliac lumen`, () => {
        const s = system(name), net = s.flowNetwork;
        const duplicate = net.containedTerminalSpurDiagnostics.roots.find(p => {
            const start = net.edges[p.rootEdgeIndex].start;
            return start.distanceTo(new THREE.Vector3(-18.189, -334.277, -11.403)) < 0.1;
        });
        assert.ok(duplicate, 'recognize the short strand responsible for the marked dark spot');
        const main = net.edges[duplicate.mainContinuationEdgeIndex];
        assert.ok(!main.transportExcluded && main.meanFlowMm3PerS > 0, 'preserve the real artery');
        for (const i of duplicate.edgeIndices) {
            const e = net.edges[i];
            assert.ok(e.transportExcluded && e.renderExcluded && e.lumenMappingExcluded);
            const location = net.findNearestLocation(e.start.clone().lerp(e.end, 0.5), {});
            assert.ok(!duplicate.edgeIndices.includes(location.edgeIndex),
                'the anatomical renderer must sample the continuing lumen instead of the duplicate');
        }
        assert.ok(net.edges.filter(e => e.renalSide).every(e => !e.transportExcluded),
            'preserve all real renal branches');
        const location = net.findNearestLocation(new THREE.Vector3(-73, -383, 14), {});
        const source = net.edges[location.edgeIndex], tip = source.start.clone().lerp(source.end, location.t);
        s.setAccess({start: tip.clone().addScaledVector(source.axis, 70), end: tip, innerRadius: 0.9}, null);
        assert.ok(s.startInjection({source: 'sheath', volumeMl: 10, rateMlPerSec: 15}).ok);
        let peak = 0;
        for (let frame = 0; frame < 360; frame++) {
            s.advanceSynchronized(1 / 60);
            peak = Math.max(peak, ...main.massMg);
            assert.ok(Math.abs(s.getMetrics().balanceErrorMg) < 1e-6, 'conserve the entire bolus');
        }
        assert.ok(peak > 1, 'contrast must pass through the real artery');
        for (const i of duplicate.edgeIndices) assert.equal(net.edges[i].massMg.reduce((a,b) => a+b, 0), 0);
        assert.ok(main.massMg.reduce((a,b) => a+b, 0) < peak * 0.01,
            'the real origin must clear over 99% by six seconds after the retrograde bolus');
    });

for (const name of ['Aorta_plain_renal', 'Aorta_infrarenal_aneurysm_renal']) {
    test(`${name}: all arch mouths and shared stems clear a finite bolus`, () => {
        const s = system(name), net = s.flowNetwork;
        const connectors = net.intraluminalAorticConnectorDiagnostics;
        const paths = net.aorticBranchPrefixDiagnostics.paths.filter(p =>
            net.edges[p.rootEdgeIndex].start.y > 0 &&
            net.edges[net.edges[p.rootEdgeIndex].parentEdgeIndex].intraluminalAorticConnector);
        assert.equal(paths.length, 6, 'include the small terminal routes as well as three major branches');
        assert.ok(connectors.sharedStems.length > 10, 'repair reservoirs upstream of nested connectors');
        const major = paths.filter(p => net.nodes.get(net.edges[p.rootEdgeIndex].endNodeId).terminalOutletCount > 3);
        assert.equal(major.length, 3, 'preserve all three major arch branches');
        const tracked = paths.map(p => p.edgeIndices);
        tracked.push(connectors.sharedStems.map(p => p.edgeIndex).filter(i => net.edges[i].start.y > 0));
        const peaks = tracked.map(() => 0);
        const mass = indices => indices.reduce((sum, i) =>
            sum + net.edges[i].massMg.reduce((a, b) => a + b, 0), 0);
        let source = net.edges[connectors.connectors.find(c => net.edges[c.parentEdgeIndex].start.y > 0).parentEdgeIndex], upstream = 0;
        while (source.parentEdgeIndex >= 0 && upstream < 20) {
            upstream += source.length;
            source = net.edges[source.parentEdgeIndex];
        }
        for (let frame = 0; frame < 720; frame++) {
            if (frame < 30) {
                net.depositIodine(source.index, 0, 20);
                s.totalInjectedIodineMassMg += 20;
            }
            s.update(1 / 60);
            tracked.forEach((indices, i) => peaks[i] = Math.max(peaks[i], mass(indices)));
            assert.ok(Math.abs(s.getMetrics().balanceErrorMg) < 1e-7, 'conserve iodine including renal uptake');
            if (frame === 359) tracked.forEach((indices, i) => {
                assert.ok(peaks[i] > 1, 'the bolus must actually enter every tested route');
                assert.ok(mass(indices) < peaks[i] * .02, `route ${i}: less than 2% remains at 6 s`);
            });
        }
        tracked.forEach(indices => assert.ok(mass(indices) < 1e-4, 'no residual plug at 12 s'));
        assert.ok(net.edges.filter(e => e.renalSide).every(e => !e.transportExcluded));
    });
}
