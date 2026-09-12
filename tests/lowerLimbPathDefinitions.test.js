import assert from 'node:assert/strict';
import * as THREE from 'three';
import {
    createLowerLimbArteryPaths,
    LOWER_LIMB_TOE_DISTAL_Z_MM
} from '../src/lowerLimbArteries.js';

const roots = [
    {
        anatomicalSide: 'right',
        sideSign: -1,
        point: new THREE.Vector3(-96.233116, -484.312012, 10.430171),
        radius: 3.04225,
        outward: new THREE.Vector3(-0.24, -0.96, 0.21).normalize()
    },
    {
        anatomicalSide: 'left',
        sideSign: 1,
        point: new THREE.Vector3(68.95491, -484.334961, 13.197962),
        radius: 2.96521,
        outward: new THREE.Vector3(0.24, -0.96, 0.21).normalize()
    }
];
const paths = createLowerLimbArteryPaths(roots);

function vessel(side, suffix) {
    return paths.find(path => path.name === `${side}-${suffix}`);
}

function samePosition(a, b, message) {
    assert.ok(
        a.position.distanceTo(b.position) < 1e-9,
        `${message}: ${a.position.toArray()} / ${b.position.toArray()}`
    );
}

for (const side of ['right', 'left']) {
    const femoral = vessel(side, 'femoral-popliteal');
    const deepFemoral = vessel(side, 'deep-femoral');
    const anterior = vessel(side, 'anterior-tibial');
    const trunk = vessel(side, 'tibioperoneal-trunk');
    const posterior = vessel(side, 'posterior-tibial');
    const fibular = vessel(side, 'fibular');
    const lateralPlantar = vessel(side, 'lateral-plantar');
    const plantarArch = vessel(side, 'deep-plantar-arch');
    const medialPlantar = vessel(side, 'medial-plantar');
    const lateralCircumflex = vessel(side, 'lateral-circumflex-femoral');
    const medialCircumflex = vessel(side, 'medial-circumflex-femoral');
    const perforating1 = vessel(side, 'perforating-femoral-1');
    const perforating2 = vessel(side, 'perforating-femoral-2');
    const descendingGenicular = vessel(side, 'descending-genicular');
    const superiorMedialGenicular = vessel(side, 'superior-medial-genicular');
    const inferiorLateralGenicular = vessel(side, 'inferior-lateral-genicular');
    const anteriorTibialRecurrent = vessel(side, 'anterior-tibial-recurrent');

    assert.ok(
        femoral && deepFemoral && anterior && trunk && posterior && fibular &&
        lateralPlantar && plantarArch && medialPlantar
    );
    assert.ok(
        deepFemoral.points[2].position.y >= -538,
        `${side} deep femoral artery should originate high in the proximal thigh`
    );

    samePosition(
        femoral.points.at(-1),
        anterior.points[1],
        `${side} anterior tibial origin`
    );
    samePosition(
        femoral.points.at(-1),
        trunk.points[1],
        `${side} tibioperoneal origin`
    );
    samePosition(
        trunk.points.at(-1),
        posterior.points[1],
        `${side} posterior tibial split`
    );
    samePosition(
        trunk.points.at(-1),
        fibular.points[1],
        `${side} fibular split`
    );
    samePosition(
        posterior.points.at(-1),
        lateralPlantar.points[0],
        `${side} lateral plantar origin`
    );
    samePosition(
        posterior.points.at(-1),
        medialPlantar.points[0],
        `${side} medial plantar origin`
    );
    samePosition(
        anterior.points.at(-1),
        plantarArch.points[0],
        `${side} deep plantar origin from dorsalis pedis`
    );
    samePosition(
        lateralPlantar.points.at(-1),
        plantarArch.points.at(-1),
        `${side} lateral end of plantar arch`
    );
    samePosition(
        deepFemoral.points[2],
        lateralCircumflex.points[0],
        `${side} lateral circumflex femoral origin`
    );
    samePosition(
        deepFemoral.points[2],
        medialCircumflex.points[0],
        `${side} medial circumflex femoral origin`
    );
    samePosition(
        deepFemoral.points[3],
        perforating1.points[0],
        `${side} first perforating femoral origin`
    );
    samePosition(
        deepFemoral.points[4],
        perforating2.points[0],
        `${side} second perforating femoral origin`
    );
    samePosition(
        femoral.points[4],
        descendingGenicular.points[0],
        `${side} descending genicular origin`
    );
    samePosition(
        femoral.points[6],
        superiorMedialGenicular.points[0],
        `${side} superior genicular origin`
    );
    samePosition(
        femoral.points[7],
        inferiorLateralGenicular.points[0],
        `${side} inferior genicular origin`
    );
    samePosition(
        anterior.points[2],
        anteriorTibialRecurrent.points[0],
        `${side} anterior tibial recurrent origin`
    );

    const division = femoral.points.at(-1).position;
    const anteriorDirection = anterior.points[2].position
        .clone()
        .sub(division);
    const trunkDirection = trunk.points[2].position
        .clone()
        .sub(division);
    const branchingAngleDegrees = THREE.MathUtils.radToDeg(
        anteriorDirection.angleTo(trunkDirection)
    );
    assert.ok(
        branchingAngleDegrees >= 50,
        `${side} anterior tibial artery should leave at a full angle (${branchingAngleDegrees}°)`
    );

    const anteriorCalf = anterior.points[3].position;
    const posteriorCalf = posterior.points[2].position;
    const fibularCalf = fibular.points[2].position;
    assert.ok(
        anteriorCalf.z >= posteriorCalf.z + 45,
        `${side} anterior tibial artery should pass anterior to the deep posterior vessels`
    );
    if (side === 'right') {
        assert.ok(
            posteriorCalf.x > fibularCalf.x,
            'right posterior tibial artery should be medial to the fibular artery'
        );
    } else {
        assert.ok(
            posteriorCalf.x < fibularCalf.x,
            'left posterior tibial artery should be medial to the fibular artery'
        );
    }

    const dorsalHindfoot = anterior.points.at(-3).position;
    const dorsalForefoot = anterior.points.at(-1).position;
    assert.ok(
        dorsalForefoot.z >= dorsalHindfoot.z + 55,
        `${side} dorsalis pedis should follow the long axis of the foot`
    );
    assert.ok(
        dorsalForefoot.y < dorsalHindfoot.y,
        `${side} dorsalis pedis should follow the distal slope of the dorsal foot`
    );

    const archPoints = plantarArch.points.slice(2).map(item => item.position);
    const archXSpan = Math.max(...archPoints.map(item => item.x)) -
        Math.min(...archPoints.map(item => item.x));
    const archZSpan = Math.max(...archPoints.map(item => item.z)) -
        Math.min(...archPoints.map(item => item.z));
    assert.ok(
        archXSpan >= 45,
        `${side} plantar arch should cross the forefoot transversely`
    );
    assert.ok(
        archZSpan <= 5,
        `${side} plantar arch should remain at the metatarsal-base level`
    );
    assert.ok(
        plantarArch.points[1].position.y <= dorsalForefoot.y - 30,
        `${side} deep plantar branch should descend from dorsum to sole`
    );

    const metatarsalArchIndices = [3, 4, 6, 7];
    const properDigitalPaths = [];
    metatarsalArchIndices.forEach((archIndex, index) => {
        const metatarsal = vessel(side, `plantar-metatarsal-${index + 1}`);
        samePosition(
            plantarArch.points[archIndex],
            metatarsal.points[0],
            `${side} plantar metatarsal ${index + 1} origin`
        );
        assert.equal(metatarsal.terminal, false);

        const lateral = vessel(
            side,
            `proper-plantar-digital-${index + 1}-lateral`
        );
        const medial = vessel(
            side,
            `proper-plantar-digital-${index + 2}-medial`
        );
        for (const proper of [lateral, medial]) {
            samePosition(
                metatarsal.points.at(-1),
                proper.points[0],
                `${side} ${proper.name} origin`
            );
            assert.equal(proper.terminal, true);
            properDigitalPaths.push(proper);
        }
    });

    const halluxMedial = vessel(side, 'proper-plantar-digital-1-medial');
    const fifthLateral = vessel(side, 'proper-plantar-digital-5-lateral');
    samePosition(
        medialPlantar.points.at(-1),
        halluxMedial.points[0],
        `${side} medial hallux artery origin`
    );
    samePosition(
        plantarArch.points.at(-1),
        fifthLateral.points[0],
        `${side} lateral fifth-toe artery origin`
    );
    properDigitalPaths.push(halluxMedial, fifthLateral);
    assert.equal(properDigitalPaths.length, 10);
    assert.ok(
        properDigitalPaths.filter(path =>
            path.points.at(-1).position.z === LOWER_LIMB_TOE_DISTAL_Z_MM
        ).length >= 2,
        `${side} hallux vessels should reach the distal end of the great toe`
    );
}

assert.equal(paths.length, 66, 'thirty-three named arterial paths should be generated per leg');
console.log('lower-limb skeletal path definitions passed');
