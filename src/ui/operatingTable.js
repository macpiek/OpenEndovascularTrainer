import * as THREE from 'three';

export function createOperatingTable() {
    const group = new THREE.Group();
    const slideGroup = new THREE.Group();
    group.userData.slideGroup = slideGroup;

    const tabletopMaterial = new THREE.MeshStandardMaterial({ color: 0x9aa4ad, roughness: 0.45 });
    const railMaterial = new THREE.MeshStandardMaterial({ color: 0xd6dde3, roughness: 0.3, metalness: 0.15 });
    const baseMaterial = new THREE.MeshStandardMaterial({ color: 0x59626b, roughness: 0.6, metalness: 0.2 });
    const padMaterial = new THREE.MeshStandardMaterial({ color: 0x263342, roughness: 0.55 });
    const drapeMaterial = new THREE.MeshStandardMaterial({ color: 0x1f7f8a, roughness: 0.7 });
    const skinMaterial = new THREE.MeshStandardMaterial({ color: 0xd8b49b, roughness: 0.65 });

    const foot = new THREE.Mesh(new THREE.BoxGeometry(86, 10, 58), baseMaterial);
    foot.position.set(0, -88, 0);
    group.add(foot);

    const column = new THREE.Mesh(new THREE.CylinderGeometry(8, 10, 76, 32), baseMaterial);
    column.position.set(0, -45, 0);
    group.add(column);

    const pedestal = new THREE.Mesh(new THREE.BoxGeometry(66, 10, 44), baseMaterial);
    pedestal.position.set(0, -8, 0);
    group.add(pedestal);

    const tabletop = new THREE.Mesh(new THREE.BoxGeometry(230, 8, 58), tabletopMaterial);
    tabletop.position.set(0, 0, 0);
    slideGroup.add(tabletop);

    const pad = new THREE.Mesh(new THREE.BoxGeometry(218, 5, 48), padMaterial);
    pad.position.set(0, 6.5, 0);
    slideGroup.add(pad);

    const leftRail = new THREE.Mesh(new THREE.BoxGeometry(224, 3, 3), railMaterial);
    leftRail.position.set(0, 7, -32);
    slideGroup.add(leftRail);

    const rightRail = leftRail.clone();
    rightRail.position.z = 32;
    slideGroup.add(rightRail);

    const body = new THREE.Mesh(new THREE.CapsuleGeometry(17, 64, 10, 22), drapeMaterial);
    body.rotation.z = Math.PI / 2;
    body.position.set(16, 23, 0);
    slideGroup.add(body);

    const chestDrape = new THREE.Mesh(new THREE.BoxGeometry(62, 8, 42), drapeMaterial);
    chestDrape.position.set(14, 20, 0);
    slideGroup.add(chestDrape);

    const head = new THREE.Mesh(new THREE.SphereGeometry(13, 28, 18), skinMaterial);
    head.scale.set(1.05, 0.82, 0.9);
    head.position.set(-50, 21, 0);
    slideGroup.add(head);

    const pillow = new THREE.Mesh(new THREE.BoxGeometry(32, 5, 32), new THREE.MeshStandardMaterial({ color: 0xe8eef2, roughness: 0.75 }));
    pillow.position.set(-50, 13, 0);
    slideGroup.add(pillow);

    const legLeft = new THREE.Mesh(new THREE.CapsuleGeometry(7, 62, 8, 16), drapeMaterial);
    legLeft.rotation.z = Math.PI / 2;
    legLeft.position.set(70, 18, -10);
    slideGroup.add(legLeft);

    const legRight = legLeft.clone();
    legRight.position.z = 10;
    slideGroup.add(legRight);

    const armLeft = new THREE.Mesh(new THREE.CapsuleGeometry(4.5, 58, 8, 14), skinMaterial);
    armLeft.rotation.z = Math.PI / 2;
    armLeft.position.set(4, 17, -31);
    slideGroup.add(armLeft);

    const armRight = armLeft.clone();
    armRight.position.z = 31;
    slideGroup.add(armRight);

    group.add(slideGroup);

    return group;
}
