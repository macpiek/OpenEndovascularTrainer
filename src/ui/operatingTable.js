import * as THREE from 'three';

export function createOperatingTable() {
    const group = new THREE.Group();

    const tabletopMaterial = new THREE.MeshStandardMaterial({ color: 0x777777 });
    const baseMaterial = new THREE.MeshStandardMaterial({ color: 0x555555 });

    const foot = new THREE.Mesh(new THREE.BoxGeometry(60, 10, 60), baseMaterial);
    foot.position.y = 5;
    group.add(foot);

    const column = new THREE.Mesh(new THREE.CylinderGeometry(5, 5, 60, 32), baseMaterial);
    column.position.y = 40; // foot height 10 -> top at 10, column center at 10 + 30 = 40
    group.add(column);

    const tabletop = new THREE.Mesh(new THREE.BoxGeometry(200, 10, 60), tabletopMaterial);
    tabletop.position.y = 75; // foot 10 + column 60 + tabletop thickness / 2 (5)
    group.add(tabletop);

    return group;
}

