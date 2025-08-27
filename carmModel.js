import * as THREE from 'three';

export function createCArmModel() {
    const material = new THREE.MeshStandardMaterial({ color: 0x888888 });

    const group = new THREE.Group();

    const base = new THREE.Mesh(new THREE.BoxGeometry(40, 10, 40), material);
    base.position.y = 5;
    group.add(base);

    const column = new THREE.Mesh(new THREE.CylinderGeometry(5, 5, 60, 32), material);
    column.position.y = 40;
    group.add(column);

    const gantryGroup = new THREE.Group();
    gantryGroup.position.y = 70;
    group.add(gantryGroup);

    const gantryGeometry = new THREE.TorusGeometry(40, 3, 16, 100, Math.PI * 1.5);
    const gantry = new THREE.Mesh(gantryGeometry, material);
    // Rotate the gantry so it stands vertically beside the operating table
    gantry.rotation.set(Math.PI / 2, 0, Math.PI / 2);
    gantryGroup.add(gantry);

    const source = new THREE.Mesh(new THREE.BoxGeometry(8, 8, 4), material);
    source.position.set(0, 0, -40);
    gantryGroup.add(source);

    const detector = new THREE.Mesh(new THREE.BoxGeometry(8, 8, 4), material);
    detector.position.set(0, 0, 40);
    gantryGroup.add(detector);

    return group;
}

