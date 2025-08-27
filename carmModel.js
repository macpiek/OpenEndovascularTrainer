import * as THREE from 'three';

export function createCArmModel() {
    const material = new THREE.MeshStandardMaterial({ color: 0x888888 });

    const group = new THREE.Group();

    // Position the base and supporting column to the side so the gantry
    // (which is centred on the group's origin) sits over the patient/table.
    const base = new THREE.Mesh(new THREE.BoxGeometry(40, 10, 40), material);
    base.position.set(-80, 5, 0);
    group.add(base);

    const column = new THREE.Mesh(new THREE.CylinderGeometry(5, 5, 60, 32), material);
    column.position.set(-80, 40, 0);
    group.add(column);

    // Simple horizontal arm connecting the column to the gantry ring
    const arm = new THREE.Mesh(new THREE.BoxGeometry(80, 5, 5), material);
    arm.position.set(-40, 70, 0);
    group.add(arm);

    const gantryGroup = new THREE.Group();
    gantryGroup.position.set(0, 70, 0);
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

    return { group, gantryGroup };
}

