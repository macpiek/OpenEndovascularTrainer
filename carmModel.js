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

    // Internal group orienting the gantry so the X-ray beam points toward
    // the negative Y-axis when no additional rotations are applied.
    const gantryModel = new THREE.Group();
    gantryModel.rotation.x = Math.PI / 2;
    gantryGroup.add(gantryModel);

    const gantryGeometry = new THREE.TorusGeometry(40, 3, 16, 100, Math.PI * 1.5);
    const gantry = new THREE.Mesh(gantryGeometry, material);
    // Rotate so the open side of the torus faces the patient.
    gantry.rotation.z = Math.PI / 2;
    gantryModel.add(gantry);

    // Position the source below and the detector above the isocenter so the
    // detector (intensifier) faces downward along the -Y axis.
    const source = new THREE.Mesh(new THREE.BoxGeometry(8, 8, 4), material);
    source.position.set(0, 0, 40);
    gantryModel.add(source);

    const detector = new THREE.Mesh(new THREE.BoxGeometry(8, 8, 4), material);
    detector.position.set(0, 0, -40);
    gantryModel.add(detector);

    return { group, gantryGroup };
}

