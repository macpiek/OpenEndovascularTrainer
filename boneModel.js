import * as THREE from 'three';

export function createBoneModel() {
    // Use a white material so bones appear bright in fluoroscopy rendering
    const material = new THREE.MeshStandardMaterial({ color: 0xffffff });
    const group = new THREE.Group();

    // Approximate pelvis using two hip boxes and a central sacrum
    const hipGeom = new THREE.BoxGeometry(80, 100, 40);
    const leftHip = new THREE.Mesh(hipGeom, material);
    leftHip.position.set(-60, -50, 0);
    leftHip.rotation.z = THREE.MathUtils.degToRad(20);
    group.add(leftHip);

    const rightHip = new THREE.Mesh(hipGeom, material);
    rightHip.position.set(60, -50, 0);
    rightHip.rotation.z = THREE.MathUtils.degToRad(-20);
    group.add(rightHip);

    const sacrum = new THREE.Mesh(new THREE.CylinderGeometry(30, 40, 100, 16), material);
    sacrum.position.set(0, -50, 0);
    group.add(sacrum);

    // Spine represented by a vertical cylinder emerging from the origin
    const spine = new THREE.Mesh(new THREE.CylinderGeometry(20, 20, 400, 16), material);
    spine.position.y = 200; // height 400 -> base at y=0
    group.add(spine);

    return group;
}

