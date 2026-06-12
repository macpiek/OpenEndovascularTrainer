import * as THREE from 'three';

function box(width, height, depth, material, position, rotation = new THREE.Euler()) {
    const mesh = new THREE.Mesh(new THREE.BoxGeometry(width, height, depth), material);
    mesh.position.copy(position);
    mesh.rotation.copy(rotation);
    return mesh;
}

function cylinder(radiusTop, radiusBottom, height, material, position, rotation = new THREE.Euler(), radialSegments = 40) {
    const mesh = new THREE.Mesh(new THREE.CylinderGeometry(radiusTop, radiusBottom, height, radialSegments), material);
    mesh.position.copy(position);
    mesh.rotation.copy(rotation);
    return mesh;
}

function capsule(radius, length, material, position, rotation = new THREE.Euler()) {
    const mesh = new THREE.Mesh(new THREE.CapsuleGeometry(radius, length, 10, 24), material);
    mesh.position.copy(position);
    mesh.rotation.copy(rotation);
    return mesh;
}

function arcTube(radius, startDeg, endDeg, z, tubeRadius, material, xOffset = 0) {
    const points = [];
    for (let i = 0; i <= 96; i++) {
        const angle = THREE.MathUtils.degToRad(startDeg + ((endDeg - startDeg) * i) / 96);
        points.push(new THREE.Vector3(
            xOffset + radius * Math.cos(angle),
            radius * Math.sin(angle),
            z
        ));
    }
    const curve = new THREE.CatmullRomCurve3(points);
    return new THREE.Mesh(new THREE.TubeGeometry(curve, 128, tubeRadius, 18, false), material);
}

export function createCArmModel() {
    const group = new THREE.Group();
    const gantryGroup = new THREE.Group();

    const shell = new THREE.MeshStandardMaterial({
        color: 0xdde5ea,
        roughness: 0.42,
        metalness: 0.08
    });
    const whiteShell = new THREE.MeshStandardMaterial({
        color: 0xf2f5f6,
        roughness: 0.36,
        metalness: 0.04
    });
    const metal = new THREE.MeshStandardMaterial({
        color: 0x9daab4,
        roughness: 0.48,
        metalness: 0.28
    });
    const darkMetal = new THREE.MeshStandardMaterial({
        color: 0x48535d,
        roughness: 0.62,
        metalness: 0.25
    });
    const rubber = new THREE.MeshStandardMaterial({
        color: 0x151b20,
        roughness: 0.75
    });
    const detectorMaterial = new THREE.MeshStandardMaterial({
        color: 0xe9f0f3,
        roughness: 0.34
    });
    const detectorFaceMaterial = new THREE.MeshBasicMaterial({ color: 0xc2e8f7 });
    const sourceMaterial = new THREE.MeshStandardMaterial({
        color: 0xe4e9ec,
        roughness: 0.4,
        metalness: 0.06
    });
    const beamMaterial = new THREE.MeshBasicMaterial({
        color: 0x91ddff,
        transparent: true,
        opacity: 0.16,
        depthWrite: false
    });
    const isoMaterial = new THREE.MeshBasicMaterial({ color: 0x42d7ff });

    // Mobile base and cabinet, kept stationary while the gantry rotates.
    group.add(box(105, 12, 54, rubber, new THREE.Vector3(-118, -91, -84)));
    group.add(box(68, 62, 58, shell, new THREE.Vector3(-116, -56, -84)));
    group.add(box(70, 18, 58, darkMetal, new THREE.Vector3(-126, -90, -84)));
    group.add(cylinder(13, 13, 10, rubber, new THREE.Vector3(-164, -94, -110), new THREE.Euler(Math.PI / 2, 0, 0), 32));
    group.add(cylinder(13, 13, 10, rubber, new THREE.Vector3(-84, -94, -110), new THREE.Euler(Math.PI / 2, 0, 0), 32));
    group.add(cylinder(10, 10, 8, rubber, new THREE.Vector3(-164, -94, -58), new THREE.Euler(Math.PI / 2, 0, 0), 32));

    const column = cylinder(12, 15, 102, metal, new THREE.Vector3(-116, -9, -84));
    group.add(column);
    group.add(box(58, 22, 50, shell, new THREE.Vector3(-116, 50, -84)));
    group.add(box(24, 10, 44, darkMetal, new THREE.Vector3(-116, 38, -84)));

    // The horizontal equipment housing reaches from the column to the C-arm
    // pivot, so the arc no longer appears detached from the standing unit.
    group.add(capsule(14, 74, shell, new THREE.Vector3(-78, 52, -84), new THREE.Euler(0, 0, Math.PI / 2)));
    group.add(box(54, 28, 42, shell, new THREE.Vector3(-44, 52, -84)));
    group.add(cylinder(23, 23, 22, darkMetal, new THREE.Vector3(-24, 52, -84), new THREE.Euler(Math.PI / 2, 0, 0), 48));

    // Small monitor and handles, modelled as simple readable silhouettes.
    group.add(cylinder(5, 6, 28, metal, new THREE.Vector3(-98, 91, -84)));
    group.add(box(46, 24, 6, whiteShell, new THREE.Vector3(-98, 108, -84), new THREE.Euler(THREE.MathUtils.degToRad(-8), 0, 0)));
    group.add(box(31, 16, 2, new THREE.MeshBasicMaterial({ color: 0x16222f }), new THREE.Vector3(-98, 108, -80)));
    group.add(capsule(3.5, 32, metal, new THREE.Vector3(-68, 38, -52), new THREE.Euler(Math.PI / 2, 0, 0)));
    group.add(capsule(3.5, 32, metal, new THREE.Vector3(-86, 31, -52), new THREE.Euler(Math.PI / 2, 0, 0)));

    gantryGroup.position.set(0, 20, 0);
    group.add(gantryGroup);

    // C-arm assembly. The left side of the arc sits on the pivot hub above,
    // while the detector and tube extend from the open side toward isocentre.
    const arcZ = -84;
    const arcRadius = 74;
    const arcX = 0;
    const mainArc = arcTube(arcRadius, 80, 280, arcZ, 5.8, whiteShell, arcX);
    const rearRail = arcTube(arcRadius + 8, 84, 276, arcZ + 5, 1.9, metal, arcX);
    const frontRail = arcTube(arcRadius - 8, 84, 276, arcZ - 5, 1.9, metal, arcX);
    gantryGroup.add(mainArc, rearRail, frontRail);

    const pivotX = arcX - arcRadius;
    gantryGroup.add(cylinder(17, 17, 18, darkMetal, new THREE.Vector3(pivotX, 0, arcZ), new THREE.Euler(Math.PI / 2, 0, 0), 48));
    gantryGroup.add(box(36, 22, 30, shell, new THREE.Vector3(pivotX + 16, 0, arcZ)));
    gantryGroup.add(box(52, 16, 20, shell, new THREE.Vector3(pivotX + 42, 0, arcZ)));

    const topY = 62;
    const bottomY = -62;
    gantryGroup.add(box(46, 12, 18, whiteShell, new THREE.Vector3(-7, topY, arcZ)));
    gantryGroup.add(box(46, 12, 18, whiteShell, new THREE.Vector3(-7, bottomY, arcZ)));
    gantryGroup.add(box(12, 12, 86, whiteShell, new THREE.Vector3(13, topY, -42)));
    gantryGroup.add(box(12, 12, 86, whiteShell, new THREE.Vector3(13, bottomY, -42)));

    const detector = box(46, 16, 42, detectorMaterial, new THREE.Vector3(13, topY, 4));
    gantryGroup.add(detector);
    const detectorFace = box(38, 2, 34, detectorFaceMaterial, new THREE.Vector3(13, topY - 9, 4));
    gantryGroup.add(detectorFace);

    const sourceHousing = box(58, 22, 44, sourceMaterial, new THREE.Vector3(13, bottomY, 4));
    gantryGroup.add(sourceHousing);
    const collimator = box(36, 9, 26, darkMetal, new THREE.Vector3(13, bottomY + 17, 4));
    gantryGroup.add(collimator);

    const beam = cylinder(16, 23, topY - bottomY - 20, beamMaterial, new THREE.Vector3(13, 0, 4));
    gantryGroup.add(beam);

    const isoMarker = new THREE.Mesh(new THREE.RingGeometry(7.5, 9, 48), isoMaterial);
    isoMarker.position.set(13, 0, 4);
    isoMarker.rotation.x = Math.PI / 2;
    gantryGroup.add(isoMarker);

    // Hinge cover in world/static coordinates aligned with the rotating pivot.
    // It visually bridges the stationary boom and the gantry hub at AP start.
    group.add(cylinder(25, 25, 14, darkMetal, new THREE.Vector3(-74, 20, -84), new THREE.Euler(Math.PI / 2, 0, 0), 48));

    return { group, gantryGroup };
}
