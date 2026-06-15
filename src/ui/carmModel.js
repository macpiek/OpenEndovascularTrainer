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

function transverseArcTube(radius, startDeg, endDeg, x, tubeRadius, material, zOffset = 0) {
    const points = [];
    for (let i = 0; i <= 96; i++) {
        const angle = THREE.MathUtils.degToRad(startDeg + ((endDeg - startDeg) * i) / 96);
        points.push(new THREE.Vector3(
            x,
            radius * Math.sin(angle),
            zOffset + radius * Math.cos(angle)
        ));
    }
    const curve = new THREE.CatmullRomCurve3(points);
    return new THREE.Mesh(new THREE.TubeGeometry(curve, 128, tubeRadius, 18, false), material);
}

export function createCArmModel() {
    const group = new THREE.Group();
    const liftGroup = new THREE.Group();
    const gantryGroup = new THREE.Group();
    const previewIsoCenter = new THREE.Vector3(10, 22, 0);

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

    // Mobile base and cabinet stay stationary; they act as the carriage and
    // support for the rotating C arc.
    group.add(box(118, 12, 58, rubber, new THREE.Vector3(10, -105, -82)));
    group.add(box(78, 52, 62, shell, new THREE.Vector3(10, -72, -82)));
    group.add(box(84, 18, 62, darkMetal, new THREE.Vector3(10, -103, -82)));
    group.add(cylinder(12, 12, 10, rubber, new THREE.Vector3(-34, -108, -108), new THREE.Euler(Math.PI / 2, 0, 0), 32));
    group.add(cylinder(12, 12, 10, rubber, new THREE.Vector3(54, -108, -108), new THREE.Euler(Math.PI / 2, 0, 0), 32));
    group.add(cylinder(10, 10, 8, rubber, new THREE.Vector3(-34, -108, -56), new THREE.Euler(Math.PI / 2, 0, 0), 32));
    group.add(cylinder(10, 10, 8, rubber, new THREE.Vector3(54, -108, -56), new THREE.Euler(Math.PI / 2, 0, 0), 32));

    group.add(cylinder(16, 18, 70, darkMetal, new THREE.Vector3(10, -37, -82)));
    liftGroup.add(cylinder(12, 14, 96, metal, new THREE.Vector3(10, -14, -82)));
    liftGroup.add(box(62, 24, 52, shell, new THREE.Vector3(10, 38, -82)));
    liftGroup.add(box(28, 10, 44, darkMetal, new THREE.Vector3(10, 24, -82)));

    // Straight support from the mobile column toward the gantry pivot.
    liftGroup.add(capsule(12, 34, shell, new THREE.Vector3(10, 37, -82), new THREE.Euler(0, 0, Math.PI / 2)));
    liftGroup.add(box(48, 26, 40, shell, new THREE.Vector3(10, 37, -82)));
    liftGroup.add(box(54, 18, 28, shell, new THREE.Vector3(10, 29, -86)));
    liftGroup.add(box(34, 22, 34, shell, new THREE.Vector3(10, 20, -86)));
    liftGroup.add(cylinder(21, 21, 18, darkMetal, new THREE.Vector3(10, 26, -82), new THREE.Euler(Math.PI / 2, 0, 0), 48));
    liftGroup.add(cylinder(25, 25, 18, darkMetal, new THREE.Vector3(10, 22, -86), new THREE.Euler(Math.PI / 2, 0, 0), 48));
    liftGroup.add(box(46, 34, 24, darkMetal, new THREE.Vector3(10, 22, -86)));

    // Small monitor and handles on the mobile module.
    liftGroup.add(cylinder(5, 6, 28, metal, new THREE.Vector3(28, 79, -82)));
    liftGroup.add(box(46, 24, 6, whiteShell, new THREE.Vector3(28, 96, -82), new THREE.Euler(THREE.MathUtils.degToRad(-8), 0, 0)));
    liftGroup.add(box(31, 16, 2, new THREE.MeshBasicMaterial({ color: 0x16222f }), new THREE.Vector3(28, 96, -78)));
    liftGroup.add(capsule(3.2, 30, metal, new THREE.Vector3(38, 19, -52), new THREE.Euler(Math.PI / 2, 0, 0)));

    gantryGroup.position.copy(previewIsoCenter);
    liftGroup.add(gantryGroup);
    group.add(liftGroup);

    // Classic C-arm: a vertical C arc in the transverse plane of the patient.
    // The patient lies along X, so the arc itself lives in Y/Z and is attached
    // to the mobile module from the lateral side.
    const arcRadius = 86;
    const arcX = 0;
    const arcZ = 0;
    const terminalDeg = 58;
    const mainArc = transverseArcTube(arcRadius, terminalDeg, 360 - terminalDeg, arcX, 6.2, whiteShell, arcZ);
    const rearRail = transverseArcTube(arcRadius + 8, terminalDeg + 2, 360 - terminalDeg - 2, arcX - 4.5, 1.8, metal, arcZ);
    const frontRail = transverseArcTube(arcRadius - 8, terminalDeg + 2, 360 - terminalDeg - 2, arcX + 4.5, 1.8, metal, arcZ);
    gantryGroup.add(mainArc, rearRail, frontRail);
    gantryGroup.add(box(28, 18, 34, shell, new THREE.Vector3(arcX, 0, -arcRadius)));

    const terminalRad = THREE.MathUtils.degToRad(terminalDeg);
    const topY = arcRadius * Math.sin(terminalRad);
    const bottomY = -topY;
    const beamZ = 0;
    const arcEndZ = arcZ + arcRadius * Math.cos(terminalRad);
    const connectorZ = (arcEndZ + beamZ) * 0.5;
    const connectorDepth = Math.abs(beamZ - arcEndZ) + 12;
    gantryGroup.add(box(48, 13, connectorDepth, whiteShell, new THREE.Vector3(arcX, topY, connectorZ)));
    gantryGroup.add(box(48, 13, connectorDepth, whiteShell, new THREE.Vector3(arcX, bottomY, connectorZ)));
    gantryGroup.add(box(42, 14, 16, whiteShell, new THREE.Vector3(arcX, topY, arcEndZ)));
    gantryGroup.add(box(42, 14, 16, whiteShell, new THREE.Vector3(arcX, bottomY, arcEndZ)));

    const detector = box(50, 16, 42, detectorMaterial, new THREE.Vector3(arcX, topY, beamZ));
    gantryGroup.add(detector);
    const detectorFace = box(40, 2, 34, detectorFaceMaterial, new THREE.Vector3(arcX, topY - 9, beamZ));
    gantryGroup.add(detectorFace);

    const sourceHousing = box(58, 22, 44, sourceMaterial, new THREE.Vector3(arcX, bottomY, beamZ));
    gantryGroup.add(sourceHousing);
    const collimator = box(36, 9, 26, darkMetal, new THREE.Vector3(arcX, bottomY + 17, beamZ));
    gantryGroup.add(collimator);

    const beam = cylinder(15, 22, topY - bottomY - 20, beamMaterial, new THREE.Vector3(arcX, 0, beamZ));
    gantryGroup.add(beam);

    const isoMarker = new THREE.Mesh(new THREE.RingGeometry(7.5, 9, 48), isoMaterial);
    isoMarker.position.set(arcX, 0, beamZ);
    isoMarker.rotation.x = Math.PI / 2;
    gantryGroup.add(isoMarker);

    return { group, gantryGroup, liftGroup };
}
