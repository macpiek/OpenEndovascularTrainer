const canvas = document.getElementById('sim');
const renderer = new THREE.WebGLRenderer({canvas, antialias: true});
renderer.setSize(window.innerWidth, window.innerHeight);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x000000);


scene.add(camera);

const light = new THREE.HemisphereLight(0xffffff, 0x444444, 1);
scene.add(light);

let vesselMaterial = new THREE.MeshStandardMaterial({color: 0x3366ff});
let vesselGroup;

function createTaperedTube(path, tubularSegments, radialSegments, startRadius, endRadius) {
    const geometry = new THREE.TubeGeometry(path, tubularSegments, 1, radialSegments, false);
    const pos = geometry.attributes.position;
    const normals = geometry.attributes.normal;
    const segments = tubularSegments + 1;
    const radials = radialSegments + 1;
    for (let i = 0; i < segments; i++) {
        const t = i / tubularSegments;
        const r = startRadius + (endRadius - startRadius) * t;
        for (let j = 0; j < radials; j++) {
            const idx = i * radials + j;
            pos.setX(idx, pos.getX(idx) + normals.getX(idx) * (r - 1));
            pos.setY(idx, pos.getY(idx) + normals.getY(idx) * (r - 1));
            pos.setZ(idx, pos.getZ(idx) + normals.getZ(idx) * (r - 1));

        }
    }
    pos.needsUpdate = true;
    geometry.computeVertexNormals();
    return geometry;
}


    }
    vesselGroup = new THREE.Group();

    const mainRadius = 20;
    const branchRadius = 14;
    const branchPoint = 80;
    const branchLength = 120 + Math.random() * 40;
    const blend = 40;
    const branchAngleOffset = (Math.random() - 0.5) * Math.PI / 12;

    // main vessel
    const trunkGeom = new THREE.CylinderGeometry(mainRadius, mainRadius, branchPoint, 32, 1, true);
    const trunk = new THREE.Mesh(trunkGeom, vesselMaterial);
    trunk.position.y = branchPoint / 2;
    vesselGroup.add(trunk);

    function branch(dir) {
        const angle = Math.PI / 6 * dir + branchAngleOffset * dir;
        const curve = new THREE.QuadraticBezierCurve3(
            new THREE.Vector3(0, branchPoint, 0),
            new THREE.Vector3(Math.sin(angle) * blend, branchPoint + blend, Math.cos(angle) * blend),
            new THREE.Vector3(Math.sin(angle) * (blend + branchLength), branchPoint + branchLength, Math.cos(angle) * (blend + branchLength))
        );
        const geom = createTaperedTube(curve, 64, 16, mainRadius, branchRadius);
        const mesh = new THREE.Mesh(geom, vesselMaterial);
        vesselGroup.add(mesh);
    }

    branch(1);
    branch(-1);

    scene.add(vesselGroup);
}


window.addEventListener('resize', () => {
    const w = window.innerWidth;
    const h = window.innerHeight;
    renderer.setSize(w, h);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
});
