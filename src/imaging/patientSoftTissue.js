import * as THREE from 'three';

const TAU = Math.PI * 2;
const smooth = x => { x = Math.max(0, Math.min(1, x)); return x * x * (3 - 2 * x); };

// Presentation only: phases consume accepted simulation time, never wall time.
// Rates are illustrative; this is not a tissue mechanics/physiology solver.
export function createPatientMotion() {
    let lastTime = 0, respiration = 0;
    const state = { time: 0, inspiration: 0, contraction: 0 };
    return {
        state,
        update(time, { respiratoryRate = 14, cardiacPhase = (time * 75 / 60) % 1 } = {}) {
            if (!Number.isFinite(time) || time < 0) throw new RangeError('Invalid patient time');
            if (time < lastTime) respiration = time * respiratoryRate / 60;
            else respiration += (time - lastTime) * respiratoryRate / 60;
            lastTime = time;
            const phase = ((cardiacPhase % 1) + 1) % 1;
            state.time = time;
            state.inspiration = (1 - Math.cos(TAU * respiration)) * .5;
            // Short systole followed by a slower return; no discontinuity at a beat.
            state.contraction = smooth((phase - .08) / .12) * (1 - smooth((phase - .25) / .35));
            return state;
        }
    };
}

function projectionMaterial(absorption, gas = false, organ = '') {
    return new THREE.ShaderMaterial({
        uniforms: { time: { value: 0 }, inspiration: { value: 0 }, contraction: { value: 0 },
            absorption: { value: new THREE.Vector2(gas ? 0 : absorption, gas ? absorption : 0) } },
        defines: organ ? { [organ]: 1 } : {},
        vertexShader: `
            uniform float time;
            uniform float inspiration;
            uniform float contraction;
            varying vec3 viewPoint;
            #ifdef BOWEL
            attribute vec3 tubeCenter;
            attribute float along;
            attribute vec2 motility;
            #endif
            void main() {
                vec3 p = position;
                #ifdef HEART
                // The ventricular apex shortens more than the broad base.
                float ventricle = 1.0 - smoothstep(-.55, .65, p.y);
                p.xz *= 1.0 - contraction * (.02 + .045 * ventricle);
                p.y += contraction * .055 * ventricle;
                float twist = contraction * .04 * ventricle;
                p.xz = mat2(cos(twist), sin(twist), -sin(twist), cos(twist)) * p.xz;
                #endif
                #ifdef BOWEL
                // Arc length keeps wave speed independent of loop length.
                // Wall and gas share the same deformation and remain nested.
                float phase = motility.x;
                float wave = pow(.5 + .5 * sin(along * .09 - time * .65 + phase), 6.0);
                wave *= .65 + .35 * sin(time * .18 + phase);
                p = tubeCenter + (p - tubeCenter) * (1.0 - .18 * motility.y * wave);
                vec2 driftPhase = vec2(tubeCenter.y * .018, tubeCenter.x * .032) + phase;
                p.xy += motility.y * vec2(1.2, .7) *
                    (sin(vec2(time * .38, time * .29) + driftPhase) - sin(driftPhase));
                p.z += .5 * motility.y * (sin(time * .22 + phase) - sin(phase));
                p.y -= inspiration * (1.3 + 1.7 * smoothstep(-90.0, 90.0, tubeCenter.y));
                #endif
                vec4 v = modelViewMatrix * vec4(p, 1.0);
                viewPoint = v.xyz;
                gl_Position = projectionMatrix * v;
            }
        `,
        fragmentShader: `
            uniform vec2 absorption;
            varying vec3 viewPoint;
            void main() {
                // Sum signed entry/exit distances of closed volumes. The result
                // is ray length, independent of draw order or overlapping organs.
                float path = length(viewPoint) * (gl_FrontFacing ? -1.0 : 1.0);
                gl_FragColor = vec4(absorption * path, 0.0, 0.0);
            }
        `,
        transparent: true, forceSinglePass: true, side: THREE.DoubleSide,
        depthTest: false, depthWrite: false, blending: THREE.CustomBlending,
        blendEquation: THREE.AddEquation, blendSrc: THREE.OneFactor, blendDst: THREE.OneFactor
    });
}

function closedTube(points, radius, { colon, phase, gas }) {
    const curve = new THREE.CatmullRomCurve3(points.map(p => new THREE.Vector3(...p)));
    const length = curve.getLength(), segments = colon ? 192 : 64, radial = 10;
    const geometry = new THREE.TubeGeometry(curve, segments, radius, radial, false);
    const positions = geometry.attributes.position, centers = new Float32Array(positions.count * 3), along = new Float32Array(positions.count);
    const motility = new Float32Array(positions.count * 2);
    const center = new THREE.Vector3(), point = new THREE.Vector3();
    for (let ring = 0; ring <= segments; ring++) {
        const t = ring / segments;
        curve.getPointAt(t, center);
        // Taper both ends to a point: closed volumes also project correctly
        // under oblique C-arm views, without an uncapped tube's depth streaks.
        const distance = t * length;
        const cap = Math.sqrt(Math.max(0, Math.min(1, distance / radius, (length - distance) / radius)));
        const folds = colon ? .9 + .1 * Math.cos(distance * .29) : .96 + .04 * Math.sin(distance * .32 + phase);
        // Variable gas pockets leave partially filled stretches, instead of
        // tracing every loop as an equally bright, uniform hose.
        const pocket = gas ? .16 + .56 * smooth((Math.sin(distance * .055 + phase) + .35) / 1.35) : 1;
        for (let j = 0; j <= radial; j++) {
            const index = ring * (radial + 1) + j;
            point.fromBufferAttribute(positions, index).sub(center).multiplyScalar(cap * folds * pocket);
            point.z *= .88;
            point.add(center);
            positions.setXYZ(index, point.x, point.y, point.z);
            centers.set(center.toArray(), index * 3); along[index] = distance;
            motility[index * 2] = phase; motility[index * 2 + 1] = colon ? .55 : 1;
        }
    }
    geometry.setAttribute('tubeCenter', new THREE.BufferAttribute(centers, 3));
    geometry.setAttribute('along', new THREE.BufferAttribute(along, 1));
    geometry.setAttribute('motility', new THREE.BufferAttribute(motility, 2));
    return geometry;
}

function heartGeometry() {
    const geometry = new THREE.SphereGeometry(1, 32, 24);
    const positions = geometry.attributes.position;
    for (let i = 0; i < positions.count; i++) {
        const x = positions.getX(i), y = positions.getY(i), z = positions.getZ(i);
        // Broad base, tapered apex and a fuller ventricular border. Keep one
        // closed volume so overlapping ellipsoids cannot create density seams.
        positions.setXYZ(i,
            x * (.93 + .18 * y) - .12 * (1 - y * y),
            y > 0 ? y * .85 : y,
            z * (.95 + .08 * y) + .08 * (1 - y * y));
    }
    geometry.computeVertexNormals();
    return geometry;
}

/** Low-contrast, world-space anatomy; deliberately separate from collision
 * geometry. One small projection pass per acquired X-ray pulse, no solver work.
 * Red stores tissue optical depth, green the reduction caused by air/gas. */
export function createPatientSoftTissue({ bifurcationY = -300 } = {}) {
    const scene = new THREE.Scene(), group = new THREE.Group();
    group.position.y = bifurcationY - 30;
    scene.add(group);
    const motion = createPatientMotion(), sphere = new THREE.SphereGeometry(1, 32, 24);
    const materials = [], geometries = [sphere];
    const material = (...args) => { const m = projectionMaterial(...args); materials.push(m); return m; };
    const bodyMaterial = material(.0018), heartMaterial = material(.0027, false, 'HEART'), liverMaterial = material(.0012), airMaterial = material(.0016, true);
    function organ(name, center, scale, material, tilt = 0) {
        const mesh = new THREE.Mesh(sphere, material); mesh.name = name;
        mesh.position.fromArray(center); mesh.scale.fromArray(scale); mesh.rotation.z = tilt;
        mesh.frustumCulled = false; group.add(mesh); return mesh;
    }
    const body = organ('soft-tissue-body', [0, 60, -12], [125, 305, 82], bodyMaterial);
    const rightLung = organ('right-lung', [-59, 249, -7], [47, 115, 48], airMaterial, -.07);
    const leftLung = organ('left-lung', [60, 253, -7], [44, 128, 48], airMaterial, .08);
    // +X is the patient's left in the anterior projection. A positive tilt
    // points the apex left/inferiorly, rather than toward the right lung.
    // Register the superior border against the STL ascending-aorta inlet
    // (world Y ~ -15 after vascular render alignment), not the abdominal frame.
    const heartRestCenter = [27, 280, 29];
    const heart = organ('heart', heartRestCenter, [48, 56, 35], heartMaterial, .48);
    heart.geometry = heartGeometry(); geometries.push(heart.geometry);
    const liver = organ('liver-diaphragm', [-43, 105, 0], [77, 40, 49], liverMaterial, -.12);
    const bowelWall = material(.0013, false, 'BOWEL'), bowelGas = material(.008, true, 'BOWEL');
    // The colon frames lower, asymmetric small-bowel loops. Coordinates are
    // relative to the same pelvic landmark as the skeleton, not the viewport.
    const loops = [
        [[-70,-78,8],[-84,-47,10],[-80,-9,8],[-83,34,10],[-69,88,12],
            [-40,80,20],[-5,57,29],[30,69,24],[66,96,10],[81,62,7],
            [78,22,9],[74,-21,10],[67,-63,15],[46,-91,24],[12,-105,29],
            [29,-127,23],[45,-109,13]],
        [[-48,35,28],[-30,47,32],[-7,33,37],[6,13,34],[-15,1,30],[-41,9,27],[-53,-9,25]],
        [[20,40,24],[46,25,30],[34,3,35],[11,-11,32],[25,-31,29],[50,-19,23]],
        [[-45,-29,28],[-19,-18,34],[1,-37,37],[-14,-54,30],[-40,-48,26],[-53,-66,23]],
        [[48,-44,22],[24,-50,27],[9,-72,33],[30,-84,29],[47,-69,22]],
        [[-47,-88,21],[-29,-75,28],[-9,-87,32],[-15,-107,29],[-39,-115,23]]
    ];
    loops.forEach((points, i) => {
        const colon = i === 0, radius = colon ? 10.5 : 5.5 + .5 * Math.sin(i);
        for (const [gas, mat] of [[false, bowelWall], [true, bowelGas]]) {
            const geometry = closedTube(points, radius, { colon, phase: i * 1.7, gas }); geometries.push(geometry);
            const mesh = new THREE.Mesh(geometry, mat); mesh.name = 'bowel-loop'; mesh.frustumCulled = false; group.add(mesh);
        }
    });
    return {
        scene, motion, heart, rightLung, leftLung, body,
        update(time, physiology) {
            const state = motion.update(time, physiology), breath = state.inspiration, beat = state.contraction;
            for (const m of materials) {
                m.uniforms.time.value = time;
                m.uniforms.inspiration.value = breath;
                m.uniforms.contraction.value = beat;
            }
            body.scale.z = 82 + breath * 1.5;
            rightLung.position.y = 249 - breath * 4; rightLung.scale.y = 115 + breath * 5;
            leftLung.position.y = 253 - breath * 4; leftLung.scale.y = 128 + breath * 5;
            liver.position.y = 105 - breath * 8;
            heart.position.set(heartRestCenter[0] + beat * .7,
                heartRestCenter[1] - breath * 3 + beat * .8, heartRestCenter[2]);
            heart.scale.set(48 * (1 - beat * .015), 56 * (1 - beat * .012), 35 * (1 - beat * .02));
            heart.rotation.z = .48 + beat * .018;
        },
        dispose() { for (const g of geometries) g.dispose(); for (const m of materials) m.dispose(); }
    };
}
