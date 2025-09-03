import * as THREE from 'https://unpkg.com/three@0.160.0/build/three.module.js';
import { ElasticRod } from '../../physics/elasticRod.js';

const rod = new ElasticRod(20, 0.5);
rod.nodes[5].y = 1;

const renderer = new THREE.WebGLRenderer({ canvas: document.getElementById('c') });
renderer.setSize(window.innerWidth, window.innerHeight);
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.1, 100);
camera.position.set(5,5,5);
camera.lookAt(0,0,0);

const material = new THREE.LineBasicMaterial({ color: 0xff0000 });
const geometry = new THREE.BufferGeometry();
const line = new THREE.Line(geometry, material);
scene.add(line);

function updateGeometry() {
    const positions = [];
    for (const n of rod.nodes) {
        positions.push(n.x, n.y, n.z);
    }
    geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
    geometry.attributes.position.needsUpdate = true;
}

function animate() {
    rod.step(0.01);
    updateGeometry();
    renderer.render(scene, camera);
    requestAnimationFrame(animate);
}

updateGeometry();
animate();
