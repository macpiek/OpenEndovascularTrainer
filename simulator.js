const canvas = document.getElementById('sim');
const ctx = canvas.getContext('2d');
let width = window.innerWidth;
let height = window.innerHeight;
canvas.width = width;
canvas.height = height;

const centerX = width / 2;
const centerY = height / 2;

// UI controls
const stiffnessSlider = document.getElementById('stiffness');
const carmSlider = document.getElementById('carm');
let wireStiffness = parseFloat(stiffnessSlider.value);
let cameraAngle = 0;
stiffnessSlider.addEventListener('input', e => {
    wireStiffness = parseFloat(e.target.value);
});
carmSlider.addEventListener('input', e => {
    cameraAngle = parseFloat(e.target.value) * Math.PI / 180;
});

// vessel geometry
const vessel = (() => {
    const radius = 20;
    const branchRadius = 14; // smaller than main vessel
    const angle = Math.PI / 6; // 30 degrees
    const branchLength = 300;
    const branchY = height / 3;
    const branchPoint = {x: centerX, y: branchY, z: 0};
    const rightEnd = {
        x: branchPoint.x + Math.sin(angle) * branchLength,
        y: branchPoint.y + Math.cos(angle) * branchLength,
        z: 0
    };
    const leftEnd = {
        x: branchPoint.x - Math.sin(angle) * branchLength,
        y: branchPoint.y + Math.cos(angle) * branchLength,
        z: 0
    };
    return {
        radius,
        branchRadius,
        branchLength,
        branchPoint,
        main: {start: {x: centerX, y: 0, z: 0}, end: {x: centerX, y: branchY, z: 0}},
        right: {start: branchPoint, end: rightEnd},
        left: {start: branchPoint, end: leftEnd}
    };
})();

function clamp(v, min, max) {
    return Math.min(Math.max(v, min), max);
}

function distToSegment(n, seg) {
    const vx = seg.end.x - seg.start.x;
    const vy = seg.end.y - seg.start.y;
    const vz = (seg.end.z || 0) - (seg.start.z || 0);
    const wx = n.x - seg.start.x;
    const wy = n.y - seg.start.y;
    const wz = n.z - (seg.start.z || 0);
    const len2 = vx * vx + vy * vy + vz * vz;
    let t = (wx * vx + wy * vy + wz * vz) / len2;
    t = clamp(t, 0, 1);
    const px = seg.start.x + vx * t;
    const py = seg.start.y + vy * t;
    const pz = (seg.start.z || 0) + vz * t;
    const dx = n.x - px;
    const dy = n.y - py;
    const dz = n.z - pz;
    return Math.hypot(dx, dy, dz);
}

function clampToSegment(n, seg, radius) {
    const vx = seg.end.x - seg.start.x;
    const vy = seg.end.y - seg.start.y;
    const vz = (seg.end.z || 0) - (seg.start.z || 0);
    const wx = n.x - seg.start.x;
    const wy = n.y - seg.start.y;
    const wz = n.z - (seg.start.z || 0);
    const len2 = vx * vx + vy * vy + vz * vz;
    let t = (wx * vx + wy * vy + wz * vz) / len2;
    t = clamp(t, 0, 1);
    const px = seg.start.x + vx * t;
    const py = seg.start.y + vy * t;
    const pz = (seg.start.z || 0) + vz * t;
    let dx = n.x - px;
    let dy = n.y - py;
    let dz = n.z - pz;
    const dist = Math.hypot(dx, dy, dz);
    if (dist > radius) {
        const s = radius / dist;
        dx *= s; dy *= s; dz *= s;
        n.x = px + dx;
        n.y = py + dy;
        n.z = pz + dz;
    }
}

function clampToVessel(n) {
    const bp = vessel.branchPoint;
    if (n.y <= bp.y) {
        clampToSegment(n, vessel.main, vessel.radius - 1);
    } else {
        const dRight = distToSegment(n, vessel.right);
        const dLeft = distToSegment(n, vessel.left);
        if (dRight < dLeft) {
            clampToSegment(n, vessel.right, vessel.branchRadius - 1);
        } else {
            clampToSegment(n, vessel.left, vessel.branchRadius - 1);
        }
    }
}

// guidewire representation
const segmentLength = 12;
const nodeCount = 40;
const nodes = [];

for (let i = 0; i < nodeCount; i++) {
    nodes.push({
        x: centerX,
        y: -i * segmentLength,
        z: 0,
        px: centerX,
        py: -i * segmentLength,
        pz: 0
    });
}

let head = nodes[0];

// control including depth (z) with W/S keys
const control = {x: 0, y: 0, z: 0};
window.addEventListener('keydown', e => {
    if (e.key === 'ArrowUp') control.y = -1;
    if (e.key === 'ArrowDown') control.y = 1;
    if (e.key === 'ArrowLeft') control.x = -1;
    if (e.key === 'ArrowRight') control.x = 1;
    if (e.key === 'w' || e.key === 'W') control.z = -1;
    if (e.key === 's' || e.key === 'S') control.z = 1;
});
window.addEventListener('keyup', e => {
    if (['ArrowUp','ArrowDown'].includes(e.key)) control.y = 0;
    if (['ArrowLeft','ArrowRight'].includes(e.key)) control.x = 0;
    if (['w','W','s','S'].includes(e.key)) control.z = 0;
});

function step() {
    // move head by control
    head.x += control.x * 2;
    head.y += control.y * 2;
    head.z += control.z * 2;

    // verlet integration
    for (const n of nodes) {
        const vx = (n.x - n.px) * 0.98;
        const vy = (n.y - n.py) * 0.98;
        const vz = (n.z - n.pz) * 0.98;
        n.px = n.x;
        n.py = n.y;
        n.pz = n.z;
        n.x += vx;
        n.y += vy;
        n.z += vz;
    }

    // constraint iterations
    for (let k = 0; k < 12; k++) {
        for (let i = 1; i < nodes.length; i++) {
            const a = nodes[i-1];
            const b = nodes[i];
            let dx = b.x - a.x;
            let dy = b.y - a.y;
            let dz = b.z - a.z;
            const dist = Math.sqrt(dx*dx + dy*dy + dz*dz);
            const diff = (dist - segmentLength) / dist;
            const offsetX = dx * 0.5 * diff;
            const offsetY = dy * 0.5 * diff;
            const offsetZ = dz * 0.5 * diff;
            a.x += offsetX;
            a.y += offsetY;
            a.z += offsetZ;
            b.x -= offsetX;
            b.y -= offsetY;
            b.z -= offsetZ;
        }

        // bending stiffness
        for (let i = 0; i < nodes.length - 2; i++) {
            const a = nodes[i];
            const c = nodes[i+2];
            let dx = c.x - a.x;
            let dy = c.y - a.y;
            let dz = c.z - a.z;
            const dist = Math.sqrt(dx*dx + dy*dy + dz*dz);
            const diff = (dist - segmentLength*2) / dist;
            const strength = wireStiffness;
            const offsetX = dx * 0.5 * diff * strength;
            const offsetY = dy * 0.5 * diff * strength;
            const offsetZ = dz * 0.5 * diff * strength;
            a.x += offsetX;
            a.y += offsetY;
            a.z += offsetZ;
            c.x -= offsetX;
            c.y -= offsetY;
            c.z -= offsetZ;
        }

        clampToVessel(head);
    }

    // collisions with vessel boundaries
    for (const n of nodes) {
        clampToVessel(n);
    }
}

function project(p) {
    const cos = Math.cos(cameraAngle);
    const sin = Math.sin(cameraAngle);
    const dx = p.x - centerX;
    const x = dx * cos - p.z * sin + centerX;
    return {x, y: p.y};
}

function draw() {
    ctx.fillStyle = 'rgba(0,0,0,0.15)';
    ctx.fillRect(0, 0, width, height);

    // vessel projection
    ctx.fillStyle = 'rgba(120,120,120,0.4)';
    const m = vessel.main;
    const mainStart = project(m.start);
    const mainEnd = project(m.end);
    ctx.fillRect(mainStart.x - vessel.radius, mainStart.y, vessel.radius * 2, mainEnd.y - mainStart.y);

    const bp = project(vessel.branchPoint);
    const endR = project(vessel.right.end);
    const endL = project(vessel.left.end);
    const br = vessel.branchRadius;
    ctx.save();
    ctx.translate(bp.x, bp.y);
    ctx.rotate(Math.atan2(endR.x - bp.x, endR.y - bp.y));
    ctx.fillRect(-br, 0, br * 2, vessel.branchLength);
    ctx.restore();
    ctx.save();
    ctx.translate(bp.x, bp.y);
    ctx.rotate(Math.atan2(endL.x - bp.x, endL.y - bp.y));
    ctx.fillRect(-br, 0, br * 2, vessel.branchLength);
    ctx.restore();

    // guidewire
    ctx.strokeStyle = '#fff';
    ctx.lineWidth = 2;
    ctx.beginPath();
    const p0 = project(nodes[0]);
    ctx.moveTo(p0.x, p0.y);
    for (let i = 1; i < nodes.length; i++) {
        const p = project(nodes[i]);
        ctx.lineTo(p.x, p.y);
    }
    ctx.stroke();
}

function loop() {
    step();
    draw();
    requestAnimationFrame(loop);
}

loop();
