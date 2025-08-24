const canvas = document.getElementById('sim');
const ctx = canvas.getContext('2d');
let width = window.innerWidth;
let height = window.innerHeight;
canvas.width = width;
canvas.height = height;

const centerX = width / 2;

// vessel represented as cylindrical tubes
const vessel = {
    radius: 20,
    main: {x: centerX, y1: 0, y2: height},              // vertical tube
    branch: {y: height / 3, x1: centerX, x2: centerX + 300} // horizontal tube
};

function clamp(v, min, max) {
    return Math.min(Math.max(v, min), max);
}

function clampToVessel(n) {
    const r = vessel.radius - 1; // keep inside slightly
    const m = vessel.main;
    const b = vessel.branch;
    const inBranch = n.x >= b.x1 && Math.abs(n.y - b.y) <= vessel.radius;

    if (inBranch) {
        // branch tube runs along +x direction
        n.x = clamp(n.x, b.x1 + 1, b.x2 - 1);
        let dy = n.y - b.y;
        let dz = n.z;
        const dist = Math.hypot(dy, dz);
        if (dist > r) {
            const s = r / dist;
            dy *= s; dz *= s;
            n.y = b.y + dy;
            n.z = dz;
        }
    } else {
        // main vertical tube centered at m.x
        n.y = clamp(n.y, m.y1 + 1, m.y2 - 1);
        let dx = n.x - m.x;
        let dz = n.z;
        const dist = Math.hypot(dx, dz);
        if (dist > r) {
            const s = r / dist;
            dx *= s; dz *= s;
            n.x = m.x + dx;
            n.z = dz;
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
    for (let k = 0; k < 8; k++) {
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
        clampToVessel(head);
    }

    // collisions with vessel boundaries
    for (const n of nodes) {
        clampToVessel(n);
    }
}

function draw() {
    ctx.fillStyle = 'rgba(0,0,0,0.15)';
    ctx.fillRect(0, 0, width, height);
    // vessel projection (orthographic like fluoroscopy)
    ctx.fillStyle = 'rgba(120,120,120,0.4)';
    const r = vessel.radius;
    const m = vessel.main;
    ctx.fillRect(m.x - r, m.y1, r * 2, m.y2 - m.y1);
    const b = vessel.branch;
    ctx.fillRect(b.x1, b.y - r, b.x2 - b.x1, r * 2);
    // guidewire
    ctx.strokeStyle = '#fff';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(nodes[0].x, nodes[0].y);
    for (let i = 1; i < nodes.length; i++) {
        ctx.lineTo(nodes[i].x, nodes[i].y);
    }
    ctx.stroke();
}

function loop() {
    step();
    draw();
    requestAnimationFrame(loop);
}

loop();
