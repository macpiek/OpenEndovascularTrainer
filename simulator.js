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

// vessel geometry with curved branching
const vessel = (() => {
    const radius = 20;
    const branchRadius = 14; // smaller than main vessel
    const angle = Math.PI / 6; // 30 degrees
    const branchLength = 300;
    const blend = 60; // length of curved transition section
    const branchY = height / 3;
    const branchPoint = {x: centerX, y: branchY, z: 0};
    const mainEnd = {x: centerX, y: branchY - blend, z: 0};

    function branch(dir) {
        const a = angle * dir;
        const curveEnd = {
            x: branchPoint.x + Math.sin(a) * blend,
            y: branchPoint.y + Math.cos(a) * blend,
            z: 0
        };
        const end = {
            x: branchPoint.x + Math.sin(a) * (branchLength + blend),
            y: branchPoint.y + Math.cos(a) * (branchLength + blend),
            z: 0
        };
        return {curveEnd, end, length: branchLength + blend};
    }

    const right = branch(1);
    const left = branch(-1);

    // collect segments for collision constraints
    const segments = [];
    segments.push({start: {x: centerX, y: 0, z: 0}, end: mainEnd, radius});

    function addCurve(p0, p1, p2) {
        const steps = 8;
        let prev = p0;
        for (let i = 1; i <= steps; i++) {
            const t = i / steps;
            const tt = 1 - t;
            const p = {
                x: tt * tt * p0.x + 2 * tt * t * p1.x + t * t * p2.x,
                y: tt * tt * p0.y + 2 * tt * t * p1.y + t * t * p2.y,
                z: 0
            };
            const r = radius + (branchRadius - radius) * t;
            segments.push({start: prev, end: p, radius: r});
            prev = p;
        }
    }

    addCurve(mainEnd, branchPoint, right.curveEnd);
    segments.push({start: right.curveEnd, end: right.end, radius: branchRadius});
    addCurve(mainEnd, branchPoint, left.curveEnd);
    segments.push({start: left.curveEnd, end: left.end, radius: branchRadius});

    return {
        radius,
        branchRadius,
        branchPoint,
        main: {start: {x: centerX, y: 0, z: 0}, end: mainEnd},
        right,
        left,
        segments
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

const wallFriction = 0.7; // fraction of velocity lost when scraping the wall

function clampToVessel(n) {
    let nearest = vessel.segments[0];
    let minDist = distToSegment(n, nearest);
    for (let i = 1; i < vessel.segments.length; i++) {
        const seg = vessel.segments[i];
        const d = distToSegment(n, seg);
        if (d < minDist) {
            minDist = d;
            nearest = seg;
        }
    }
    const collided = minDist > nearest.radius - 1;
    clampToSegment(n, nearest, nearest.radius - 1);
    if (collided) {
        const vx = n.x - n.px;
        const vy = n.y - n.py;
        const vz = n.z - n.pz;
        n.px = n.x - vx * (1 - wallFriction);
        n.py = n.y - vy * (1 - wallFriction);
        n.pz = n.z - vz * (1 - wallFriction);
    }
}

// guidewire representation
const segmentLength = 12;
const nodeCount = 40;
const nodes = [];

// direction pointing from left branch tip toward the bifurcation
const leftDir = {
    x: (vessel.branchPoint.x - vessel.left.end.x) / vessel.left.length,
    y: (vessel.branchPoint.y - vessel.left.end.y) / vessel.left.length
};

// starting position of the tail outside the vessel
const tailStart = {
    x: vessel.left.end.x - leftDir.x * segmentLength * (nodeCount - 1),
    y: vessel.left.end.y - leftDir.y * segmentLength * (nodeCount - 1),
    z: 0
};

for (let i = 0; i < nodeCount; i++) {
    const x = vessel.left.end.x - leftDir.x * segmentLength * i;
    const y = vessel.left.end.y - leftDir.y * segmentLength * i;
    nodes.push({x, y, z: 0, px: x, py: y, pz: 0});
}

let head = nodes[0];

// only allow inserting or withdrawing the wire
let advance = 0;
window.addEventListener('keydown', e => {
    if (e.key === 'w' || e.key === 'W') advance = 1;
    if (e.key === 's' || e.key === 'S') advance = -1;
});
window.addEventListener('keyup', e => {
    if (['w','W','s','S'].includes(e.key)) advance = 0;
});

let tailProgress = 0;
// allow inserting the full wire length while keeping a small portion outside
const maxInsert = segmentLength * (nodeCount - 1) - 40;

function step() {
    const tail = nodes[nodes.length - 1];
    tailProgress = clamp(tailProgress + advance * 2, 0, maxInsert);
    const tx = tailStart.x + leftDir.x * tailProgress;
    const ty = tailStart.y + leftDir.y * tailProgress;
    tail.x = tx;
    tail.y = ty;
    tail.z = 0;
    tail.px = tx;
    tail.py = ty;
    tail.pz = 0;

    // verlet integration
    for (let i = 0; i < nodes.length - 1; i++) {
        const n = nodes[i];
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
        // keep segments at fixed length
        for (let i = 1; i < nodes.length; i++) {
            const a = nodes[i - 1];
            const b = nodes[i];
            let dx = b.x - a.x;
            let dy = b.y - a.y;
            let dz = b.z - a.z;
            const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
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

        // bending stiffness -- pull middle node toward neighbours' midpoint
        for (let i = 1; i < nodes.length - 1; i++) {
            const prev = nodes[i - 1];
            const curr = nodes[i];
            const next = nodes[i + 1];
            const mx = (prev.x + next.x) * 0.5;
            const my = (prev.y + next.y) * 0.5;
            const mz = (prev.z + next.z) * 0.5;
            curr.x += (mx - curr.x) * wireStiffness;
            curr.y += (my - curr.y) * wireStiffness;
            curr.z += (mz - curr.z) * wireStiffness;
        }

        clampToVessel(head);
        tail.x = tx;
        tail.y = ty;
        tail.z = 0;
    }

    // collisions with vessel boundaries
    for (let i = 0; i < nodes.length - 1; i++) {
        clampToVessel(nodes[i]);
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
    ctx.strokeStyle = 'rgba(120,120,120,0.4)';
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';

    const sheathPos = project(vessel.left.end);
    const sheathAngle = Math.atan2(vessel.left.end.y - vessel.branchPoint.y, vessel.left.end.x - vessel.branchPoint.x);
    ctx.save();
    ctx.translate(sheathPos.x, sheathPos.y);
    ctx.rotate(sheathAngle + Math.PI);
    ctx.fillStyle = 'rgba(180,180,180,0.4)';
    ctx.fillRect(-vessel.branchRadius * 0.7, 0, vessel.branchRadius * 1.4, 40);
    ctx.restore();
    ctx.fillStyle = 'rgba(120,120,120,0.4)';

    // main tube
    ctx.lineWidth = vessel.radius * 2;
    const mainStart = project(vessel.main.start);
    const mainEnd = project(vessel.main.end);
    ctx.beginPath();
    ctx.moveTo(mainStart.x, mainStart.y);
    ctx.lineTo(mainEnd.x, mainEnd.y);
    ctx.stroke();

    // right branch
    ctx.lineWidth = vessel.branchRadius * 2;
    const pStart = project(vessel.main.end);
    const pCtrl = project(vessel.branchPoint);
    const pRight = project(vessel.right.curveEnd);
    const endR = project(vessel.right.end);
    ctx.beginPath();
    ctx.moveTo(pStart.x, pStart.y);
    ctx.quadraticCurveTo(pCtrl.x, pCtrl.y, pRight.x, pRight.y);
    ctx.lineTo(endR.x, endR.y);
    ctx.stroke();

    // left branch
    const pLeft = project(vessel.left.curveEnd);
    const endL = project(vessel.left.end);
    ctx.beginPath();
    ctx.moveTo(pStart.x, pStart.y);
    ctx.quadraticCurveTo(pCtrl.x, pCtrl.y, pLeft.x, pLeft.y);
    ctx.lineTo(endL.x, endL.y);
    ctx.stroke();

    // guidewire
    ctx.strokeStyle = '#fff';
    ctx.lineWidth = 2;
    ctx.beginPath();
    const p0 = project(nodes[0]);
    ctx.moveTo(p0.x, p0.y);
    for (let i = 1; i < nodes.length - 1; i++) {
        const p1 = project(nodes[i]);
        const p2 = project(nodes[i + 1]);
        const mid = {x: (p1.x + p2.x) * 0.5, y: (p1.y + p2.y) * 0.5};
        ctx.quadraticCurveTo(p1.x, p1.y, mid.x, mid.y);
    }
    const last = project(nodes[nodes.length - 1]);
    ctx.lineTo(last.x, last.y);
    ctx.stroke();
}

function loop() {
    step();
    draw();
    requestAnimationFrame(loop);
}

loop();
