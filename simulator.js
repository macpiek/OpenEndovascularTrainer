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
        // use more subdivision steps so that the collision geometry
        // more closely follows a smooth Bézier curve. With too few
        // segments the guidewire hits sharp corners and appears to
        // "stick" when traversing the bifurcation.
        const steps = 24;
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

function projectOnSegment(n, seg) {
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
    const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
    return {px, py, pz, dx, dy, dz, dist};
}

// Less aggressive damping when the wire rubs against the wall to allow
// smoother gliding along the vessel surface.
const wallFriction = 0.2; // fraction of velocity lost when scraping the wall

function clampToVessel(n) {
    let nearest = vessel.segments[0];
    let best = projectOnSegment(n, nearest);
    for (let i = 1; i < vessel.segments.length; i++) {
        const seg = vessel.segments[i];
        const p = projectOnSegment(n, seg);
        if (p.dist < best.dist) {
            best = p;
            nearest = seg;
        }
    }
    const radius = nearest.radius - 1;
    if (best.dist > radius) {
        const inv = 1 / best.dist;
        const nx = best.dx * inv;
        const ny = best.dy * inv;
        const nz = best.dz * inv;
        n.x = best.px + nx * radius;
        n.y = best.py + ny * radius;
        n.z = best.pz + nz * radius;
        const vn = n.vx * nx + n.vy * ny + n.vz * nz;
        n.vx = (n.vx - vn * nx) * (1 - wallFriction);
        n.vy = (n.vy - vn * ny) * (1 - wallFriction);
        n.vz = (n.vz - vn * nz) * (1 - wallFriction);
    }
}

// guidewire representation using position-based dynamics
const segmentLength = 12;
const nodeCount = 40;

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

class Wire {
    constructor(segLen, count, start, dir) {
        this.segmentLength = segLen;
        this.nodes = [];
        this.tailStart = start;
        this.dir = dir;
        for (let i = 0; i < count; i++) {
            const x = vessel.left.end.x - dir.x * segLen * i;
            const y = vessel.left.end.y - dir.y * segLen * i;
            this.nodes.push({x, y, z: 0, vx: 0, vy: 0, vz: 0, oldx: x, oldy: y, oldz: 0});
        }
        this.tailProgress = 0;
        this.maxInsert = segLen * (count - 1) - 40;
    }

    updateTail(advance, dt) {
        this.tailProgress = clamp(this.tailProgress + advance * 40 * dt, 0, this.maxInsert);
        const tail = this.nodes[this.nodes.length - 1];
        const tx = this.tailStart.x + this.dir.x * this.tailProgress;
        const ty = this.tailStart.y + this.dir.y * this.tailProgress;
        // move the fixed tail segment smoothly and keep a velocity estimate
        // instead of abruptly zeroing it each frame, which caused the
        // remainder of the wire to lose momentum when inserting.
        const oldx = tail.x;
        const oldy = tail.y;
        const oldz = tail.z;
        tail.x = tx;
        tail.y = ty;
        tail.z = 0;
        tail.vx = (tail.x - oldx) / dt;
        tail.vy = (tail.y - oldy) / dt;
        tail.vz = (tail.z - oldz) / dt;
    }

    integrate(dt) {
        for (let i = 0; i < this.nodes.length - 1; i++) {
            const n = this.nodes[i];
            n.vx *= 0.98;
            n.vy *= 0.98;
            n.vz *= 0.98;
            n.x += n.vx * dt;
            n.y += n.vy * dt;
            n.z += n.vz * dt;
        }
    }

    satisfyConstraints(iterations) {
        const nodes = this.nodes;
        const len = this.segmentLength;
        const tail = nodes[nodes.length - 1];
        for (let k = 0; k < iterations; k++) {
            for (let i = 1; i < nodes.length; i++) {
                const a = nodes[i - 1];
                const b = nodes[i];
                let dx = b.x - a.x;
                let dy = b.y - a.y;
                let dz = b.z - a.z;
                const dist = Math.sqrt(dx * dx + dy * dy + dz * dz) || 1;
                const diff = (dist - len) / dist;
                const offx = dx * 0.5 * diff;
                const offy = dy * 0.5 * diff;
                const offz = dz * 0.5 * diff;
                a.x += offx;
                a.y += offy;
                a.z += offz;
                if (i !== nodes.length - 1) {
                    b.x -= offx;
                    b.y -= offy;
                    b.z -= offz;
                }
            }

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

            clampToVessel(nodes[0]);
            tail.x = this.tailStart.x + this.dir.x * this.tailProgress;
            tail.y = this.tailStart.y + this.dir.y * this.tailProgress;
            tail.z = 0;
        }
    }

    collide() {
        for (let i = 0; i < this.nodes.length - 1; i++) {
            clampToVessel(this.nodes[i]);
        }
    }

    step(dt, advance) {
        for (const n of this.nodes) {
            n.oldx = n.x;
            n.oldy = n.y;
            n.oldz = n.z;
        }
        this.updateTail(advance, dt);
        this.integrate(dt);
        // perform more iterations for better convergence and smoother
        // bending of the wire, especially in curved sections
        this.satisfyConstraints(24);
        this.collide();
        for (let i = 0; i < this.nodes.length - 1; i++) {
            const n = this.nodes[i];
            n.vx = (n.x - n.oldx) / dt;
            n.vy = (n.y - n.oldy) / dt;
            n.vz = (n.z - n.oldz) / dt;
        }
    }
}

const wire = new Wire(segmentLength, nodeCount, tailStart, leftDir);

// only allow inserting or withdrawing the wire
let advance = 0;
window.addEventListener('keydown', e => {
    if (e.key === 'w' || e.key === 'W') advance = 1;
    if (e.key === 's' || e.key === 'S') advance = -1;
});
window.addEventListener('keyup', e => {
    if (['w','W','s','S'].includes(e.key)) advance = 0;
});

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
    const p0 = project(wire.nodes[0]);
    ctx.moveTo(p0.x, p0.y);
    for (let i = 1; i < wire.nodes.length - 1; i++) {
        const p1 = project(wire.nodes[i]);
        const p2 = project(wire.nodes[i + 1]);
        const mid = {x: (p1.x + p2.x) * 0.5, y: (p1.y + p2.y) * 0.5};
        ctx.quadraticCurveTo(p1.x, p1.y, mid.x, mid.y);
    }
    const last = project(wire.nodes[wire.nodes.length - 1]);
    ctx.lineTo(last.x, last.y);
    ctx.stroke();
}

let lastTime = performance.now();
function loop(time) {
    const dt = (time - lastTime) / 1000;
    lastTime = time;
    wire.step(dt, advance);
    draw();
    requestAnimationFrame(loop);
}
requestAnimationFrame(loop);
