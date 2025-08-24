const canvas = document.getElementById('sim');
const ctx = canvas.getContext('2d');
let width = window.innerWidth;
let height = window.innerHeight;
canvas.width = width;
canvas.height = height;

// vessel geometry: union of vertical and branch rectangles
const vessel = {
    main: {x1: width/2 - 20, x2: width/2 + 20, y1: 0, y2: height},
    branch: {x1: width/2 + 20, x2: width/2 + 300, y1: height/3 - 20, y2: height/3 + 20}
};

function insideVessel(x, y) {
    const m = vessel.main;
    const b = vessel.branch;
    const inMain = x >= m.x1 && x <= m.x2 && y >= m.y1 && y <= m.y2;
    const inBranch = x >= b.x1 && x <= b.x2 && y >= b.y1 && y <= b.y2;
    return inMain || inBranch;
}

// guidewire representation
const segmentLength = 12;
const nodeCount = 40;
const nodes = [];

for (let i = 0; i < nodeCount; i++) {
    nodes.push({
        x: width/2,
        y: -i * segmentLength,
        px: width/2,
        py: -i * segmentLength
    });
}

let head = nodes[0];

// control
const control = {x: 0, y: 0};
window.addEventListener('keydown', e => {
    if (e.key === 'ArrowUp') control.y = -1;
    if (e.key === 'ArrowDown') control.y = 1;
    if (e.key === 'ArrowLeft') control.x = -1;
    if (e.key === 'ArrowRight') control.x = 1;
});
window.addEventListener('keyup', e => {
    if (['ArrowUp','ArrowDown'].includes(e.key)) control.y = 0;
    if (['ArrowLeft','ArrowRight'].includes(e.key)) control.x = 0;
});

function step() {
    // move head by control
    head.x += control.x * 2;
    head.y += control.y * 2;

    // verlet integration
    for (const n of nodes) {
        const vx = (n.x - n.px) * 0.98;
        const vy = (n.y - n.py) * 0.98;
        n.px = n.x;
        n.py = n.y;
        n.x += vx;
        n.y += vy;
    }

    // constraint iterations
    for (let k = 0; k < 8; k++) {
        for (let i = 1; i < nodes.length; i++) {
            const a = nodes[i-1];
            const b = nodes[i];
            let dx = b.x - a.x;
            let dy = b.y - a.y;
            const dist = Math.sqrt(dx*dx + dy*dy);
            const diff = (dist - segmentLength) / dist;
            const offsetX = dx * 0.5 * diff;
            const offsetY = dy * 0.5 * diff;
            a.x += offsetX;
            a.y += offsetY;
            b.x -= offsetX;
            b.y -= offsetY;
        }
        // fix head after adjustments
        head.x = Math.min(Math.max(head.x, vessel.main.x1 + 1), vessel.main.x2 - 1);
        head.y = Math.max(head.y, 0);
    }

    // collisions with vessel boundaries
    for (const n of nodes) {
        if (!insideVessel(n.x, n.y)) {
            if (n.y < vessel.branch.y1 || n.y > vessel.branch.y2 || n.x < vessel.branch.x1) {
                n.x = Math.min(Math.max(n.x, vessel.main.x1+1), vessel.main.x2-1);
            }
            if (n.y > vessel.branch.y1 && n.y < vessel.branch.y2 && n.x > vessel.main.x2) {
                n.y = Math.min(Math.max(n.y, vessel.branch.y1+1), vessel.branch.y2-1);
                n.x = Math.min(Math.max(n.x, vessel.branch.x1+1), vessel.branch.x2-1);
            }
        }
    }
}

function draw() {
    ctx.fillStyle = 'rgba(0,0,0,0.15)';
    ctx.fillRect(0, 0, width, height);
    // vessel
    ctx.fillStyle = 'rgba(120,120,120,0.4)';
    const m = vessel.main;
    ctx.fillRect(m.x1, m.y1, m.x2 - m.x1, m.y2 - m.y1);
    const b = vessel.branch;
    ctx.fillRect(b.x1, b.y1, b.x2 - b.x1, b.y2 - b.y1);
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
