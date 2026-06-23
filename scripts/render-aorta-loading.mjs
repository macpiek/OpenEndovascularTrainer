import fs from 'node:fs';
import zlib from 'node:zlib';

const INPUT = 'res/Aorta_plain.stl';
const OUTPUT = 'res/aorta-loading.png';
const WIDTH = 720;
const HEIGHT = 560;
const MARGIN = 60;
const YAW = -0.26;

const buffer = fs.readFileSync(INPUT);
const triangleCount = buffer.readUInt32LE(80);
const cosYaw = Math.cos(YAW);
const sinYaw = Math.sin(YAW);

function viewPoint(x, y, z) {
    const depth = -y;
    return {
        x: x * cosYaw + depth * sinYaw,
        y: z,
        z: -x * sinYaw + depth * cosYaw
    };
}

function forEachTriangle(callback) {
    let offset = 84;
    for (let i = 0; i < triangleCount && offset + 50 <= buffer.length; i++) {
        offset += 12;
        const a = viewPoint(
            buffer.readFloatLE(offset),
            buffer.readFloatLE(offset + 4),
            buffer.readFloatLE(offset + 8)
        );
        offset += 12;
        const b = viewPoint(
            buffer.readFloatLE(offset),
            buffer.readFloatLE(offset + 4),
            buffer.readFloatLE(offset + 8)
        );
        offset += 12;
        const c = viewPoint(
            buffer.readFloatLE(offset),
            buffer.readFloatLE(offset + 4),
            buffer.readFloatLE(offset + 8)
        );
        offset += 14;
        callback(a, b, c);
    }
}

let minX = Infinity;
let maxX = -Infinity;
let minY = Infinity;
let maxY = -Infinity;

forEachTriangle((a, b, c) => {
    for (const p of [a, b, c]) {
        minX = Math.min(minX, p.x);
        maxX = Math.max(maxX, p.x);
        minY = Math.min(minY, p.y);
        maxY = Math.max(maxY, p.y);
    }
});

const scale = Math.min(
    (WIDTH - MARGIN * 2) / Math.max(1, maxX - minX),
    (HEIGHT - MARGIN * 2) / Math.max(1, maxY - minY)
);
const centerX = (minX + maxX) * 0.5;
const centerY = (minY + maxY) * 0.5;

function project(p) {
    return {
        x: WIDTH * 0.5 + (p.x - centerX) * scale,
        y: HEIGHT * 0.5 - (p.y - centerY) * scale,
        z: p.z
    };
}

const rgba = new Uint8Array(WIDTH * HEIGHT * 4);
const zBuffer = new Float32Array(WIDTH * HEIGHT);
zBuffer.fill(-Infinity);

function setPixel(index, depth, shade) {
    if (depth <= zBuffer[index]) return;
    zBuffer[index] = depth;

    const baseR = 159;
    const baseG = 29;
    const baseB = 43;
    const lift = Math.max(0, Math.min(1, shade));
    const out = index * 4;
    rgba[out] = Math.round(baseR + 62 * lift);
    rgba[out + 1] = Math.round(baseG + 54 * lift);
    rgba[out + 2] = Math.round(baseB + 55 * lift);
    rgba[out + 3] = 218;
}

function splat(x, y, depth, shade) {
    const cx = Math.round(x);
    const cy = Math.round(y);
    for (let yy = cy - 1; yy <= cy + 1; yy++) {
        if (yy < 0 || yy >= HEIGHT) continue;
        for (let xx = cx - 1; xx <= cx + 1; xx++) {
            if (xx < 0 || xx >= WIDTH) continue;
            setPixel(yy * WIDTH + xx, depth, shade);
        }
    }
}

function renderTriangle(a, b, c) {
    const abx = b.x - a.x;
    const aby = b.y - a.y;
    const abz = b.z - a.z;
    const acx = c.x - a.x;
    const acy = c.y - a.y;
    const acz = c.z - a.z;
    const nx = aby * acz - abz * acy;
    const ny = abz * acx - abx * acz;
    const nz = abx * acy - aby * acx;
    const nLen = Math.hypot(nx, ny, nz) || 1;
    const light = Math.max(0, (-0.28 * nx - 0.38 * ny + 0.88 * nz) / nLen);
    const facing = Math.abs(nz / nLen);
    const shade = 0.18 + light * 0.56 + facing * 0.26;

    const pa = project(a);
    const pb = project(b);
    const pc = project(c);
    const area = (pb.x - pa.x) * (pc.y - pa.y) - (pb.y - pa.y) * (pc.x - pa.x);
    if (Math.abs(area) < 0.05) {
        splat((pa.x + pb.x + pc.x) / 3, (pa.y + pb.y + pc.y) / 3, (pa.z + pb.z + pc.z) / 3, shade);
        return;
    }

    const minPx = Math.max(0, Math.floor(Math.min(pa.x, pb.x, pc.x)));
    const maxPx = Math.min(WIDTH - 1, Math.ceil(Math.max(pa.x, pb.x, pc.x)));
    const minPy = Math.max(0, Math.floor(Math.min(pa.y, pb.y, pc.y)));
    const maxPy = Math.min(HEIGHT - 1, Math.ceil(Math.max(pa.y, pb.y, pc.y)));
    const invArea = 1 / area;

    for (let y = minPy; y <= maxPy; y++) {
        for (let x = minPx; x <= maxPx; x++) {
            const px = x + 0.5;
            const py = y + 0.5;
            const w0 = ((pb.x - px) * (pc.y - py) - (pb.y - py) * (pc.x - px)) * invArea;
            const w1 = ((pc.x - px) * (pa.y - py) - (pc.y - py) * (pa.x - px)) * invArea;
            const w2 = 1 - w0 - w1;
            if (w0 < 0 || w1 < 0 || w2 < 0) continue;
            const depth = pa.z * w0 + pb.z * w1 + pc.z * w2;
            setPixel(y * WIDTH + x, depth, shade);
        }
    }
}

forEachTriangle(renderTriangle);

const outlined = new Uint8Array(rgba);
for (let y = 1; y < HEIGHT - 1; y++) {
    for (let x = 1; x < WIDTH - 1; x++) {
        const index = y * WIDTH + x;
        if (rgba[index * 4 + 3] !== 0) continue;
        let touchesModel = false;
        for (let oy = -1; oy <= 1 && !touchesModel; oy++) {
            for (let ox = -1; ox <= 1; ox++) {
                if (rgba[((y + oy) * WIDTH + x + ox) * 4 + 3] > 0) {
                    touchesModel = true;
                    break;
                }
            }
        }
        if (!touchesModel) continue;
        const out = index * 4;
        outlined[out] = 126;
        outlined[out + 1] = 24;
        outlined[out + 2] = 36;
        outlined[out + 3] = 92;
    }
}

function crc32(bufferLike) {
    let crc = 0xffffffff;
    for (const byte of bufferLike) {
        crc ^= byte;
        for (let i = 0; i < 8; i++) {
            crc = (crc >>> 1) ^ (0xedb88320 & -(crc & 1));
        }
    }
    return (crc ^ 0xffffffff) >>> 0;
}

function chunk(type, data) {
    const typeBuffer = Buffer.from(type);
    const out = Buffer.alloc(12 + data.length);
    out.writeUInt32BE(data.length, 0);
    typeBuffer.copy(out, 4);
    data.copy(out, 8);
    out.writeUInt32BE(crc32(Buffer.concat([typeBuffer, data])), 8 + data.length);
    return out;
}

function cropToContent(pixels) {
    let left = WIDTH;
    let right = -1;
    let top = HEIGHT;
    let bottom = -1;
    for (let y = 0; y < HEIGHT; y++) {
        for (let x = 0; x < WIDTH; x++) {
            if (pixels[(y * WIDTH + x) * 4 + 3] === 0) continue;
            left = Math.min(left, x);
            right = Math.max(right, x);
            top = Math.min(top, y);
            bottom = Math.max(bottom, y);
        }
    }

    const padding = 52;
    left = Math.max(0, left - padding);
    right = Math.min(WIDTH - 1, right + padding);
    top = Math.max(0, top - padding);
    bottom = Math.min(HEIGHT - 1, bottom + padding);
    const croppedWidth = right - left + 1;
    const croppedHeight = bottom - top + 1;
    const cropped = new Uint8Array(croppedWidth * croppedHeight * 4);
    for (let y = 0; y < croppedHeight; y++) {
        const sourceStart = ((top + y) * WIDTH + left) * 4;
        const targetStart = y * croppedWidth * 4;
        cropped.set(pixels.subarray(sourceStart, sourceStart + croppedWidth * 4), targetStart);
    }
    return { pixels: cropped, width: croppedWidth, height: croppedHeight };
}

function writePng(path, pixels, width, height) {
    const raw = Buffer.alloc((width * 4 + 1) * height);
    for (let y = 0; y < height; y++) {
        const rowStart = y * (width * 4 + 1);
        raw[rowStart] = 0;
        Buffer.from(pixels.buffer, pixels.byteOffset + y * width * 4, width * 4).copy(raw, rowStart + 1);
    }

    const header = Buffer.alloc(13);
    header.writeUInt32BE(width, 0);
    header.writeUInt32BE(height, 4);
    header[8] = 8;
    header[9] = 6;
    header[10] = 0;
    header[11] = 0;
    header[12] = 0;

    fs.writeFileSync(path, Buffer.concat([
        Buffer.from([0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a]),
        chunk('IHDR', header),
        chunk('IDAT', zlib.deflateSync(raw, { level: 9 })),
        chunk('IEND', Buffer.alloc(0))
    ]));
}

const cropped = cropToContent(outlined);
writePng(OUTPUT, cropped.pixels, cropped.width, cropped.height);
console.log(`Rendered ${triangleCount} STL triangles to ${OUTPUT} (${cropped.width}x${cropped.height})`);
