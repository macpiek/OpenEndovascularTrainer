/** Typed material-node storage shared by feed, rendering and the Kirchhoff world. */
function createNodeStorage(count, mass, bendingStiffness, bendAngleLimit) {
    const storage = {
        x: new Float64Array(count),
        y: new Float64Array(count),
        z: new Float64Array(count),
        vx: new Float64Array(count),
        vy: new Float64Array(count),
        vz: new Float64Array(count),
        mass: new Float64Array(count),
        bendingStiffness: new Float64Array(count),
        bendAngleLimit: new Float64Array(count),
        pinned: new Uint8Array(count)
    };
    storage.mass.fill(mass);
    storage.bendingStiffness.fill(bendingStiffness);
    storage.bendAngleLimit.fill(bendAngleLimit);
    return storage;
}

class RodNodeView {
    constructor(storage, index) {
        this._storage = storage;
        this.index = index;
    }

    get x() { return this._storage.x[this.index]; }
    set x(value) { this._storage.x[this.index] = value; }

    get y() { return this._storage.y[this.index]; }
    set y(value) { this._storage.y[this.index] = value; }

    get z() { return this._storage.z[this.index]; }
    set z(value) { this._storage.z[this.index] = value; }

    get vx() { return this._storage.vx[this.index]; }
    set vx(value) { this._storage.vx[this.index] = value; }

    get vy() { return this._storage.vy[this.index]; }
    set vy(value) { this._storage.vy[this.index] = value; }

    get vz() { return this._storage.vz[this.index]; }
    set vz(value) { this._storage.vz[this.index] = value; }

    get mass() { return this._storage.mass[this.index]; }
    set mass(value) { this._storage.mass[this.index] = value; }

    get bendingStiffness() { return this._storage.bendingStiffness[this.index]; }
    set bendingStiffness(value) { this._storage.bendingStiffness[this.index] = value; }

    get bendAngleLimit() { return this._storage.bendAngleLimit[this.index]; }
    set bendAngleLimit(value) { this._storage.bendAngleLimit[this.index] = value; }

    get pinned() { return this._storage.pinned[this.index] !== 0; }
    set pinned(value) { this._storage.pinned[this.index] = value ? 1 : 0; }
}

export class RodState {
    constructor(count, segmentLength, {
        mass = 1,
        bendingStiffness = 32,
        bendAngleLimit = 50
    } = {}) {
        this.segmentLength = segmentLength;
        this.nodeStorage = createNodeStorage(count, mass, bendingStiffness, bendAngleLimit);
        this.nodes = Array.from({ length: count }, (_, index) =>
            new RodNodeView(this.nodeStorage, index)
        );
        this.nodes.nodeStorage = this.nodeStorage;
        for (let index = 0; index < count; index++) {
            this.nodeStorage.x[index] = index * segmentLength;
        }
    }

    computeLength() {
        const { x, y, z } = this.nodeStorage;
        let total = 0;
        for (let i = 0; i < this.nodes.length - 1; i++) {
            total += Math.hypot(x[i + 1] - x[i], y[i + 1] - y[i], z[i + 1] - z[i]);
        }
        return total;
    }

    bendAngleAt(index) {
        if (index <= 0 || index >= this.nodes.length - 1) return 0;
        const { x, y, z } = this.nodeStorage;
        const ax = x[index] - x[index - 1];
        const ay = y[index] - y[index - 1];
        const az = z[index] - z[index - 1];
        const bx = x[index + 1] - x[index];
        const by = y[index + 1] - y[index];
        const bz = z[index + 1] - z[index];
        const aLen = Math.hypot(ax, ay, az);
        const bLen = Math.hypot(bx, by, bz);
        if (aLen < 1e-8 || bLen < 1e-8) return 0;

        const dot = (ax * bx + ay * by + az * bz) / (aLen * bLen);
        return Math.acos(Math.max(-1, Math.min(1, dot))) * 180 / Math.PI;
    }
}
