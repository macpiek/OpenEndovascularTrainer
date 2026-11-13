// Exact coordinate keys for the anatomical wall weld. This reproduces the
// former string keys, including their numerical-zero normalization, without
// allocating a coordinate string for every repeated STL corner.
const normalized = value => Math.abs(value) < 1e-7 ? 0 : Number.isNaN(value) ? NaN : value;
const equal = (a, b) => a === b || (Number.isNaN(a) && Number.isNaN(b));

export class ExactCoordinateMap {
    constructor() {
        this._table = new Uint32Array(32);
        this._coordinates = new Float64Array(16 * 3);
        this._values = [];
        this._hashCoordinates = new Float64Array(3);
        this._hashWords = new Uint32Array(this._hashCoordinates.buffer);
    }

    get size() { return this._values.length; }

    _hash(x, y, z) {
        const coordinates = this._hashCoordinates, words = this._hashWords;
        coordinates[0] = x; coordinates[1] = y; coordinates[2] = z;
        let hash = 0x811c9dc5;
        for (let i = 0; i < 6; i++) hash = Math.imul(hash ^ words[i], 0x01000193);
        hash = Math.imul(hash ^ (hash >>> 16), 0x7feb352d);
        return (hash ^ (hash >>> 16)) >>> 0;
    }

    _slot(x, y, z) {
        const table = this._table, coordinates = this._coordinates, mask = table.length - 1;
        let slot = this._hash(x, y, z) & mask;
        while (table[slot]) {
            const at = (table[slot] - 1) * 3;
            // The hash only locates a bucket. Compare all coordinates so a
            // collision cannot weld distinct physical wall positions.
            if (equal(coordinates[at], x) && equal(coordinates[at + 1], y) &&
                equal(coordinates[at + 2], z)) break;
            slot = (slot + 1) & mask;
        }
        return slot;
    }

    _growTable() {
        const table = new Uint32Array(this._table.length * 2), mask = table.length - 1;
        const coordinates = this._coordinates;
        for (let id = 0; id < this.size; id++) {
            const at = id * 3;
            let slot = this._hash(coordinates[at], coordinates[at + 1], coordinates[at + 2]) & mask;
            while (table[slot]) slot = (slot + 1) & mask;
            table[slot] = id + 1;
        }
        this._table = table;
    }

    get(x, y, z) {
        const id = this._table[this._slot(normalized(x), normalized(y), normalized(z))];
        return id ? this._values[id - 1] : undefined;
    }

    set(x, y, z, value) {
        x = normalized(x); y = normalized(y); z = normalized(z);
        let slot = this._slot(x, y, z);
        const existing = this._table[slot];
        if (existing) {
            this._values[existing - 1] = value;
            return this;
        }
        const id = this.size;
        if ((id + 1) * 10 > this._table.length * 7) {
            this._growTable();
            slot = this._slot(x, y, z);
        }
        if ((id + 1) * 3 > this._coordinates.length) {
            const coordinates = new Float64Array(this._coordinates.length * 2);
            coordinates.set(this._coordinates); this._coordinates = coordinates;
        }
        const at = id * 3;
        this._coordinates[at] = x; this._coordinates[at + 1] = y; this._coordinates[at + 2] = z;
        this._table[slot] = id + 1;
        this._values.push(value);
        return this;
    }

    // Coordinate IDs remain in insertion order when the hash table grows.
    // Replacing a territory-list head must not move or duplicate its entry.
    values() { return this._values.values(); }
}
