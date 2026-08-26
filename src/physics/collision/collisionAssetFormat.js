const MAGIC = 'OETCOLL1';
const MAGIC_BYTES = 8;
const HEADER_BYTES = MAGIC_BYTES + 4;
const FORMAT_VERSION = 1;
const textEncoder = new TextEncoder();
const textDecoder = new TextDecoder();

export const COLLISION_ASSET_MAGIC = MAGIC;
export const COLLISION_ASSET_VERSION = FORMAT_VERSION;

const ARRAY_TYPES = {
    Float32Array,
    Uint32Array,
    Int16Array,
    Uint8Array,
    Int8Array
};

function align(value, alignment = 8) {
    return Math.ceil(value / alignment) * alignment;
}

function copyMagic(target) {
    for (let index = 0; index < MAGIC_BYTES; index++) {
        target[index] = MAGIC.charCodeAt(index);
    }
}

function readMagic(bytes) {
    let value = '';
    for (let index = 0; index < MAGIC_BYTES; index++) {
        value += String.fromCharCode(bytes[index]);
    }
    return value;
}

export function encodeCollisionAsset(metadata, arrays) {
    const sections = [];
    for (const [name, array] of Object.entries(arrays)) {
        if (!array?.buffer || !ARRAY_TYPES[array.constructor.name]) {
            throw new TypeError(`Unsupported collision asset section: ${name}`);
        }
        sections.push({
            name,
            type: array.constructor.name,
            length: array.length,
            byteLength: array.byteLength
        });
    }

    const manifest = {
        ...metadata,
        format: MAGIC,
        version: FORMAT_VERSION,
        sections
    };
    const json = textEncoder.encode(JSON.stringify(manifest));
    let byteLength = align(HEADER_BYTES + json.byteLength);
    for (const section of sections) byteLength = align(byteLength + section.byteLength);

    const buffer = new ArrayBuffer(byteLength);
    const bytes = new Uint8Array(buffer);
    copyMagic(bytes);
    new DataView(buffer).setUint32(MAGIC_BYTES, json.byteLength, true);
    bytes.set(json, HEADER_BYTES);

    let offset = align(HEADER_BYTES + json.byteLength);
    for (const section of sections) {
        const source = arrays[section.name];
        bytes.set(new Uint8Array(source.buffer, source.byteOffset, source.byteLength), offset);
        offset = align(offset + source.byteLength);
    }
    return buffer;
}

export function decodeCollisionAsset(buffer) {
    if (!(buffer instanceof ArrayBuffer)) {
        throw new TypeError('Collision asset must be an ArrayBuffer');
    }
    if (buffer.byteLength < HEADER_BYTES) throw new Error('Collision asset is truncated');

    const bytes = new Uint8Array(buffer);
    const magic = readMagic(bytes);
    if (magic !== MAGIC) throw new Error(`Unexpected collision asset magic: ${magic}`);
    const jsonLength = new DataView(buffer).getUint32(MAGIC_BYTES, true);
    const jsonEnd = HEADER_BYTES + jsonLength;
    if (jsonEnd > buffer.byteLength) throw new Error('Collision asset manifest is truncated');

    const metadata = JSON.parse(textDecoder.decode(bytes.subarray(HEADER_BYTES, jsonEnd)));
    if (metadata.version !== FORMAT_VERSION) {
        throw new Error(`Unsupported collision asset version: ${metadata.version}`);
    }

    const arrays = {};
    let offset = align(jsonEnd);
    for (const section of metadata.sections || []) {
        const Type = ARRAY_TYPES[section.type];
        if (!Type) throw new Error(`Unsupported collision asset array type: ${section.type}`);
        const byteLength = section.length * Type.BYTES_PER_ELEMENT;
        if (offset + byteLength > buffer.byteLength) {
            throw new Error(`Collision asset section is truncated: ${section.name}`);
        }
        arrays[section.name] = new Type(buffer, offset, section.length);
        offset = align(offset + byteLength);
    }

    return { metadata, arrays, buffer };
}
