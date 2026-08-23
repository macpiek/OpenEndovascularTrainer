const EPSILON = 1e-12;

function finiteNumber(value, label) {
    if (!Number.isFinite(value)) {
        throw new TypeError(`${label} must be a finite number`);
    }
    return value;
}

function nonNegative(value, label) {
    finiteNumber(value, label);
    if (value < 0) throw new RangeError(`${label} must be non-negative`);
    return value;
}

function encodeMaterialId(value, label) {
    if (typeof value === 'string') return `s${value.length}:${value}`;
    if (typeof value === 'number' && Number.isFinite(value)) {
        return `n:${Object.is(value, -0) ? 0 : value}`;
    }
    if (typeof value === 'bigint') return `b:${value}`;
    throw new TypeError(`${label} must be a finite number, bigint or string`);
}

function vectorComponent(vector, index, key) {
    const value = Array.isArray(vector) || ArrayBuffer.isView(vector)
        ? vector[index]
        : vector?.[key];
    return finiteNumber(value, `vector.${key}`);
}

function readVector3(vector, label) {
    if (vector == null) throw new TypeError(`${label} is required`);
    return [
        vectorComponent(vector, 0, 'x'),
        vectorComponent(vector, 1, 'y'),
        vectorComponent(vector, 2, 'z')
    ];
}

function normalize3(vector, label) {
    const length = Math.sqrt(
        vector[0] * vector[0] +
        vector[1] * vector[1] +
        vector[2] * vector[2]
    );
    if (length <= EPSILON) throw new RangeError(`${label} must have positive length`);
    vector[0] /= length;
    vector[1] /= length;
    vector[2] /= length;
    return vector;
}

function dot3(a, b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function cross3(a, b) {
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0]
    ];
}

function fallbackTangent(normal) {
    const reference = Math.abs(normal[0]) <= Math.abs(normal[1]) &&
        Math.abs(normal[0]) <= Math.abs(normal[2])
        ? [1, 0, 0]
        : Math.abs(normal[1]) <= Math.abs(normal[2])
            ? [0, 1, 0]
            : [0, 0, 1];
    const projection = dot3(reference, normal);
    return normalize3([
        reference[0] - normal[0] * projection,
        reference[1] - normal[1] * projection,
        reference[2] - normal[2] * projection
    ], 'generated contact tangent');
}

function contactBasis(normalValue, tangentValue) {
    const normal = normalize3(readVector3(normalValue, 'normal'), 'normal');
    let tangentU;
    if (tangentValue == null) {
        tangentU = fallbackTangent(normal);
    } else {
        tangentU = readVector3(tangentValue, 'tangentU');
        const normalProjection = dot3(tangentU, normal);
        tangentU[0] -= normal[0] * normalProjection;
        tangentU[1] -= normal[1] * normalProjection;
        tangentU[2] -= normal[2] * normalProjection;
        const tangentLength = Math.sqrt(
            tangentU[0] * tangentU[0] +
            tangentU[1] * tangentU[1] +
            tangentU[2] * tangentU[2]
        );
        tangentU = tangentLength > EPSILON
            ? normalize3(tangentU, 'tangentU')
            : fallbackTangent(normal);
    }
    const tangentV = normalize3(cross3(normal, tangentU), 'tangentV');
    return { normal, tangentU, tangentV };
}

function writeContactBasisInPlace(
    normalValue,
    tangentValue,
    normal,
    tangentU,
    tangentV
) {
    let nx = vectorComponent(normalValue, 0, 'x');
    let ny = vectorComponent(normalValue, 1, 'y');
    let nz = vectorComponent(normalValue, 2, 'z');
    const normalLength = Math.sqrt(nx * nx + ny * ny + nz * nz);
    if (normalLength <= EPSILON) {
        throw new RangeError('normal must have positive length');
    }
    nx /= normalLength;
    ny /= normalLength;
    nz /= normalLength;
    let ux;
    let uy;
    let uz;
    if (tangentValue == null) {
        if (Math.abs(nx) <= Math.abs(ny) && Math.abs(nx) <= Math.abs(nz)) {
            ux = 1; uy = 0; uz = 0;
        } else if (Math.abs(ny) <= Math.abs(nz)) {
            ux = 0; uy = 1; uz = 0;
        } else {
            ux = 0; uy = 0; uz = 1;
        }
    } else {
        ux = vectorComponent(tangentValue, 0, 'x');
        uy = vectorComponent(tangentValue, 1, 'y');
        uz = vectorComponent(tangentValue, 2, 'z');
    }
    let normalProjection = ux * nx + uy * ny + uz * nz;
    ux -= nx * normalProjection;
    uy -= ny * normalProjection;
    uz -= nz * normalProjection;
    let tangentLength = Math.sqrt(ux * ux + uy * uy + uz * uz);
    if (tangentLength <= EPSILON) {
        if (Math.abs(nx) <= Math.abs(ny) && Math.abs(nx) <= Math.abs(nz)) {
            ux = 1; uy = 0; uz = 0;
        } else if (Math.abs(ny) <= Math.abs(nz)) {
            ux = 0; uy = 1; uz = 0;
        } else {
            ux = 0; uy = 0; uz = 1;
        }
        normalProjection = ux * nx + uy * ny + uz * nz;
        ux -= nx * normalProjection;
        uy -= ny * normalProjection;
        uz -= nz * normalProjection;
        tangentLength = Math.sqrt(ux * ux + uy * uy + uz * uz);
    }
    ux /= tangentLength;
    uy /= tangentLength;
    uz /= tangentLength;
    let vx = ny * uz - nz * uy;
    let vy = nz * ux - nx * uz;
    let vz = nx * uy - ny * ux;
    const bitangentLength = Math.sqrt(vx * vx + vy * vy + vz * vz);
    vx /= bitangentLength;
    vy /= bitangentLength;
    vz /= bitangentLength;
    normal[0] = nx; normal[1] = ny; normal[2] = nz;
    tangentU[0] = ux; tangentU[1] = uy; tangentU[2] = uz;
    tangentV[0] = vx; tangentV[1] = vy; tangentV[2] = vz;
}

function resolveContact(contacts, contactOrId) {
    const contact = typeof contactOrId === 'string'
        ? contacts.get(contactOrId)
        : contactOrId;
    if (!contact || contacts.get(contact.id) !== contact) {
        throw new RangeError('Unknown Kirchhoff contact');
    }
    return contact;
}

/**
 * Stable identity for a contact between two material rod segments. Solver
 * array indices are deliberately excluded: remeshing may move the same
 * material segments to different slots without creating a new contact.
 */
export function materialSegmentContactId(
    innerMaterialSegmentId,
    outerMaterialSegmentId,
    feature = 'lumen'
) {
    if (typeof feature !== 'string' || feature.length === 0) {
        throw new TypeError('feature must be a non-empty string');
    }
    return [
        `f${feature.length}:${feature}`,
        encodeMaterialId(innerMaterialSegmentId, 'innerMaterialSegmentId'),
        encodeMaterialId(outerMaterialSegmentId, 'outerMaterialSegmentId')
    ].join('|');
}

/**
 * Persistent contact state shared by future guidewire/catheter Kirchhoff rod
 * solvers. It owns warm-start multipliers and transient material-to-array
 * mappings, but it does not know about a particular world or integrator.
 */
export class KirchhoffContactManifold {
    constructor({ frictionCoefficient = 0, retentionSteps = 1 } = {}) {
        this.frictionCoefficient = nonNegative(
            frictionCoefficient,
            'frictionCoefficient'
        );
        this.retentionSteps = Math.max(
            0,
            Math.floor(nonNegative(retentionSteps, 'retentionSteps'))
        );
        this._contacts = new Map();
        this._step = -1;
        this._stepOpen = false;
    }

    get size() {
        return this._contacts.size;
    }

    get step() {
        return this._step;
    }

    contacts() {
        return this._contacts.values();
    }

    getContact(id) {
        return this._contacts.get(id) ?? null;
    }

    beginStep() {
        if (this._stepOpen) {
            throw new Error('endStep() must be called before beginStep()');
        }
        this._step++;
        this._stepOpen = true;
        return this._step;
    }

    endStep({ prune = true } = {}) {
        if (!this._stepOpen) return this._contacts.size;
        if (prune) {
            for (const [id, contact] of this._contacts) {
                if (this._step - contact.lastSeenStep > this.retentionSteps) {
                    contact._manifold = null;
                    this._contacts.delete(id);
                }
            }
        }
        this._stepOpen = false;
        return this._contacts.size;
    }

    /**
     * Creates or refreshes a material contact. Reusing the same material IDs
     * preserves all multipliers while indices and the local tangent basis may
     * change. Tangential lambda is re-expressed in the new basis so its world
     * impulse does not rotate merely because the discretization was remapped.
     */
    upsertContact({
        innerMaterialSegmentId,
        outerMaterialSegmentId,
        feature = 'lumen',
        innerSegmentIndex = -1,
        outerSegmentIndex = -1,
        normal,
        tangentU = null,
        frictionCoefficient,
        twistFrictionCoefficient,
        effectiveTwistRadius,
        id: suppliedId = null
    }) {
        if (!this._stepOpen) this.beginStep();
        const id = suppliedId ?? materialSegmentContactId(
            innerMaterialSegmentId,
            outerMaterialSegmentId,
            feature
        );
        const existing = this._contacts.get(id);
        if (existing) {
            return this.#remapContactValues(existing, {
                innerSegmentIndex,
                outerSegmentIndex,
                normal,
                tangentU,
                frictionCoefficient,
                twistFrictionCoefficient,
                effectiveTwistRadius
            });
        }

        const basis = contactBasis(normal, tangentU);
        const contactFriction = nonNegative(
            frictionCoefficient ?? this.frictionCoefficient,
            'frictionCoefficient'
        );
        const twistRadius = nonNegative(
            effectiveTwistRadius ?? 0,
            'effectiveTwistRadius'
        );
        const twistFriction = nonNegative(
            twistFrictionCoefficient ?? contactFriction,
            'twistFrictionCoefficient'
        );
        const contact = {
            _manifold: this,
            id,
            feature,
            innerMaterialSegmentId,
            outerMaterialSegmentId,
            innerSegmentIndex,
            outerSegmentIndex,
            normal: basis.normal,
            tangentU: basis.tangentU,
            tangentV: basis.tangentV,
            frictionCoefficient: contactFriction,
            twistFrictionCoefficient: twistFriction,
            effectiveTwistRadius: twistRadius,
            normalLambda: 0,
            tangentLambda: new Float64Array(2),
            twistLambda: 0,
            innerTwistImpulse: 0,
            outerTwistImpulse: 0,
            lastSeenStep: this._step
        };
        this._contacts.set(id, contact);
        return contact;
    }

    /**
     * Hot-path equivalent of remapContact for a contact object already owned
     * by this manifold. Runtime contact records retain that object across
     * solver sweeps, so repeating a Map lookup cannot add correctness.
     */
    refreshKnownContact(contact, values = {}) {
        if (contact?._manifold !== this) {
            throw new RangeError('Unknown Kirchhoff contact');
        }
        return this.#remapContactValues(contact, values);
    }

    /**
     * Marks an unchanged, zero-impulse contact as observed without rebuilding
     * its orthonormal tangent basis. The caller must only use this while the
     * contact remains open; a closing contact is fully refreshed before any
     * normal or friction projection is evaluated.
     */
    touchKnownOpenContact(contact) {
        if (contact?._manifold !== this) {
            throw new RangeError('Unknown Kirchhoff contact');
        }
        if (
            contact.normalLambda > EPSILON ||
            Math.abs(contact.tangentLambda[0]) > EPSILON ||
            Math.abs(contact.tangentLambda[1]) > EPSILON ||
            Math.abs(contact.twistLambda) > EPSILON
        ) {
            throw new RangeError('Only zero-impulse contacts may be touched');
        }
        contact.lastSeenStep = this._step;
        return contact;
    }

    /**
     * Recycles an already allocated contact for the material segment pair now
     * represented by the same runtime contact slot. A tip-anchored rod mesh
     * changes every segment's exact material coordinate during axial feed.
     * The old implementation allocated O(active segments) objects every fixed
     * step. A changed material identity still clears all XPBD/Coulomb state,
     * exactly like constructing a new contact, while retaining the storage.
     */
    rekeyKnownContact(contact, {
        innerMaterialSegmentId,
        outerMaterialSegmentId,
        feature = contact?.feature ?? 'lumen',
        id: suppliedId = null,
        ...values
    } = {}) {
        if (contact?._manifold !== this) {
            throw new RangeError('Unknown Kirchhoff contact');
        }
        const id = suppliedId ?? materialSegmentContactId(
            innerMaterialSegmentId,
            outerMaterialSegmentId,
            feature
        );
        const materialChanged =
            innerMaterialSegmentId !== contact.innerMaterialSegmentId ||
            outerMaterialSegmentId !== contact.outerMaterialSegmentId ||
            feature !== contact.feature;
        if (id !== contact.id) {
            const existing = this._contacts.get(id);
            if (existing && existing !== contact) {
                // A retained contact from an older material labelling no
                // longer owns this identity. A contact already refreshed in
                // the current sweep is authoritative and is reused instead.
                if (existing.lastSeenStep === this._step) {
                    if (this._contacts.get(contact.id) === contact) {
                        this._contacts.delete(contact.id);
                    }
                    contact._manifold = null;
                    return this.#remapContactValues(existing, values);
                }
                existing._manifold = null;
                this._contacts.delete(id);
            }
            if (this._contacts.get(contact.id) === contact) {
                this._contacts.delete(contact.id);
            }
            contact.id = id;
            this._contacts.set(id, contact);
        }
        if (materialChanged) {
            contact.feature = feature;
            contact.innerMaterialSegmentId = innerMaterialSegmentId;
            contact.outerMaterialSegmentId = outerMaterialSegmentId;
            contact.normalLambda = 0;
            contact.tangentLambda[0] = 0;
            contact.tangentLambda[1] = 0;
            contact.twistLambda = 0;
            contact.innerTwistImpulse = 0;
            contact.outerTwistImpulse = 0;
        }
        return this.#remapContactValues(contact, values);
    }

    remapContact(contactOrId, {
        innerSegmentIndex,
        outerSegmentIndex,
        normal,
        tangentU,
        frictionCoefficient,
        twistFrictionCoefficient,
        effectiveTwistRadius
    } = {}) {
        const contact = resolveContact(this._contacts, contactOrId);
        return this.#remapContactValues(contact, {
            innerSegmentIndex,
            outerSegmentIndex,
            normal,
            tangentU,
            frictionCoefficient,
            twistFrictionCoefficient,
            effectiveTwistRadius
        });
    }

    #remapContactValues(contact, {
        innerSegmentIndex,
        outerSegmentIndex,
        normal,
        tangentU,
        frictionCoefficient,
        twistFrictionCoefficient,
        effectiveTwistRadius
    }) {
        const sameNormal = normal == null || (
            vectorComponent(normal, 0, 'x') === contact.normal[0] &&
            vectorComponent(normal, 1, 'y') === contact.normal[1] &&
            vectorComponent(normal, 2, 'z') === contact.normal[2]
        );
        const sameTangent = tangentU == null || (
            vectorComponent(tangentU, 0, 'x') === contact.tangentU[0] &&
            vectorComponent(tangentU, 1, 'y') === contact.tangentU[1] &&
            vectorComponent(tangentU, 2, 'z') === contact.tangentU[2]
        );
        const sameIndices = (
            innerSegmentIndex === undefined ||
            innerSegmentIndex === contact.innerSegmentIndex
        ) && (
            outerSegmentIndex === undefined ||
            outerSegmentIndex === contact.outerSegmentIndex
        );
        const sameFriction = frictionCoefficient === undefined ||
            frictionCoefficient === contact.frictionCoefficient;
        const sameTwistFriction =
            twistFrictionCoefficient === undefined ||
            twistFrictionCoefficient ===
                contact.twistFrictionCoefficient;
        const sameRadius = effectiveTwistRadius === undefined ||
            effectiveTwistRadius === contact.effectiveTwistRadius;
        if (
            sameNormal && sameTangent && sameIndices &&
            sameFriction && sameTwistFriction && sameRadius
        ) {
            contact.lastSeenStep = this._step;
            return contact;
        }
        const previousImpulseX =
            contact.tangentU[0] * contact.tangentLambda[0] +
            contact.tangentV[0] * contact.tangentLambda[1];
        const previousImpulseY =
            contact.tangentU[1] * contact.tangentLambda[0] +
            contact.tangentV[1] * contact.tangentLambda[1];
        const previousImpulseZ =
            contact.tangentU[2] * contact.tangentLambda[0] +
            contact.tangentV[2] * contact.tangentLambda[1];
        if (normal != null || tangentU != null) {
            writeContactBasisInPlace(
                normal ?? contact.normal,
                tangentU ?? contact.tangentU,
                contact.normal,
                contact.tangentU,
                contact.tangentV
            );
        }
        contact.tangentLambda[0] =
            previousImpulseX * contact.tangentU[0] +
            previousImpulseY * contact.tangentU[1] +
            previousImpulseZ * contact.tangentU[2];
        contact.tangentLambda[1] =
            previousImpulseX * contact.tangentV[0] +
            previousImpulseY * contact.tangentV[1] +
            previousImpulseZ * contact.tangentV[2];
        if (innerSegmentIndex !== undefined) {
            contact.innerSegmentIndex = innerSegmentIndex;
        }
        if (outerSegmentIndex !== undefined) {
            contact.outerSegmentIndex = outerSegmentIndex;
        }
        if (frictionCoefficient !== undefined) {
            contact.frictionCoefficient = nonNegative(
                frictionCoefficient,
                'frictionCoefficient'
            );
        }
        if (twistFrictionCoefficient !== undefined) {
            contact.twistFrictionCoefficient = nonNegative(
                twistFrictionCoefficient,
                'twistFrictionCoefficient'
            );
        }
        if (effectiveTwistRadius !== undefined) {
            contact.effectiveTwistRadius = nonNegative(
                effectiveTwistRadius,
                'effectiveTwistRadius'
            );
        }
        contact.lastSeenStep = this._step;
        this.#projectTangentialLambda(contact, contact.frictionCoefficient);
        this.#projectTwistLambda(contact);
        return contact;
    }

    setNormalLambda(contactOrId, value) {
        const contact = resolveContact(this._contacts, contactOrId);
        const previous = contact.normalLambda;
        contact.normalLambda = nonNegative(value, 'normalLambda');
        this.#projectTangentialLambda(contact, contact.frictionCoefficient);
        this.#projectTwistLambda(contact);
        return contact.normalLambda - previous;
    }

    accumulateNormalLambda(contactOrId, delta) {
        const contact = resolveContact(this._contacts, contactOrId);
        finiteNumber(delta, 'normal lambda delta');
        return this.setNormalLambda(
            contact,
            Math.max(0, contact.normalLambda + delta)
        );
    }

    accumulateKnownNormalLambda(contact, delta) {
        if (contact?._manifold !== this) {
            throw new RangeError('Unknown Kirchhoff contact');
        }
        const previous = contact.normalLambda;
        contact.normalLambda = Math.max(0, previous + delta);
        this.#projectTangentialLambda(contact, contact.frictionCoefficient);
        this.#projectTwistLambda(contact);
        return contact.normalLambda - previous;
    }

    /**
     * Accumulates a two-component surface impulse and projects the total onto
     * the Coulomb disk ||lambda_t|| <= mu * lambda_n. The returned correction,
     * rather than the requested delta, is what a solver must apply to its rod
     * degrees of freedom.
     */
    accumulateTangentialLambda(
        contactOrId,
        deltaU,
        deltaV,
        { frictionCoefficient, out = null } = {}
    ) {
        const contact = resolveContact(this._contacts, contactOrId);
        finiteNumber(deltaU, 'tangential lambda deltaU');
        finiteNumber(deltaV, 'tangential lambda deltaV');
        const friction = nonNegative(
            frictionCoefficient ?? contact.frictionCoefficient,
            'frictionCoefficient'
        );
        contact.frictionCoefficient = friction;
        this.#projectTwistLambda(contact);
        const previousU = contact.tangentLambda[0];
        const previousV = contact.tangentLambda[1];
        contact.tangentLambda[0] += deltaU;
        contact.tangentLambda[1] += deltaV;
        const projection = this.#projectTangentialLambda(contact, friction);
        const limit = friction * contact.normalLambda;
        const result = out ?? {};
        result.appliedU = contact.tangentLambda[0] - previousU;
        result.appliedV = contact.tangentLambda[1] - previousV;
        result.lambdaU = contact.tangentLambda[0];
        result.lambdaV = contact.tangentLambda[1];
        result.limit = limit;
        result.clamped = projection;
        return result;
    }

    accumulateKnownTangentialLambda(
        contact,
        deltaU,
        deltaV,
        { frictionCoefficient, out = null } = {}
    ) {
        if (contact?._manifold !== this) {
            throw new RangeError('Unknown Kirchhoff contact');
        }
        const friction = frictionCoefficient ?? contact.frictionCoefficient;
        contact.frictionCoefficient = friction;
        this.#projectTwistLambda(contact);
        const previousU = contact.tangentLambda[0];
        const previousV = contact.tangentLambda[1];
        contact.tangentLambda[0] += deltaU;
        contact.tangentLambda[1] += deltaV;
        const projection = this.#projectTangentialLambda(contact, friction);
        const result = out ?? {};
        result.appliedU = contact.tangentLambda[0] - previousU;
        result.appliedV = contact.tangentLambda[1] - previousV;
        result.lambdaU = contact.tangentLambda[0];
        result.lambdaV = contact.tangentLambda[1];
        result.limit = friction * contact.normalLambda;
        result.clamped = projection;
        return result;
    }

    /**
     * Records one generalized angular impulse for the relative material-frame
     * twist constraint. The inner and outer rods always receive equal and
     * opposite impulses; integration into their angular velocities is left to
     * the owning Kirchhoff solver.
     */
    accumulateTwistImpulse(
        contactOrId,
        deltaTwistImpulse,
        { frictionCoefficient, effectiveRadius, out = null } = {}
    ) {
        const contact = resolveContact(this._contacts, contactOrId);
        finiteNumber(deltaTwistImpulse, 'deltaTwistImpulse');
        if (frictionCoefficient !== undefined) {
            contact.twistFrictionCoefficient = nonNegative(
                frictionCoefficient,
                'frictionCoefficient'
            );
        }
        if (effectiveRadius !== undefined) {
            contact.effectiveTwistRadius = nonNegative(
                effectiveRadius,
                'effectiveRadius'
            );
        }
        const previous = contact.twistLambda;
        contact.twistLambda += deltaTwistImpulse;
        const projection = this.#projectTwistLambda(contact);
        const limit = contact.twistFrictionCoefficient *
            contact.effectiveTwistRadius * contact.normalLambda;
        const applied = contact.twistLambda - previous;
        const result = out ?? {};
        result.appliedInner = applied;
        result.appliedOuter = -applied;
        result.inner = contact.innerTwistImpulse;
        result.outer = contact.outerTwistImpulse;
        result.limit = limit;
        result.clamped = projection;
        return result;
    }

    accumulateKnownTwistImpulse(
        contact,
        deltaTwistImpulse,
        { frictionCoefficient, effectiveRadius, out = null } = {}
    ) {
        if (contact?._manifold !== this) {
            throw new RangeError('Unknown Kirchhoff contact');
        }
        if (frictionCoefficient !== undefined) {
            contact.twistFrictionCoefficient = frictionCoefficient;
        }
        if (effectiveRadius !== undefined) {
            contact.effectiveTwistRadius = effectiveRadius;
        }
        const previous = contact.twistLambda;
        contact.twistLambda += deltaTwistImpulse;
        const projection = this.#projectTwistLambda(contact);
        const applied = contact.twistLambda - previous;
        const result = out ?? {};
        result.appliedInner = applied;
        result.appliedOuter = -applied;
        result.inner = contact.innerTwistImpulse;
        result.outer = contact.outerTwistImpulse;
        result.limit = contact.twistFrictionCoefficient *
            contact.effectiveTwistRadius * contact.normalLambda;
        result.clamped = projection;
        return result;
    }

    clearLambdas(contactOrId) {
        const contact = resolveContact(this._contacts, contactOrId);
        contact.normalLambda = 0;
        contact.tangentLambda.fill(0);
        contact.twistLambda = 0;
        contact.innerTwistImpulse = 0;
        contact.outerTwistImpulse = 0;
        return contact;
    }

    clear() {
        for (const contact of this._contacts.values()) {
            contact._manifold = null;
        }
        this._contacts.clear();
    }

    #projectTangentialLambda(contact, frictionCoefficient) {
        const limit = frictionCoefficient * contact.normalLambda;
        const magnitude = Math.sqrt(
            contact.tangentLambda[0] * contact.tangentLambda[0] +
            contact.tangentLambda[1] * contact.tangentLambda[1]
        );
        if (magnitude <= limit + EPSILON) {
            return false;
        }
        const scale = magnitude > EPSILON ? limit / magnitude : 0;
        contact.tangentLambda[0] *= scale;
        contact.tangentLambda[1] *= scale;
        return true;
    }

    #projectTwistLambda(contact) {
        const limit = contact.twistFrictionCoefficient *
            contact.effectiveTwistRadius * contact.normalLambda;
        const requested = contact.twistLambda;
        contact.twistLambda = Math.max(-limit, Math.min(limit, requested));
        contact.innerTwistImpulse = contact.twistLambda;
        contact.outerTwistImpulse = -contact.twistLambda;
        return Math.abs(requested) > limit + EPSILON;
    }
}
