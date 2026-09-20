import * as THREE from 'three';

export const CATHETER_MARKER_SPACING_MM = 10;
const BAND_LENGTH_MM = 1;

/** Embedded radiopaque bands: one draw call, no additional collision bodies.
 * The published rod uses material coordinates, so adaptive remeshing cannot
 * move the graduations along the catheter. The loop-base marker is zero.
 */
export class CatheterGraduationMarkers extends THREE.InstancedMesh {
    constructor(radius, maxLength, material) {
        const capacity = Math.ceil(maxLength / CATHETER_MARKER_SPACING_MM);
        super(new THREE.CylinderGeometry(radius, radius, BAND_LENGTH_MM, 12), material, capacity);
        this.capacity = capacity;
        this.count = 0;
        this.visible = false;
        this.frustumCulled = false;
        this.renderOrder = 8;
        this.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
        this.userData.radiopaque = true;
        this.userData.spacingMm = CATHETER_MARKER_SPACING_MM;
        this._pose = new THREE.Object3D();
        this._tangent = new THREE.Vector3();
        this._up = new THREE.Vector3(0, 1, 0);
        this._curve = new THREE.CatmullRomCurve3();
    }

    update({enabled, points, path = null, spanMm, loopLengthMm}) {
        this.count = 0;
        this.visible = false;
        if (!enabled || points.length < 2) return;
        if (!path) {
            this._curve.points = points;
            this._curve.updateArcLengths();
            path = this._curve;
            spanMm = path.getLength();
        }
        if (!(spanMm > 0)) return;
        for (let n = 1; n <= this.capacity; n++) {
            const distanceFromTip = loopLengthMm + n * CATHETER_MARKER_SPACING_MM;
            if (distanceFromTip + BAND_LENGTH_MM / 2 > spanMm) break;
            const u = 1 - distanceFromTip / spanMm;
            path.getPointAt(u, this._pose.position);
            path.getTangentAt(u, this._tangent);
            this._pose.quaternion.setFromUnitVectors(this._up, this._tangent);
            this._pose.updateMatrix();
            this.setMatrixAt(this.count++, this._pose.matrix);
        }
        this.instanceMatrix.needsUpdate = true;
        this.visible = this.count > 0;
    }

    dispose() {
        this.geometry.dispose();
        super.dispose();
    }
}
