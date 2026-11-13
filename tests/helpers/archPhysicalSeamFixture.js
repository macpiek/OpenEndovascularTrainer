import {archColumnFixture} from './archColumnFixture.js';

// Dedicated replay wrapper: bulk transport and the unmixed outlet jet are
// independent fields. The stored jet uses the same optical path IDs.
export function archPhysicalSeamFixture(data, {reference = false, frozenTransport = false} = {}) {
    // Retain an independent renderer-only comparison after the combined
    // transport/renderer replay is regenerated.
    const replay = frozenTransport && data.referenceFrames ? {...data, frames: data.referenceFrames} : data;
    const fixture = archColumnFixture(replay, {reference});
    const baseShow = fixture.show;
    fixture.show = index => {
        const frame = baseShow(index);
        fixture.activeFrameIndex = index;
        if (!reference && frame.jet) {
            const uniforms = fixture.surface.material.uniforms;
            uniforms.outletJetCount.value = frame.jet.count;
            for (const [key, values] of Object.entries(frame.jet.vectors))
                values.forEach((value, i) => uniforms[key].value[i].fromArray(value));
            uniforms.outletJetMin.value.fromArray(frame.jet.min);
            uniforms.outletJetMax.value.fromArray(frame.jet.max);
            fixture.surface.projection.outletJet.syncPathMembership();
        }
        // A cropped diagnostic always draws the complete physical lumen.
        // This makes its geometry independent of the concentration under test.
        fixture.surface.material.uniforms.projectionCullActive.value = false;
        fixture.surface.mesh.visible = true;
        const geometry = fixture.surface.mesh.geometry;
        geometry.clearGroups();
        geometry.addGroup(0, geometry.index?.count ?? geometry.attributes.position.count, 0);
        return frame;
    };
    fixture.show(0);
    return fixture;
}
