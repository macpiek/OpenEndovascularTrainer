import { createKirchhoffCompositeMaterialCache } from '../../src/physics/kirchhoffCompositeMaterialCache.js';
import { buildKirchhoffCompositeTopology } from '../../src/physics/kirchhoffCompositeTopology.js';
import { buildKirchhoffCompositeMesh } from '../../src/physics/kirchhoffCompositeMesh.js';
import { kirchhoffMaterialProfile } from '../../src/physics/kirchhoffMaterialProfile.js';
import { createCompositeChainWorkspace, assembleCompositeChain } from '../../src/physics/kirchhoffCompositeChain.js';

/** Exact first cold310.017mm pair from benchmark-composite-material-cache.
 * Wire core boundary294mm and catheter plateau boundary294.017mm create the
 * real, mandatory0.017mm coordinate interval. No coordinate snapping.
 */
export function createCompositeColdMaterialFixture({ elementBackend = 'wasm', cached = true } = {}) {
    const cache = createKirchhoffCompositeMaterialCache();
    const wire = cache.profileTool({ profile: kirchhoffMaterialProfile('glidewire'), materialInterval: [0, 500], insertion: 318, radius: .4445 });
    const catheter = cache.profileTool({ profile: kirchhoffMaterialProfile('berenstein'), materialInterval: [0, 500], insertion: 310.017, radius: .8, innerRadius: .485 });
    const topology = buildKirchhoffCompositeTopology({ wire, catheter }), optional = [0, 318];
    for (const section of topology.sections) {
        const count = Math.ceil((section.end - section.start) / 5);
        for (let i = 1; i < count; i++) optional.push(section.start + (section.end - section.start) * i / count);
    }
    optional.sort((a, b) => a - b);
    const mesh = buildKirchhoffCompositeMesh({ topology, meshCoordinates: optional,
        sampleCenterline: x => [x, .2 * Math.sin(x / 40), .1 * Math.cos(x / 37)],
        spinFields: { wire: s => .001 * s, catheter: s => -.002 * s },
        materialIntegrator: cached ? cache.integrator : null });
    const workspace = createCompositeChainWorkspace(mesh.layout, { elementBackend });
    assembleCompositeChain(mesh.data, workspace);
    return { mesh, workspace, diagonal: new Float64Array(mesh.layout.dofCount).fill(3) };
}
