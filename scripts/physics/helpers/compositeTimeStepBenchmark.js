import { createKirchhoffCompositeMaterialCache } from '../../../src/physics/kirchhoffCompositeMaterialCache.js';
import { buildKirchhoffCompositeTopology } from '../../../src/physics/kirchhoffCompositeTopology.js';
import { buildKirchhoffCompositeMesh } from '../../../src/physics/kirchhoffCompositeMesh.js';
import { kirchhoffMaterialProfile } from '../../../src/physics/kirchhoffMaterialProfile.js';
import { createCompositeTimeStepState, advanceCompositeTimeStep } from '../../../src/physics/kirchhoffCompositeTimeStep.js';

export const conventions = {
    dt: 1 / 120,
    mass: { name: 'body-nominal-density', wire: 1 / 5, catheter: 1.4 / 4,
        units: 'existing model mass units per material mm; not measured kg',
        provenance: 'DEFAULT_TOOL_PROFILES node masses1/1.4, simulator wire spacing5mm and Pigtail physics spacing4mm. Density=nominal node mass/nominal material spacing; continuous edge inertia retains half-end cells rather than old lumped node counts.' },
    strict: { force: 1e-7, torque: 1e-8, length: 1e-8, linear: 5e-10 },
    runtimeGeometry: { force: 1e-7, torque: 1e-8, length: .002, linear: 5e-10 },
    wallStrict: { gap: 1e-8, force: 1e-7, work: 1e-7 },
    wallRuntimeGeometry: { gap: .001, force: 1e-7, work: 1e-7 },
    toleranceScope: 'Original unit-test force/torque/linear/work thresholds in both series. The separate runtime-geometric series changes only existing World physical length0.002mm and containment0.001mm gates. World has no corresponding original-force/torque thresholds for this model; none are invented.',
    budgets: { directions: 120, outerIterations: 20, lineSearchTrials: 24, evaluations: 1000 },
    wallBudgets: { directions: 150, outerIterations: 32, lineSearchTrials: 30, evaluations: 2000 }
};

export function makeFixture(insertion, scenario) {
    const cache = createKirchhoffCompositeMaterialCache();
    const wire = cache.profileTool({ profile: kirchhoffMaterialProfile('glidewire'), materialInterval: [0, 500], insertion: 318, radius: .4445 });
    const catheter = cache.profileTool({ profile: kirchhoffMaterialProfile('berenstein'), materialInterval: [0, 500], insertion, radius: .8, innerRadius: .485 });
    const topology = buildKirchhoffCompositeTopology({ wire, catheter }), meshCoordinates = [0];
    for (const section of topology.sections) {
        const count = Math.ceil((section.end - section.start) / 5);
        for (let i = 1; i <= count; i++) meshCoordinates.push(i === count ? section.end : section.start + (section.end - section.start) * i / count);
    }
    const height = scenario === 'analytic-plane' ? .82 : 0;
    const mesh = buildKirchhoffCompositeMesh({ topology, meshCoordinates, sampleCenterline: x => [x, height, 0],
        spinFields: { wire: 0, catheter: 0 }, materialIntegrator: cache.integrator });
    return { mesh, topology, state: createCompositeTimeStepState({ data: mesh.data, layout: mesh.layout }), scenario, insertion,
        modelScope: 'Fixed exact-boundary straight common centerline; real nominal material profiles. Common-axis candidate only, no finite-clearance admission or frame/material transfer from two prior rods.' };
}

export function preparedOptions(fixture, state, series) {
    const wall = fixture.scenario === 'analytic-plane', layout = state.layout;
    const inertiaEdges = layout.edgeToolIds.map((ids, edge) => ({ tools: ids.map(id => {
        const map = fixture.mesh.materialMaps.get(id)[edge];
        const accepted = state.materialVelocities?.[edge]?.tools.find(t => t.id === id);
        const initialVelocity = wall ? [0, -10, 0] : [0, 0, 0];
        return { id, massPerMaterialLength: conventions.mass[id], materialMap: { sStart: map.sStart, dsDx: map.dsDx, dsDt: 0 },
            oldMaterialVelocities: accepted ? accepted.velocities.map(v => [...v]) : [[...initialVelocity], [...initialVelocity]] };
    }) }));
    const prescribed = [...layout.spins.values()].map(offsets => ({ dof: offsets.find(v => v >= 0), value: 0 }));
    // Contact-free bending has a clamped proximal handle. The plane control
    // retains the wall test's free common translation and independent spins.
    if (!wall) for (let node = 0; node < 2; node++) for (let axis = 0; axis < 3; axis++)
        prescribed.push({ dof: layout.positions[node] + axis, value: fixture.mesh.data.positions[node][axis] });
    const loads = new Float64Array(layout.dofCount);
    loads[layout.positions.at(-1) + 1] = wall ? -.005 : .005;
    const options = { dt: conventions.dt, torsionMode: 'quasi-static', inertiaEdges, prescribed, loads,
        tolerances: { ...(series === 'strict-tests' ? conventions.strict : conventions.runtimeGeometry) },
        initialPenalty: 1e4, budget: { ...(wall ? conventions.wallBudgets : conventions.budgets) } };
    if (wall) {
        options.tolerances.linear = 1e-10;
        options.wall = { friction: 'frictionless', initialPenalty: 1e5,
            contactOwners: fixture.mesh.contactOwners,
            tolerances: { ...(series === 'strict-tests' ? conventions.wallStrict : conventions.wallRuntimeGeometry) },
            field: { queryCapsuleCoordinates(ax, ay, az, bx, by, bz, radius, out) {
                const t = ay === by ? .5 : ay < by ? 0 : 1;
                out.signedGap = (1 - t) * ay + t * by - radius; out.segmentT = t;
                out.inward.values.set([0, 1, 0]); out.closestPoint.values.set([(1 - t) * ax + t * bx, 0, (1 - t) * az + t * bz]);
                out.faceIndex = 1; out.branchId = 0; out.source = 'analytic-plane'; out.capsuleSampleCount = 2; return out;
            } } };
    }
    return options;
}

export const advance = advanceCompositeTimeStep;
