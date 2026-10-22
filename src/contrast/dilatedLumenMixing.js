// Conservative axial mixing along continuous vessel chains, with extra
// dispersion in dilations. Artificial degree-two centerline nodes do not
// interrupt mixing; actual bifurcations remain separate selective territories.
export class DilatedLumenMixing {
    constructor(network, {includeNormal=false} = {}) {
        this.includeNormal=includeNormal;
        this.network = network;
        this.weights = new Float32Array(network.edges.length);
        for (const edge of network.edges) {
            if (edge.transportExcluded || edge.renderExcluded) continue;
            const radius = (edge.radiusStart + edge.radiusEnd) * 0.5;
            let neck = radius,
                distance = 0,
                upstream = edge;
            // Reach past the expanded sac to its inlet even on its distal half.
            // A shorter window mistakes the wide sac itself for the normal neck.
            while (upstream.parentEdgeIndex >= 0 && distance < radius * 6) {
                upstream = network.edges[upstream.parentEdgeIndex];
                distance += upstream.length;
                neck = Math.min(neck, (upstream.radiusStart + upstream.radiusEnd) * 0.5);
            }
            const ratio = radius / Math.max(0.2, neck);
            const t = Math.max(0, Math.min(1, (ratio - 1.35) / 0.45));
            // Small local radius noise must not turn narrow branches into clouds.
            const size = Math.max(0, Math.min(1, (radius - 6) / 4));
            this.weights[edge.index] = t * t * (3 - 2 * t) * size;
        }
        const eligible=edge=>edge && !edge.transportExcluded && !edge.renderExcluded &&
            (includeNormal || this.weights[edge.index]>0);
        this.regions = [];
        const visited = new Set();
        for (const root of network.edges) {
            if (!eligible(root) || visited.has(root.index)) continue;
            const parent = network.edges[root.parentEdgeIndex];
            if (eligible(parent) && parent.childEdgeIndices.length === 1)
                continue;
            const cells = [];
            let edge = root;
            while (eligible(edge) && !visited.has(edge.index)) {
                visited.add(edge.index);
                for (let cell = 0; cell < edge.cellCount; cell++) cells.push({ edge, cell });
                edge =
                    edge.childEdgeIndices.length === 1
                        ? network.edges[edge.childEdgeIndices[0]]
                        : null;
            }
            if (cells.length)
                this.regions.push({
                    cells,
                    upper: new Float64Array(cells.length),
                    rhs: new Float64Array(cells.length),
                    conductance: new Float64Array(cells.length)
                });
        }
    }

    update(dt, touched, marks, waveform=1) {
        const base = Math.max(0, this.network.hemodynamics.axialDispersionMm2PerS);
        if (!base) return;
        const shear=this.includeNormal
            ? Math.max(0,this.network.hemodynamics.axialFlowDispersionFactor??.04)
            : 0;
        for (const { cells, upper, rhs, conductance } of this.regions) {
            let mass = 0;
            for (let i = 0; i < cells.length; i++) {
                const { edge, cell } = cells[i];
                rhs[i] = marks[edge.index] ? edge.nextMassMg[cell] : edge.massMg[cell];
                mass += rhs[i];
            }
            if (!mass) continue;
            conductance.fill(0);
            for(let i=1;i<cells.length;i++) {
                const a = cells[i - 1],
                    b = cells[i];
                // Do not diffuse through excluded lumen or deployed fabric.
                if (
                    a.edge.transportExcluded ||
                    b.edge.transportExcluded ||
                    a.edge.graftSections?.[a.cell] ||
                    b.edge.graftSections?.[b.cell]
                )
                    continue;
                const area = Math.min(a.edge.areas[a.cell], b.edge.areas[b.cell]);
                const radius = Math.sqrt(area / Math.PI);
                const flow=this.network.getFaceSignedFlowMm3PerS
                    ? this.network.getFaceSignedFlowMm3PerS(b.edge.index,b.cell,waveform)
                    : b.edge.meanFlowMm3PerS;
                const speed = Math.abs(flow) / Math.max(1e-9, area);
                const weight = Math.min(this.weights[a.edge.index], this.weights[b.edge.index]);
                // Reduced unresolved shear dispersion: broaden the front and
                // tail in flowing blood, including ordinary-calibre vessels.
                const diffusion = base + (shear + 0.12 * weight) * speed * radius;
                conductance[i] =
                    (dt * diffusion * area) / ((a.edge.cellLength + b.edge.cellLength) * 0.5);
            }
            // Two volumes have a closed-form solution, including the stiff
            // limit. Use it for either representation (one edge or two).
            if(cells.length===2) {
                const va=Math.max(1e-9,cells[0].edge.volumes[cells[0].cell]);
                const vb=Math.max(1e-9,cells[1].edge.volumes[cells[1].cell]);
                const effectiveVolume=va*vb/(va+vb);
                const transfer=(rhs[0]/va-rhs[1]/vb)*effectiveVolume*
                    (-Math.expm1(-conductance[1]/effectiveVolume));
                rhs[0]=(rhs[0]-transfer)/va;
                rhs[1]=(rhs[1]+transfer)/vb;
            } else {
                // Solve (V + dt L)c = mass. This M-matrix preserves nonnegativity
                // and its column sums equal V, so exchange cannot create iodine.
                for (let i = 0; i < cells.length; i++) {
                    const { edge, cell } = cells[i];
                    const left = conductance[i],
                        right = conductance[i + 1] || 0;
                    const diagonal =
                        Math.max(1e-9, edge.volumes[cell]) +
                        left +
                        right -
                        (i ? left * upper[i - 1] : 0);
                    upper[i] = right / diagonal;
                    rhs[i] = (rhs[i] + (i ? left * rhs[i - 1] : 0)) / diagonal;
                }
                for (let i = cells.length - 2; i >= 0; i--) rhs[i] += upper[i] * rhs[i + 1];
            }
            for (let i = 0; i < cells.length; i++) {
                const { edge, cell } = cells[i];
                if (!marks[edge.index]) {
                    marks[edge.index] = 1;
                    touched.push(edge.index);
                    edge.nextMassMg.set(edge.massMg);
                }
                edge.nextMassMg[cell] = rhs[i] * Math.max(1e-9, edge.volumes[cell]);
            }
        }
    }
}
