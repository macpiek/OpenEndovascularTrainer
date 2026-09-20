/** Switch only between physical timesteps. Each entry retains its own solver,
 * material history and scene objects. Both solvers may run in bounded slices. */
export function createFemoralAccessController({initial, entries, capture, restore,
    isPending, beforeSwitch = () => {}, afterSwitch = () => {}}) {
    let activeId = initial, executionId = initial, requested = null;
    const values = Object.values(entries);
    if (!entries[initial]) throw new Error('Unknown initial femoral access');
    return {
        get activeId() { return activeId; },
        get active() { return entries[activeId]; },
        get switchRequested() { return requested !== null; },
        values: () => values,
        request(id) {
            if (!entries[id]) throw new Error(`Unknown femoral access: ${id}`);
            requested = id;
        },
        applyRequested() {
            if (requested === null || isPending()) return false;
            const next = requested;
            requested = null;
            if (next === activeId) return false;
            beforeSwitch();
            Object.assign(entries[activeId], capture());
            restore(entries[next]);
            activeId = executionId = next;
            afterSwitch();
            return true;
        },
        // This operation is synchronous. Solver callbacks see the selected
        // execution context, while the UI/control owner remains unchanged.
        run(id, operation) {
            if (!entries[id]) throw new Error(`Unknown femoral access: ${id}`);
            const previous = executionId;
            Object.assign(entries[previous], capture());
            restore(entries[id]);
            executionId = id;
            try { return operation(); }
            finally {
                Object.assign(entries[id], capture());
                restore(entries[previous]);
                executionId = previous;
            }
        },
        getCompletedSteps() {
            const current = capture();
            return Object.fromEntries(Object.entries(entries).map(([id, saved]) =>
                [id, (id === executionId ? current : saved).simulationExecutedSteps ?? 0]));
        },
        getReport() {
            const current = capture();
            return Object.entries(entries).map(([id, saved]) => {
                const state = id === executionId ? current : saved;
                return {id, active:id === activeId, guidewireMm:state.guidewireTransport.progress,
                    catheterMm:state.pigtailCatheter.progress, sheath:state.activeSheath,
                    steps:state.simulationExecutedSteps ?? 0, pending:state.simulationStepTransaction?.pending ?? false};
            });
        }
    };
}
