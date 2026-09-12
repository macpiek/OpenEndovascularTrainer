# Production World whole-dt hook — frozen handoff

`EndovascularPhysicsWorld` now accepts optional `wholeStepSystem=null`. A configured provider receives the entire prepared fixed dt before the original predictor, integrate, damping, constraint, contact and friction passes. Those old passes are bypassed completely, including after a rejection/exception; there is no fallback. This is the World integration hook, tested with a controlled synchronous provider. The actual physical adapter/importer is parent-owned work. No simulator/selection/UI file changed and no FPS or completed-physics claim is made.

## Contract

```js
new EndovascularPhysicsWorld({
  wholeStepSystem: {
    id: 'a-nonempty-stable-system-id',
    step(world, dt) {
      // Own private candidates, physical history and atomic body publication.
      // On rejection/exception, roll back before returning/throwing.
      return { accepted: true, dt, status: 'accepted', diagnostics: { /* owned snapshot */ } };
    },
    reset(world) { /* clear own accepted/pending adapter state */ }
  }
})
```

World validates the provider methods/id, exact returned dt, boolean accepted, nonempty status, and synchronous result. Promise/thenable results cannot commit World clocks. The provider must publish body/own-state changes atomically only on success and return stable diagnostic snapshots; World deliberately does not take a legacy split snapshot of the new physical state. Contract-violating providers that publish before producing a valid success are not made transactional by this wrapper. The provider must leave World stepCount/accumulator/configuration ownership to World.

Accepted steps increment `stepCount` once. An `advance`-prepared dt keeps its identity (dt, system object/id/step/reset) across false results or thrown provider exceptions; its `beforeSubstep` callback runs only once. It consumes accumulator time only after success, including a direct `stepFixed()` retry. A failed direct `stepFixed()` started without `advance` keeps its prepared dt but does not subtract elapsed time that was never queued; a later `advance` may fund that same pending step from its actual backlog. `maxSubsteps` retains remaining elapsed time, and a false result stops the current advance attempt.

System/dt changes while pending reject before a new advance changes the accumulator. Exceptions from the provider propagate, preserve pending inputs/clocks, and publish `lastStepResult={accepted:false,status:'whole-step-error',...}`. A partially executed `beforeSubstep` exception is retained as `whole-step-preparation-error`: neither the callback nor provider is run on retry until explicit `resetSimulationState()` abandons that incomplete preparation. Falsy JavaScript thrown values are also retained. Reentrant stepping/reset and provider clock mutation fail explicitly.

`resetSimulationState()` clears pending time and invokes the current provider's reset; when explicitly abandoning a pending transaction after a rejected system change it also resets that old pending owner. The existing body/World reset semantics remain in place. A null provider retains the original default and split routes, their return behavior and scheduling.

The total timing sample covers preparation in the configured whole-step advance, provider work, failed work and provider rollback; retry does not count the first preparation twice. Original physics phase counters are not incremented by a whole-step attempt. `getStats()` exposes `mode:'whole-step'`, `wholeStepSystem:id`, `wholeStep:lastStepResult`, and no stale split `jointMotion` certificate. Caller-level rAF/idle retry throttling remains in the existing scheduler; no simulator scheduling changes were included.

## Validation

- New controlled-provider tests: **12/12 PASS** in the worktree and immutable source snapshot. Actual World and Body traps prove old prediction/damping/contact hooks are bypassed. Tests cover accept, early reject, exception/rollback, direct retry, funded versus unfunded pending dt, backlog/maxSubsteps, exact timing accounting, reset, malformed results, method/id/dt guards, reentrancy, and explicit new-step selection.
- Existing split transaction tests: **12/12 PASS**.
- Existing scheduler/real-World transaction tests: **12/12 PASS**, including real tool transport preparation/retry. Combined applicable transaction result: **36/36 PASS**.
- World source syntax: PASS.
- Broader `endovascularPhysicsWorld.test.js`: FAIL at existing profile assertion line 630 (`60 !== 18`). The unchanged guidewire profile sets `inheritRodStateBendLimit:false` and `maxBendAngle:60`; that test expects an inherited 18-degree RodState limit. `body-control.mjs/json` reproduces 60 with only the unchanged Body/profile and with the whole World class absent. This patch does not change that physical parameter or relax the old assertion; the monolithic test's later cases are not claimed passing.
- Mocked timing control gives 3 units preparation + 5 provider work + 2 rollback = 10 first-attempt units, then 5 units for the accepted retry. This is a clock-accounting test, not latency measurement.

## Frozen files

- Owned source: `src/physics/endovascularPhysicsWorld.js` SHA `a718d57a74accaa0a97b9f06e10fc722556b3d6e71241fe7f360cd2303793ad0`.
- New test: `tests/kirchhoffCompositeWholeStepWorld.test.js` SHA `80ae69112a5eca0007ece9b2fa2a054e2fb4234c903f587aefbb90e6b5147e86`.
- Bundle: `/tmp/oet-composite-whole-step-world-final`.
- Reproduce new tests: `node --test /tmp/oet-composite-whole-step-world-final/stage/tests/kirchhoffCompositeWholeStepWorld.test.js`.
- Reproduce unrelated Body control: `node /tmp/oet-composite-whole-step-world-final/body-control.mjs`.
- `source.json` freezes 47 transitive source/test files; source check at handoff is `True`. Node and npm packages are borrowed via the worktree's node_modules link. `manifest.json` additionally freezes evidence, logs, and the no-World Body control.
- Parent owns package test registration and physical adapter integration. No other existing source files were modified by this bounded task.
