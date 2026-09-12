# Share immutable contact-coordinate preparation

Normal and friction contact mappings previously revalidated and copied the whole chart for each local sample. The surface mapping additionally rebuilt the canonical chain and serialized its full layout twice per sample. With S samples and N nodes this repeated O(S*N) chart work in every dt, although each actual mapping has only local support.

`createCompositeContactPullbackFactory` and `createCompositeJointSurfacePullbackFactory` now validate and own the chart once, then build independent local numeric mappings. Their closure contains copied layout/basis data. `matches()` compares the actual relevant index/basis values, not caller object identity. The public single-mapping constructors retain their behavior.

Both lumen and vessel normal managers use one factory per prepared manager. Both friction workspaces retain one owned surface-chart factory across sequential dt; a changed chart replaces it after the original validation. Contact tool/edge identities are bound afresh. Every sample still owns its numeric arrays, normal/tangential reactions and material history. Geometry, frames, feed maps, radii, coefficients, force columns and Hessians are not cached as current. Existing manager generations still reject old leases.

## Whole-step A/B

The baseline includes the preceding needed-derivative optimization. Both sides have the same full step solver, fixture, mechanics, inputs and acceptance criteria. Each case uses 10 alternating warmup pairs and 24 measured pairs, with reusable workspaces. Accepted states, material momenta, reactions, force balances and physical certificates are exactly equal, as are directions, evaluations and original query counts.

| Synthetic case | Preparation median, before → after | Whole-step median, before → after | Whole-step P95, before → after |
| --- | ---: | ---: | ---: |
| Open lumen, 65 nodes | 28.027 → 7.073 ms | 42.067 → 22.154 ms | 56.435 → 31.618 ms |
| Loaded lumen, 17 nodes | 3.080 → 1.844 ms | 32.571 → 29.892 ms | 35.620 → 32.947 ms |
| Loaded lumen, 65 nodes | 26.594 → 6.774 ms | 148.051 → 127.032 ms | 178.304 → 148.151 ms |

This is Node with declared synthetic material parameters, not anatomy or browser FPS. These times still fail the target budget. Do not combine percentages from separate historical benchmark runs. The loaded 65-node case still uses 9 directions, 58 evaluations and 7552 queries; its remaining iteration median is 120.177 ms. Local mapping/row numeric buffers are still constructed per manager, so this does not complete all preparation or allocation optimization.

`benchmark.json` preserves the measured source hashes and raw records. Two factory documentation comments were expanded afterwards; executable code is unchanged. For reproduction, make two isolated copies with identical dependencies, apply `baseline.patch` only to the baseline, and run `node scripts/benchmark-composite-contact-preparation.mjs BASELINE_ROOT CANDIDATE_ROOT output.json`.

## Verification and scope

**784/784 composite tests PASS**, 16.937 s. Build PASS, 1.65 s, with the existing bundle-size warning. New tests compare shared-factory mappings with independent dense and cold operators, prove that numeric arrays are separate, preserve the owned map after caller mutations, rebuild on changed bases/layout and retain fresh semantic IDs/history on a chart hit. The production wall, actual-Aorta, coupled friction, feed and rollback suites also pass.

This removes repeated chart construction from the existing full common solver. It does not establish the final reduced mechanical model, adaptive discretization or application integration. General material/angular history transport, sheath/portal/tip/external source laws, loaded anatomy lifecycle, mesh/range transfer, UI selection and deep/max insertion 60 FPS/120 Hz validation remain unfinished. The active goal is unchanged.
