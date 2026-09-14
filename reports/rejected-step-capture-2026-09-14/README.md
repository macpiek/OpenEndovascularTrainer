# Rejected shared-axis step capture

A terminal rejection now retains one detached replay record. Successful steps and
cooperative yields do not capture the full native state. Each attempted subdivision
adds a small diagnostic summary (at most 15 substeps per application step).

The report includes:
- The last accepted axis coordinates/positions, both material frames and profiles,
  reactions, loads, fixed DOFs, velocities and dynamic/friction history.
- Accepted wall gaps, which influence which old wall rows survive the next feed.
- The sheath, timestep, previous rotations, requested tool insertions/rotations and
  serializable solver options.
- Final status/error, residuals, counters/timings and results of subdivision/fallback
  attempts. Exceptions and incomplete captures are explicitly identified.

Debug → **Pobierz zapis odrzuconego kroku** downloads JSON. The nearby text area
shows a concise summary. One report is retained in sessionStorage across refreshes
of the same tab. If storage is denied/full, it remains downloadable from memory;
the panel reports that persistence failed. Recovery and simulation resets retain
the last rejection as historical evidence. A new rejection replaces it.

From the same solver/anatomy checkout:

```sh
node scripts/physics/replay-shared-axis-step.mjs /path/to/download.json /tmp/oet-replay
```

This runs the captured options and writes `captured-result.json`; a successful
replay also writes `captured-terminal.json`. Exit 1 means the replayed step failed,
which is expected when reproducing a rejection. Optional `--compare-fallback`
runs both fallback settings, writing `reference-*` and `guard-*` instead.
The anatomy itself is loaded from this checkout, not embedded in the report.
Custom JavaScript observers are not serialized.

Validation:
- Actual provider rejection → JSON → restored state → same failed status and all
  subdivision results; subsequent commands/reset do not mutate the record.
- Existing anatomy checkpoint, retained-knot and material/friction round trips.
- UI download/persistence and storage-quota failure tests.
- Diagnostic presentation failure cannot leave terminal timestep debt behind.
- Browser Debug panel is present and no JavaScript errors were reported at startup.

This instruments future failures; it does not recover a rejection that occurred
before this code was loaded, and does not change convergence tolerances or physics.

Final checks: 38/38 targeted tests pass; full shared-axis suite 194/196 passes.
The two unchanged failures are the frozen inconsistent-contact anatomy audit
(`shared-axis-wall-discovery`) and the historical Pigtail live-fallback-count
assertion (0 instead of 1). Production build passes. The replay CLI was also
checked on the saved Pigtail withdrawal checkpoint: converged, 14 iterations,
144 factorizations, with captured options unchanged.
