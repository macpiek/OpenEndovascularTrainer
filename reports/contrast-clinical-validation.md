# Hybrid Contrast Clinical Review Protocol

This protocol validates the simulated angiographic appearance of the
`hybrid-v1` contrast model. It is intended for review by a vascular clinician;
it is not a clinical device validation.

## Hydraulic model under review

The sheath, Berenstein catheter, and pigtail use one pressure/resistance
solver. Device type changes only explicit geometry and ratings: internal
length/diameter, roughness, pressure rating, outlet count/area/direction, and
discharge coefficient. There are no anatomy-specific switches deciding where
reflux is permitted.

For each requested rate, the solver adds Darcy-Weisbach tube loss and outlet
loss:

- `ΔP_tube = f · (L / D) · ρv² / 2`, with laminar, transitional, or turbulent
  friction selected from Reynolds number;
- `ΔP_outlet = ρ · (Q / CdA)² / 2`;
- actual rate is the highest rate for which total loss does not exceed the
  lower of injector pressure limit and device pressure rating.

The actual rate, not the requested rate, drives iodine mass, injection
duration, outlet flow division, jet momentum, reflux, global transport, and
fluoroscopic opacity.

The physical outlet velocity is retained in diagnostics. Because the nozzle
core is below the spatial resolution of the vascular mesh, the visible plume
starts after near-field blood entrainment and is capped at 1.2 m/s in the
resolved solver. Its mixing distance is calculated in outlet diameters and
limited to 1.5 local vessel diameters. At a nearby upstream junction only an
exponentially decaying coherent-core fraction can arrive; the rest mixes in
the source vessel during the same injection step. Local parcels must be fully
entrained within 0.45 s, preventing a late second bolus or detached fragments.

### Provisional training parameters

| Parameter | Sheath | Berenstein | Pigtail |
| --- | ---: | ---: | ---: |
| Length | 110 mm | 1000 mm | 1000 mm |
| Internal diameter | 1.80 mm | 0.97 mm | 0.97 mm |
| Pressure rating | 300 psi | 1050 psi | 1200 psi |
| Outlet coefficient | 0.82 | 0.82 | 0.76 |
| Outlet pattern | one axial outlet | one axial end hole | eight radial side holes |

Common defaults are 1200 psi injector limit, density 1349 kg/m³, and dynamic
viscosity 6.3 mPa·s. Every value shown above is editable in Debug. These are
generic training profiles and must be replaced with the selected product IFU
before any device-specific claim.

At a requested 29.1 ml/s with the defaults, the current deterministic model
predicts 29.1 ml/s through the sheath at about 51 psi, 12.13 ml/s through the
Berenstein at its 1050 psi rating, and 12.71 ml/s through the pigtail at its
1200 psi rating. The UI must show requested and actual rate separately and
display a pressure-limited warning for both catheters.

Reference context: the [Bayer MEDRAD Mark 7 Arterion brochure](https://www.radiologysolutions.bayer.com/sites/g/files/vrxlpx50981/files/2024-04/PP-M-MARK-US-0117-1%20MEDRAD%20Mark%207%20Arterion%20Injection%20System%20Brochure.pdf)
lists a 0.1–45 ml/s fixed-flow range and 100–1200 psi pressure limit; the
[Cordis catheter catalog](https://cordis.com/uploads/productResources/apac/INFINTI-and-SUPERTORQUE-Catheter-Flyer-Australia.pdf)
lists representative 5F flow limits at 8 cP and a 1200 psi rating; and the
[Merit Impress SSCP](https://www.merit.com/wp-content/uploads/2023/11/Impress-SSCP0037_001.pdf)
documents Berenstein-family lengths, internal diameters, and configuration-
dependent pressure ratings. These sources bound plausible inputs; they do not
validate the generic profiles above.

## Fixed setup

- Use the AP projection, 15 pulses/s, contrast opacity 100%, and contrast gain 5.
- Use the default generic iohexol 300 medium.
- Start every scenario after the previous contrast has fully washed out unless
  the scenario explicitly tests a repeated injection.
- Record the full injection and at least five seconds of washout.
- Export the Debug-panel contrast diagnostic line with each recording.

## Scenarios

1. **Iliac sheath injection**
   - Source: Sheath.
   - Injection: 20 ml at 10 ml/s.
   - Expected: a directed proximal plume, limited reflux dependent on the
     injection-to-blood-flow ratio, followed by antegrade iliac washout.

2. **Selective Berenstein injection**
   - Advance the Berenstein end port beyond the sheath and position it in one
     iliac branch.
   - Source: Catheter.
   - Injection: 8 ml at 4 ml/s.
   - Expected: a narrow end-hole jet aligned with the distal catheter tangent,
     preferential filling of the selected downstream territory, and no
     instantaneous opacification of the contralateral branch.

3. **Aortic pigtail injection**
   - Position the released pigtail in the distal aorta above the bifurcation.
   - Source: Catheter.
   - Injection: 30 ml at 15 ml/s.
   - Expected: a multi-directional side-hole plume without a single spear-like
     jet, rapid local mixing, and flow-weighted filling of both iliac systems.

4. **Distal-aortic Berenstein regression (22.3/24.1 cm)**
   - Use Debug → `Odtwórz 22,3/24,1 cm`, then select Catheter.
   - Injection: 30 ml at 15 ml/s.
   - Expected: the end-hole bolus first traverses the approximately 48 mm
     aortic segment above the bifurcation, then enters both common iliac
     arteries in the same flow frame. The black line in the access-side iliac
     before bolus arrival is the catheter, not opacified blood.
   - Record the complete two-second injection. A single early still image is
     not sufficient to score contralateral filling.

5. **Rate comparison**
   - Repeat a 10 ml sheath injection at 2 ml/s and 10 ml/s from the same
     position, allowing complete washout between recordings.
   - Expected: identical delivered iodine mass, with the high-rate injection
     producing a longer reflux/mixing zone and a steeper bolus front.

6. **Residual contrast**
   - Source: Sheath.
   - Inject 15 ml at 10 ml/s, wait two seconds, then repeat without resetting.
   - Expected: physically additive residual opacification, continuous washout,
     and no clearing or restarting of the transport field at the second bolus.

7. **High requested-rate Berenstein jet**
   - Position the end hole in the aorta, directed against blood flow.
   - Request 43.5 ml at 39 ml/s with the default hydraulic profile.
   - Record requested rate, actual rate, applied/required pressure, physical
     outlet velocity, resolved velocity, mixing length, coherent-core
     fraction, and maximum retrograde progress.
   - Expected: pressure-limited actual flow; immediate local aortic opacity;
     simultaneous continuous delivery to any physically reachable nearby
     branch; no spear-like long aortic streak; no dark gap followed by a
     second bolus; and no detached local fragments 0.6 s after delivery ends.

## Quantitative acceptance gates

- Delivered-volume error: at most 0.1 ml or 0.5%, whichever is larger.
- Iodine mass-balance error: below 0.5% for every recording.
- No particle penetration beyond the lumen wall greater than 0.2 mm.
- No visible mass duplication at the local-3D to global-1D handoff.
- No sustained frame-rate reduction greater than 20% relative to the same
  camera view without active contrast.
- Actual flow never exceeds requested flow and applied pressure never exceeds
  either configured pressure limit.
- Physical and resolved jet velocities are both reported; resolved velocity
  is at most 1.2 m/s.
- Maximum local retrograde progress is at most 110% of the reported jet mixing
  length unless the conservative pressure-driven column mode is active.
- Active local-particle count is zero within 0.6 s after injection stops.
- A nearby-junction injection shows local source-vessel opacity and a coherent
  core in the same bolus; no visually separate precursor or late second pulse.

## Clinical scorecard

Score every category from 1 (unacceptable) to 5 (clinically convincing).

| Scenario | Bolus front | Branch order | Reflux | Mixing | Washout | Source-specific jet | Critical artifact |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| Iliac sheath |  |  |  |  |  |  |  |
| Berenstein |  |  |  |  |  |  |  |
| Berenstein 22.3/24.1 cm |  |  |  |  |  |  |  |
| Pigtail |  |  |  |  |  |  |  |
| Rate comparison |  |  |  |  |  |  |  |
| Residual contrast |  |  |  |  |  |  |  |
| High-rate Berenstein jet |  |  |  |  |  |  |  |

Acceptance requires a median score of at least 4 in every category and no
critical anatomical, flow-direction, wall-leak, or rendering artifact.
