# Ipsilateral delivery contact and cloth preparation

The weak incoming-branch recovery spring (1000) left a 3 mm radius delivery shaft penetrating the septum by 1.757 mm on the right and 1.094 mm on the left in the new regression fixture after 180 accepted settling steps. Matching the ordinary fabric stiffness (100000), with the existing 0.5 mm correction bound per step, passed the same full-radius clearance test (penetration below 0.1 mm). The valid threaded fixture also passes withdrawal/reinsertion by 48 mm with triangle intersection checks on every accepted movement step.

Cloth deformation now uses a spatial lookup over immutable indentation patches in both mechanical preparation and rendering. A seeded 3100-point comparison matches the exhaustive displacement exactly. A delivery-side wire inside the graft is not classified as an exterior wire lifting the cloth.

## Controlled preparation benchmark

Input: `stent-graft-release-incoming.json.gz`, current aneurysm anatomy and main body, after 20 withdrawal steps using the existing interaction test contract. Same accepted state, 525 indentation patches, 2 warmup pairs then 10 alternating measured pairs. Node.js on this workstation.

| Preparation | Before mean | After mean |
|---|---:|---:|
| compliantGraftSurface | 134.81 ms | 15.65 ms |

Ranges were 94.38–289.77 ms before and 9.41–34.69 ms after. This is a preparation-only measurement, not a whole-step or live browser FPS claim. At the earlier, light 24-patch state, both versions were about 3.5 ms; the gain is for accumulated contact regions.

Validation: targeted contact, threading, sliding, withdrawal, interaction and garbage-collection regressions passed; Vite production build passed. Live browser diagnosis before changes showed roughly 2.4 Hz on the active side and a 391 ms completed step, but the exact live pose was not exported/replayed.
