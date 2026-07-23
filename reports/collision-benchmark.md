# Collision benchmark

Generated: 2026-07-22T20:32:51.839Z

Host: Apple M3, 8 logical CPUs, 8.0 GB RAM

| Mode | Steps | Physics avg | Physics p95 | Narrow avg | Narrow p95 | Max penetration | Length error | Heap delta |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| legacy | 2968 | 14.199 ms | 31.159 ms | 11.597 ms | 25.761 ms | 1.995 mm | 0.000% | 1.65 MB |
| xpbd-contact-v1 | 2400 | 0.255 ms | 0.509 ms | 0.019 ms | 0.058 ms | 0.018 mm | 0.177% | 0.29 MB |

Scenarios:
- legacy: full-insert, full-withdraw, stenosis, pigtail-deploy-rotate, berenstein-deploy-rotate
- xpbd-contact-v1: full-insert, small-branch, stenosis, taper, wire-inside-catheter, external-wire-catheter, pigtail-deploy-rotate, berenstein-deploy-rotate, full-withdraw, sheath
- xpbd-contact-v1 acceptance: PASS

> Node timings are deterministic engineering comparisons, not the final Chrome/Safari M3 acceptance run.
