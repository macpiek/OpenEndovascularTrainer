# Historical whole-step cost before persistent workspace reuse

This is a Node diagnostic of the bounded contact-free full-motion model, not an application FPS result. The [raw records](composite-joint-timestep-pre-reuse-120hz.json) contain 30 warmup pairs and 50 measured pairs at dt=1/120 s, 33 nodes, complete overlap and explicitly declared synthetic material parameters. Each pair starts from the same incoming state; its second step uses the first accepted state's own material velocities. Constructors are rebuilt for both steps. The [probe protocol](../scripts/benchmark-composite-joint-timestep-baseline.mjs) can be rerun against the current sources.

| Step | Total median / P95 (ms) | Preparation median (ms) | Iteration median (ms) | Directions |
| --- | ---: | ---: | ---: | ---: |
| First | 14.799 / 20.725 | 5.120 | 8.660 | 2 |
| Continued | 10.146 / 16.575 | 4.577 | 4.642 | 1 |

The records precede both the ToolLengths gradient-only optimization and persistent JointAssembly/JointTimeStep workspace changes. A later comparison with these historical values cannot isolate either change. The fair workspace measurement must alternate cold construction and persistent reuse using the same current source, input states, acceptance gates and warmup policy. JIT warming and continuation of a physical state are distinct from workspace reuse.

This measurement establishes that even the small contact-free full-relative reference exceeds the target whole-step budget. It supports removing repeated structure construction, then measuring the remaining numerical work and implementing controlled reduction. It does not establish real-profile, lumen, anatomy, deep-insertion, rendering, 120 Hz throughput or 60 FPS acceptance.
