# Composite timestep handoff

Exactly two new files. No existing source or test was modified.

`createCompositeTimeStepState` owns geometry, spin, frame and multiplier arrays. `advanceCompositeTimeStep` accepts prepared per-edge Kinematics inputs, absolute prescribed DOFs, nodal loads, positive original force/torque/length tolerances, positive initial AL penalty, and explicit `torsionMode: 'quasi-static'`. Each active tool needs an explicit spin boundary. Old material velocities must already be sampled at current material labels.

Per dt, material samples, old accepted reference frames, map rates and old material velocities are frozen. Banded elastic GN plus full consistent translational inertia and length GN supplies directions. Exact augmented energy controls backtracking at fixed candidate multipliers and penalty. Candidate physical length multipliers remain transaction-local. Fresh base-force and original length equations gate acceptance independently. Fixed-DOF reactions are published from original equations.

Success returns a new owned state with transported accepted frames, reference winding anchors, physical material endpoint velocities, multipliers, time and step. Failure returns the original state object, leaves every prior array and history intact, and commits no time. Angular velocity, material spin and frame spin remain explicitly unknown (`null`).

Work is bounded by global directions, outer iterations, full assembly evaluations and a maximum number of line-search trials per direction. Diagnostics expose counts, original certificates and rollback/acceptance status. The forthcoming Chain iterative refinement must also be counted globally after its separate frozen handoff; this patch uses the already tested pre-refinement Chain snapshot as instructed.

Validation: 13 timestep tests; 48/48 combined with Length, Kinematics and Chain. Includes rest, opposite feed with rigid translation, nonlinear loaded bending, independent finite differences of the original Lagrangian, independent opposite spins, stretched trial recovery, inconsistent prescribed length rejection, nonzero staged multiplier rollback, every work budget, two successive 120 Hz steps, and frozen material sampling. Owned source bytes match the tested snapshot. `git apply --check` passes against root 901c.

Dependencies and exact SHA-256 hashes are in `manifest.json`. Length is the validator's current e955ea4f source; root imports it separately. The root Chain is changing, so this is not a claim that the new iterative-refinement version has been tested.

Scope excludes contacts, friction, remeshing/history transfer, angular inertia, app clock integration and FPS measurements. No claim of a complete angular dynamic model.
