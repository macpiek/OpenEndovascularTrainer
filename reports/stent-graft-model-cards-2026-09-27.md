# Dimensioned stent-graft model cards

Replaced fixed-width body sketches and anisotropically stretched limb sketches with orthographic projections of the simulator's unloaded implant geometry. Both SVG axes use millimetres; all cards have a common viewBox. Larger diameters change the geometry, not just labels.

Shared buildGraftPart now serves the simulator and catalogue geometry. Catalogue views also use the actual junction fitting, IIs oval crotch section, sewn ring layout/kinematics, fixed-arm crown and marker layout. No vessel fitting, delivery pose or patient-specific deformation is applied in the picker. This reproduces the simulator's procedural model, not manufacturer CAD.

Changing proximal or ipsilateral diameter redraws card contents while preserving buttons and selection handlers. Unsupported combinations are resolved with the same fallback used when selecting/loading a model. Captions show each card's actual dimensions. All 30 limb variants use their real distal flare/taper profile. SVGs are cached by configuration, and temporary Three.js resources are disposed.

Validation: 25/25 tests passed (stentGraftControls, stentGraftRotation, stentGraftLimbRelease). Added geometric diameter/length/flare assertions and input-to-preview-to-loaded-device integration coverage. Production build passed with the existing bundle-size warning. Browser checked the real modal markup/CSS and controls in an isolated fixture: 23/14, 32/16, 32/20 and the 124 mm limb filter. Screenshot: screenshots/stent-model-picker-2026-09-27.png.
