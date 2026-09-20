# Conservative arterial mesh repair

The baseline and original infrarenal-aneurysm STL files had identical topology
defects. They were repaired independently with `scripts/anatomy/repair-vessel-mesh.py`.
Subsequent aneurysm generation inherits the repaired baseline connectivity;
its mesh manifest records the additional deformation and current output hash.

| Metric, per model | Before | After |
| --- | ---: | ---: |
| Triangles | 1,266,871 | 1,309,328 |
| Zero-area triangles | 506 | 0 |
| Duplicate triangles | 6 | 0 |
| Unmatched edges | 51,088 | 3,786 |
| Edges with more than two incident faces | 558 | 119 |
| Inconsistent paired-edge orientation | 84 | 25 |

The zero-area and duplicate counts overlap and must not be summed. Unmatched
edges are a topology metric, not a count of physical holes. In particular,
the existing finite-thickness outlet plugs overlap the artery wall without
sharing its vertices.

The repair welds compatible boundary vertices within 0.0001 source units
(approximately 0.000097 mm), splits T-junctions, and orients manifold patches.
It rejects welds/splits that create non-manifold edges or incompatible winding.
It never automatically fills anatomical openings or changes artery diameters.
The extra triangles make neighboring faces conform; this is not decimation.

Independent symmetric sampling compared 10,055 original and 10,072 repaired
triangle centroids per model. The maximum measured deviation was approximately
0.0000174 mm. This is a sampled shape check, not a global Hausdorff bound or
a proof that the surface has no self-intersections. Every vertex outside the
aneurysm support is identical between the two repaired variants.

The remaining 119 non-manifold edges, 25 orientation inconsistencies, and one
non-orientable patch are retained as explicit limitations. Automatic solid
reconstruction there could join neighboring vessels or obstruct a lumen.
No claim of a fully watertight manifold is made.

Per-model hashes, counts, tolerance, and operations are recorded in
`res/Aorta_plain.mesh-repair.json` and
`res/Aorta_infrarenal_aneurysm.mesh-repair.json`. Both collision assets are
rebuilt from their repaired STL; centerline positions and branch IDs are
preserved exactly and validated against the repaired wall.

Validation commands:

```sh
python3 tests/vesselMeshRepair.test.py  # requires NumPy
npm run test:anatomy:mesh
node --test tests/femoralAccess.test.js tests/stentGraft.test.js
npm run build
```

The closure checks probe all 41 caps at their centers and near their rims
(3,977 probes), plus containment and tool contact at the aortic root. The
anatomy checks cover subclavian clearance and the aneurysm's connected lumen.
