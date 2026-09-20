"""Conforming local subdivision before deformation. Offline dependency: numpy.

Usage: python3 subdivide-stl.py INPUT.stl OUTPUT.stl LANDMARKS.json
Both triangles sharing a split edge receive the same midpoint.
"""
import json
import struct
import sys
import numpy as np

source, destination, landmarks = sys.argv[1:]
transform = json.load(open(landmarks))["transform"]
dtype = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
data = np.fromfile(source, dtype=dtype, offset=84)
vertices, inverse = np.unique(data["v"].reshape(-1, 3), axis=0, return_inverse=True)
faces = inverse.reshape(-1, 3)
vertices = vertices.astype(float)

for iteration in range(8):
    world = vertices - np.array(transform["sourceCenter"])
    world = world[:, [0, 2, 1]]
    world[:, 2] *= -1
    world = world * transform["scale"] + transform["targetCenter"]
    tri = world[faces]
    region = ((np.abs(tri[:, :, 0]).min(1) < 215) &
              (np.abs(tri[:, :, 0]).max(1) > 18) &
              (tri[:, :, 1].max(1) > -70) & (tri[:, :, 1].min(1) < 180) &
              (tri[:, :, 2].max(1) > -105) & (tri[:, :, 2].min(1) < 75))
    selected = faces[region]
    edges = np.concatenate([selected[:, [0, 1]], selected[:, [1, 2]], selected[:, [2, 0]]])
    edges.sort(1)
    edges = np.unique(edges, axis=0)
    long = edges[np.linalg.norm(world[edges[:, 0]] - world[edges[:, 1]], axis=1) > 2.0]
    print(f"Subdivision {iteration}: {len(faces)} faces, {len(long)} long edges", flush=True)
    if not len(long):
        break
    lookup = {tuple(edge): len(vertices) + i for i, edge in enumerate(long)}
    vertices = np.concatenate([vertices, (vertices[long[:, 0]] + vertices[long[:, 1]]) / 2])
    result = []
    for a, b, c in faces:
        ab = lookup.get(tuple(sorted((a, b))))
        bc = lookup.get(tuple(sorted((b, c))))
        ca = lookup.get(tuple(sorted((c, a))))
        count = sum(x is not None for x in [ab, bc, ca])
        if count == 0:
            triangles = [(a, b, c)]
        elif count == 3:
            triangles = [(a, ab, ca), (ab, b, bc), (ca, bc, c), (ab, bc, ca)]
        elif count == 1:
            if ab is not None:
                triangles = [(a, ab, c), (ab, b, c)]
            elif bc is not None:
                triangles = [(b, bc, a), (bc, c, a)]
            else:
                triangles = [(c, ca, b), (ca, a, b)]
        elif ca is None:
            triangles = [(b, bc, ab), (a, ab, c), (ab, bc, c)]
        elif ab is None:
            triangles = [(c, ca, bc), (b, bc, a), (bc, ca, a)]
        else:
            triangles = [(a, ab, ca), (c, ca, b), (ca, ab, b)]
        result.extend(triangles)
    faces = np.array(result)
else:
    raise RuntimeError("Subdivision did not converge")

tri = vertices[faces]
normal = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
normal /= np.maximum(np.linalg.norm(normal, axis=1)[:, None], 1e-30)
output = np.zeros(len(faces), dtype=dtype)
output["n"], output["v"] = normal, tri
with open(destination, "wb") as file:
    file.write(b"Presplit subclavian deformation".ljust(80, b"\0"))
    file.write(struct.pack("<I", len(output)))
    file.write(output.tobytes())
