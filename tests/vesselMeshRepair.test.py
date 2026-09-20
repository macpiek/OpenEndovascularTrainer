"""Run with a Python interpreter containing numpy."""
import unittest, tempfile, subprocess, sys, json, pathlib
import numpy as np

SCRIPT = pathlib.Path(__file__).resolve().parents[1] / 'scripts/anatomy/repair-vessel-mesh.py'
DTYPE = np.dtype([('n','<f4',3),('v','<f4',(3,3)),('a','<u2')])

class RepairTest(unittest.TestCase):
    def repair(self, triangles):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            source, output, report = [root / name for name in ['in.stl','out.stl','report.json']]
            data = np.zeros(len(triangles), dtype=DTYPE)
            data['v'] = triangles
            original = bytes(80) + len(data).to_bytes(4,'little') + data.tobytes()
            source.write_bytes(original)
            subprocess.run([sys.executable, str(SCRIPT), str(source), str(output), str(report)],
                           check=True, capture_output=True)
            self.assertEqual(source.read_bytes(), original, 'source must never be overwritten')
            return json.loads(report.read_text()), np.fromfile(output, dtype=DTYPE, offset=84)

    def test_degenerate_and_duplicate_faces(self):
        tri = [[0,0,0],[1,0,0],[0,1,0]]
        report, data = self.repair([tri,tri,[[0,0,0]]*3])
        self.assertEqual(report['after']['triangles'],1)
        self.assertEqual(report['after']['zeroArea'],0)
        self.assertEqual(report['after']['duplicates'],0)
        np.testing.assert_allclose(data['n'],[[0,0,1]])

    def test_t_junction_is_stitched_without_capping_outer_boundary(self):
        a,b,c,d,m = [0,0,0],[2,0,0],[0,2,0],[0,-2,0],[1,0,0]
        report, data = self.repair([[a,b,c],[a,d,m],[m,d,b]])
        self.assertEqual(report['before']['boundaryEdges'],7)
        self.assertEqual(report['after']['boundaryEdges'],4)
        self.assertEqual(report['after']['nonManifoldEdges'],0)
        self.assertEqual(report['after']['inconsistentWinding'],0)
        area = np.linalg.norm(np.cross(data['v'][:,1]-data['v'][:,0],
                                      data['v'][:,2]-data['v'][:,0]),axis=1).sum()/2
        self.assertAlmostEqual(area,4)

    def test_winding_fixed_without_moving_geometry(self):
        report,data = self.repair([[[0,0,0],[1,0,0],[0,1,0]],
                                  [[1,0,0],[0,1,0],[1,1,0]]])
        self.assertEqual(report['before']['inconsistentWinding'],1)
        self.assertEqual(report['after']['inconsistentWinding'],0)
        self.assertEqual(report['flippedTriangles'],1)
        self.assertEqual(report['after']['boundaryEdges'],4)

    def test_nearby_sheet_is_not_welded_into_a_three_face_edge(self):
        a,b=[0,0,0],[2,0,0]
        report,_=self.repair([[a,b,[0,2,0]], [b,a,[0,-2,0]],
                             [[0,0,.00005],[2,0,.00005],[0,0,2]]])
        self.assertEqual(report['after']['nonManifoldEdges'],0)
        self.assertEqual(report['after']['inconsistentWinding'],0)

if __name__ == '__main__': unittest.main()
