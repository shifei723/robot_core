import copy,sys,tempfile,unittest
from pathlib import Path
import numpy as np
import xml.etree.ElementTree as E
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
from map_geometry import fit_circle,static_wall_points

class LaserGeometryTests(unittest.TestCase):
 def test_changing_visible_arcs_do_not_invent_center_motion(self):
  rng=np.random.default_rng(3);center=np.array([2.3,-.7])
  for start in [-2.,-.5,1.]:
   a=np.linspace(start,start+1.7,35);p=center+.65*np.c_[np.cos(a),np.sin(a)]+rng.normal(0,.002,(35,2))
   c,r,e=fit_circle(p);self.assertLess(np.linalg.norm(c-center),.03);self.assertLess(abs(r-.65),.03)
 def test_short_and_straight_surfaces_are_not_circle_tracks(self):
  self.assertIsNone(fit_circle(np.c_[np.arange(6),np.zeros(6)]))
  self.assertIsNone(fit_circle(np.c_[np.linspace(1,3,30),np.zeros(30)]))
 def test_two_merged_targets_are_not_one_confident_circle(self):
  a=np.linspace(-1.5,1.5,40);p=np.r_[np.c_[np.cos(a),np.sin(a)]*.5,np.c_[np.cos(a),np.sin(a)]*.5+[1.0,.1]]
  self.assertIsNone(fit_circle(p))
 def test_static_background_does_not_depend_on_dynamic_truth(self):
  path=ROOT/'worlds/official_world_2.world';expected=static_wall_points(path);tree=E.parse(path)
  for m in tree.find('world').findall('model'):
   if m.find("plugin[@name='dynabarn::Motion']") is not None:
    m.find('pose').text='100 -100 0 0 0 0'
    for key in m.findall('plugin/keyframe/position'):key.text='100 -100 0'
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'world.world';tree.write(p);np.testing.assert_array_equal(static_wall_points(p),expected)

if __name__=='__main__':unittest.main()
