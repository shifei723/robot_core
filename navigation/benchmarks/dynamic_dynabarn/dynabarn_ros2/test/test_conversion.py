"""Offline tests require Python 3, binutils, g++; no simulator or ROS 1."""
import importlib.util,json,subprocess,sys,tempfile,unittest
from pathlib import Path
import xml.etree.ElementTree as E
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
from extract_trajectory import extract
from convert_world import convert

class ConversionTests(unittest.TestCase):
 def test_independent_binary_fixture(self):
  with tempfile.TemporaryDirectory() as d:
   out=Path(d)/'fixture.so'
   subprocess.run(['g++','-O0','-fPIC','-shared',str(ROOT/'test/extraction_fixture.cpp'),'-o',str(out)],check=True)
   got=extract(out)
   self.assertEqual(got['duration'],4.5)
   self.assertEqual(got['keyframes'],[[0.,-2.25,3.,0.],[1.25,0.,1.,0.],[4.5,6.,-2.5,0.]])
 def test_wrong_binary_fails_closed(self):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'bad.so';p.write_bytes(b'Not an ELF file')
   with self.assertRaises(ValueError):extract(p)
 def test_world_geometry_and_contact_scope(self):
  world=E.parse(ROOT/'worlds/official_world_2.world').find('world')
  motion=world.findall("model/plugin[@name='dynabarn::Motion']")
  self.assertEqual(len(motion),8)
  self.assertIsNone(world.find("model[@name='ground_plane']/plugin"))
  self.assertEqual(len(world.findall("model/plugin[@name='gz::sim::systems::TouchPlugin']")),11)
  for p in motion:
   keys=p.findall('keyframe');self.assertGreaterEqual(len(keys),2)
   self.assertEqual(float(keys[0].findtext('time')),0.)
   self.assertEqual(float(keys[-1].findtext('time')),float(p.findtext('duration')))
 def test_official_roundtrip_is_deterministic(self):
  source=ROOT.parents[2]/'dynabarn_docker/official'
  if not source.exists():self.skipTest('Optional original downloaded dataset unavailable')
  with tempfile.TemporaryDirectory() as d:
   out=Path(d)/'world.world'
   m=convert(source/'worlds/world_2.world',source/'plugins',ROOT/'worlds/harmonic_base.world',out)
   self.assertEqual(len(m['objects']),8)
   self.assertEqual(out.read_bytes(),(ROOT/'worlds/official_world_2.world').read_bytes())

if __name__=='__main__':unittest.main()
