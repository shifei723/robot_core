#!/usr/bin/env python3
"""Port Classic DynaBARN worlds without executing a Classic/ROS 1 process."""
import argparse,copy,hashlib,json,re
from pathlib import Path
import xml.etree.ElementTree as E
from extract_trajectory import extract

def convert(world_file,plugins,base_world,out):
 raw=world_file.read_text();fixed=re.sub(r'frame\s*=\s*>','frame="">',raw).replace('\\','')
 tree=E.ElementTree(E.fromstring(fixed));world=tree.getroot().find('world');world.set('name','default')
 for child in list(world):
  if child.tag in ('include','plugin','physics'):world.remove(child)
 base=E.parse(base_world).find('world')
 for child in list(base):world.insert(0,copy.deepcopy(child))
 for pose in world.iter('pose'):
  if pose.get('frame')=='':pose.attrib.pop('frame')
 coords=E.SubElement(world,'spherical_coordinates')
 for k,v in [('surface_model','EARTH_WGS84'),('world_frame_orientation','ENU'),('latitude_deg','0'),('longitude_deg','0'),('elevation','0'),('heading_deg','0')]:E.SubElement(coords,k).text=v
 manifest={'source_world':str(world_file.resolve()),'source_sha256':hashlib.sha256(world_file.read_bytes()).hexdigest(),
 'runtime':'ROS 2 Jazzy + Gazebo Harmonic; no ROS 1 or Classic shared libraries',
 'motion':'Absolute world PoseAnimation keyframes; original looping including endpoint jumps is retained',
 'xml_repairs':['quote empty frame attributes','remove stray backslash text'],'objects':[]}
 for model in world.findall('model'):
  for old in list(model.findall('plugin')):
   filename=old.get('filename');path=plugins/filename
   if not path.exists():
    options=[plugins/(Path(filename).stem+ext) for ext in ('.cc','.cpp')]
    path=next((x for x in options if x.exists()),path)
   motion=extract(path);model.remove(old)
   p=E.SubElement(model,'plugin',{'filename':'libdynabarn_motion.so','name':'dynabarn::Motion'})
   E.SubElement(p,'duration').text=str(motion['duration'])
   for t,x,y,z in motion['keyframes']:
    k=E.SubElement(p,'keyframe');E.SubElement(k,'time').text=str(t);E.SubElement(k,'position').text=f'{x} {y} {z}'
   # Initialize at the actual t=0 keyframe rather than the old placeholder pose.
   pose=model.find('pose');pose.text=' '.join(map(str,motion['keyframes'][0][1:]))+' 0 0 0';pose.attrib.clear()
   motion.update(name=model.get('name'),radius=float(model.findtext('.//collision/geometry/cylinder/radius')),
                 length=float(model.findtext('.//collision/geometry/cylinder/length')))
   manifest['objects'].append(motion)
  for material in model.findall('.//material'):
   for script in list(material.findall('script')):material.remove(script)
   if material.find('diffuse') is None:E.SubElement(material,'diffuse').text='.8 .15 .1 1'
   if material.find('ambient') is None:E.SubElement(material,'ambient').text='.6 .1 .05 1'
  if model.get('name') in ('ground_plane','goal'):continue
  for link in model.findall('link'):
   collision=link.find('collision')
   if collision is None:continue
   sensor=E.SubElement(link,'sensor',{'name':'dynabarn_contact','type':'contact'})
   E.SubElement(E.SubElement(sensor,'contact'),'collision').text=collision.get('name')
   E.SubElement(sensor,'always_on').text='true'
  if model.find('.//collision') is not None:
   touch=E.SubElement(model,'plugin',{'filename':'gz-sim-touchplugin-system','name':'gz::sim::systems::TouchPlugin'})
   for key,val in [('target','robot'),('namespace','robot'),('time','0.001'),('enabled','true')]:E.SubElement(touch,key).text=val
 if not manifest['objects']:raise ValueError('No dynamic DynaBARN models found')
 out.parent.mkdir(parents=True,exist_ok=True);tree.getroot().set('version','1.9');E.indent(tree)
 tree.write(out,encoding='utf-8',xml_declaration=True)
 manifest['converted_world_sha256']=hashlib.sha256(out.read_bytes()).hexdigest()
 out.with_suffix('.json').write_text(json.dumps(manifest,indent=2)+'\n')
 return manifest

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--world',type=Path,required=True);p.add_argument('--plugins',type=Path,required=True)
 p.add_argument('--base-world',type=Path,default=None);p.add_argument('--output',type=Path,required=True)
 a=p.parse_args()
 if a.base_world is None:
  a.base_world=Path(__file__).resolve().parents[1]/'worlds/harmonic_base.world'
  if not a.base_world.exists():
   from ament_index_python.packages import get_package_share_directory
   a.base_world=Path(get_package_share_directory('dynabarn_ros2'))/'worlds/harmonic_base.world'
 m=convert(a.world,a.plugins,a.base_world,a.output);print(f"Converted {len(m['objects'])} moving models -> {a.output}")
