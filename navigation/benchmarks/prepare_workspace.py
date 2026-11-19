#!/usr/bin/env python3
"""Generate an ignored build workspace whose algorithm packages use canonical sources."""
import os
from pathlib import Path

ROOT=Path(__file__).resolve().parent
CORE=ROOT.parents[1]
WORKSPACE=ROOT/'.workspaces/jazzy'

def link(source,destination):
 destination.parent.mkdir(parents=True,exist_ok=True)
 target=os.path.relpath(source,destination.parent)
 if destination.is_symlink():
  if os.readlink(destination)==target:return
  destination.unlink()
 elif destination.exists():
  raise RuntimeError(f'Refusing to replace non-symlink source: {destination}')
 destination.symlink_to(target,target_is_directory=True)

def main():
 for name in ['bspline_opt','path_searching','plan_env','plan_manage','scan_planner_msgs','traj_utils']:
  link(CORE/f'navigation/scan_planner/src/planner/{name}',WORKSPACE/f'src/slash_planner/{name}')
 link(CORE/'navigation/scan_planner/src/perception/geometric_obstacles',WORKSPACE/'src/slash_planner/geometric_obstacles')
 link(CORE/'navigation/scan_planner/src/simulator/map_generator',WORKSPACE/'src/slash_planner/map_generator')
 link(ROOT/'static_barn/upstream',WORKSPACE/'src/The-Barn-Challenge-Ros2')
 link(ROOT/'dynamic_dynabarn/dynabarn_ros2',WORKSPACE/'src/dynabarn_ros2')
 print(WORKSPACE)
if __name__=='__main__':main()
