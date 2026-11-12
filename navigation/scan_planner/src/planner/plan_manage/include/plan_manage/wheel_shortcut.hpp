#pragma once
#include <Eigen/Core>
#include <vector>
#include <cmath>
namespace scan_planner {
template<class CollisionFree>
std::vector<Eigen::Vector3d> shortcutWheelPath(const std::vector<Eigen::Vector3d>& path,
    double spacing,CollisionFree free) {
  if(path.size()<3 || !(spacing>0)) return path;
  std::vector<Eigen::Vector3d> result{path.front()};
  size_t current=0;
  while(current+1<path.size()) {
    size_t next=current+1;
    for(size_t candidate=path.size()-1;candidate>current+1;--candidate) {
      const Eigen::Vector3d delta=path[candidate]-path[current];
      const double yaw=std::atan2(delta.y(),delta.x());
      const int n=std::max(1,static_cast<int>(std::ceil(delta.norm()/spacing)));
      bool clear=true;
      for(int k=0;k<=n;++k) if(!free(path[current]+delta*(double(k)/n),yaw)) {clear=false;break;}
      if(clear) {next=candidate;break;}
    }
    result.push_back(path[next]);current=next;
  }
  return result;
}
}
