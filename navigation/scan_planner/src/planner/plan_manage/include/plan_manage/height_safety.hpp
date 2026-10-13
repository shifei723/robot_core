#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include <Eigen/Core>

namespace scan_planner {
// Geometry shared by the controller and unit tests. Input points are occupied
// voxel centres, not ground truth: include the half-cell extent conservatively.
struct HeightSafety {
  double radius{0.25}, offset{0.18}, resolution{0.05};
  double top{0.328}, margin{0.03}, ground{0.0};
  std::vector<Eigen::Vector3d> occupied;

  double maxBaseHeight(const Eigen::Vector3d &base, double yaw) const {
    double ceiling=std::numeric_limits<double>::infinity();
    const double r=radius+resolution*std::sqrt(2.0)*0.5;
    for (const auto &p:occupied) {
      if (p.z()<=ground+resolution || p.z()>1.0+ground) continue;
      if (std::abs(p.x()-base.x())>offset+r || std::abs(p.y()-base.y())>offset+r) continue;
      for (int side: {-1,1}) {
        const double dx=p.x()-base.x()-side*offset*std::cos(yaw);
        const double dy=p.y()-base.y()-side*offset*std::sin(yaw);
        if (dx*dx+dy*dy<=r*r) ceiling=std::min(ceiling,p.z()-resolution*0.5);
      }
    }
    return ceiling-top-margin;
  }

  double sweptMaxBaseHeight(Eigen::Vector3d base, double yaw,
                             double v, double w, double horizon) const {
    double limit=maxBaseHeight(base,yaw);
    const int steps=std::max(1,static_cast<int>(std::ceil(horizon/0.025)));
    const double dt=horizon/steps;
    for(int i=0;i<steps;++i) {
      base.x()+=v*dt*std::cos(yaw+w*dt*0.5);
      base.y()+=v*dt*std::sin(yaw+w*dt*0.5);
      yaw+=w*dt;
      limit=std::min(limit,maxBaseHeight(base,yaw));
    }
    return limit;
  }
};
} // namespace scan_planner
