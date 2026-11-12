#pragma once
#include <bspline_opt/uniform_bspline.h>
#include <vector>
#include <cmath>

namespace scan_planner {
// Cubic derivative-control-point bounds imply bounds over every knot span.
// Stretch only the supports of offending derivatives, then recheck all supports.
inline bool locallyRetimeWheelSpline(UniformBspline &s, double vmax, double amax,
                                    double height_rate) {
  if (!(vmax>0 && amax>0 && height_rate>0)) return false;
  for (int iteration=0;iteration<100;++iteration) {
    auto u=s.getKnot(); const auto p=s.getControlPoint();
    Eigen::VectorXd scale=Eigen::VectorXd::Ones(u.size()-1);
    Eigen::MatrixXd v(3,p.cols()-1);
    bool feasible=true;
    auto stretch=[&](int first,int last,double ratio) {
      if (ratio<=1.000001) return;
      feasible=false;
      for(int k=first;k<=last;++k) scale(k)=std::max(scale(k),ratio*1.01);
    };
    for(int i=0;i<p.cols()-1;++i) {
      v.col(i)=3.*(p.col(i+1)-p.col(i))/(u(i+4)-u(i+1));
      stretch(i+1,i+3,std::max(v.col(i).head<2>().norm()/vmax,
                              std::abs(v(2,i))/height_rate));
    }
    for(int i=0;i<v.cols()-1;++i) {
      Eigen::Vector3d a=2.*(v.col(i+1)-v.col(i))/(u(i+4)-u(i+2));
      stretch(i+1,i+4,std::sqrt(a.norm()/amax));
    }
    if (feasible) return true;
    auto updated=u;
    for(int k=0;k<scale.size();++k) updated(k+1)=updated(k)+(u(k+1)-u(k))*scale(k);
    if (!updated.allFinite()) return false;
    s.setKnot(updated);
  }
  return false; // Never publish an unverified time allocation.
}

inline bool makeLocallyTimedWheelSpline(const std::vector<Eigen::Vector3d> &path,
                                        double vmax,double amax,double height_rate,
                                        UniformBspline &s) {
  if (path.empty() || !(vmax>0 && amax>0 && height_rate>0)) return false;
  std::vector<Eigen::Vector3d> clean{path.front()};
  std::vector<double> times{0.};
  for (size_t i=1;i<path.size();++i) {
    if (!path[i].allFinite()) return false;
    Eigen::Vector3d d=path[i]-clean.back();
    double dt=std::max(d.head<2>().norm()/vmax,std::abs(d.z())/height_rate);
    if(dt<1e-8) continue;
    clean.push_back(path[i]);times.push_back(times.back()+dt);
  }
  const double dt=.10;
  std::vector<Eigen::Vector3d> points(3,clean.front());
  size_t segment=1;
  for(double t=dt;t<times.back();t+=dt) {
    while(segment+1<times.size() && times[segment]<t) ++segment;
    double f=(t-times[segment-1])/(times[segment]-times[segment-1]);
    points.push_back(clean[segment-1]+f*(clean[segment]-clean[segment-1]));
  }
  for(int i=0;i<3;++i) points.push_back(clean.back());
  Eigen::MatrixXd controls(3,points.size());
  for(size_t i=0;i<points.size();++i) controls.col(i)=points[i];
  s=UniformBspline(controls,3,dt);
  return locallyRetimeWheelSpline(s,vmax,amax,height_rate);
}
}
