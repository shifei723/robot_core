#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
namespace scan_planner {
template<class Free>
bool brakingMotionFree(Eigen::Vector3d pose,double yaw,double candidate_v,double candidate_w,
    double measured_v,double measured_w,double acceleration,Free free) {
  if(!free(pose,yaw))return false;
  double v=std::max(candidate_v,std::max(0.,measured_v)),w=measured_w;
  const double dt=.025,reaction=.12;
  const double horizon=reaction+v/acceleration+.05;
  for(double t=0;t<horizon;t+=dt){
    w+=std::clamp(candidate_w-w,-2.*dt,2.*dt);
    if(t>=reaction)v=std::max(0.,v-acceleration*dt);
    pose.x()+=v*dt*std::cos(yaw+w*dt*.5);
    pose.y()+=v*dt*std::sin(yaw+w*dt*.5);yaw+=w*dt;
    if(!free(pose,yaw))return false;
  }
  return true;
}
}
