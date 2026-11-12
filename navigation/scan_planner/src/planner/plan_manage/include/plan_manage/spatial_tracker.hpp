#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <vector>
#include <limits>

namespace scan_planner {
inline double spatialCollisionArrival(double distance,double speed,double acceleration) {
  if(distance<=0.) return 0.;
  speed=std::max(0.,speed);
  if(acceleration>1e-8)
    return 2.*distance/(std::sqrt(speed*speed+2.*acceleration*distance)+speed);
  return speed>1e-8?distance/speed:std::numeric_limits<double>::infinity();
}
struct SpatialSample { Eigen::Vector3d p; double t,arc,curvature; double curvature_rate=0.; };
struct SpatialCommand { double v=0,w=0,phase=0,remaining=0,error=0,yaw_error=0; };
// A timing choice is a speed cap, not repeated multiplication of retained v.
// Preserve curvature when reducing translation; scale zero stops both axes.
inline SpatialCommand limitSpatialRisk(SpatialCommand command,double scale,double cruise) {
  if(scale<=0.) {command.v=command.w=0.;return command;}
  const double limited=std::min(command.v,cruise*std::clamp(scale,0.,1.));
  if(command.v>.01)command.w*=limited/command.v;
  command.v=limited;return command;
}
// Static differential-drive tracking. Progress is geometric: accepting a new
// reference never resets the retained command velocity or restarts a time ramp.
class SpatialTracker {
 public:
  std::vector<SpatialSample> samples;
  size_t index=0;
  double v=0,w=0;
  Eigen::Vector2d previous_position=Eigen::Vector2d::Zero();
  bool have_position=false,aligning=false;
  void setPath(std::vector<SpatialSample> path) {samples=std::move(path);index=0;have_position=false;aligning=false;}
  SpatialCommand step(const Eigen::Vector2d &position,double yaw,double dt,
      double max_v,double max_w,double acceleration,double finish,double lookahead_scale=1.) {
    SpatialCommand out;
    if(samples.empty()) {v=w=0;return out;}
    const double movement=have_position?(position-previous_position).norm():
        (position-samples.front().p.head<2>()).norm();
    const double search_end=samples[index].arc+std::max(.08,2.*movement+.08);
    previous_position=position;have_position=true;
    double best=(samples[index].p.head<2>()-position).squaredNorm();
    size_t nearest=index;
    for(size_t i=index+1;i<samples.size() && samples[i].arc<=search_end;++i) {
      double d=(samples[i].p.head<2>()-position).squaredNorm();
      if(d<best) {best=d;nearest=i;}
    }
    index=nearest;
    const double remaining=samples.back().arc-samples[index].arc;
    const double goal_distance=(samples.back().p.head<2>()-position).norm();
    const double lookahead=lookahead_scale*(.12+.20*std::abs(v));
    size_t carrot=index;
    while(carrot+1<samples.size() && samples[carrot].arc<samples[index].arc+lookahead) ++carrot;
    const Eigen::Vector2d target=samples[carrot].p.head<2>()-position;
    const double bearing=target.norm()>1e-6?std::atan2(target.y(),target.x()):yaw;
    const double error=std::atan2(std::sin(bearing-yaw),std::cos(bearing-yaw));
    double desired=max_v;
    for(size_t i=index;i<samples.size() && samples[i].arc<=samples[index].arc+.8;++i) {
      const double k=std::abs(samples[i].curvature);
      const double bend=std::min(max_v,std::min(max_w/std::max(k,1e-6),
                        std::min(std::sqrt(acceleration/std::max(k,1e-6)),
                          std::sqrt(2./std::max(std::abs(samples[i].curvature_rate),1e-6)))));
      desired=std::min(desired,std::sqrt(bend*bend+2*acceleration*
                          std::max(0.,samples[i].arc-samples[index].arc)));
    }
    desired=std::min(desired,std::sqrt(2*acceleration*std::max(0.,remaining-finish*.5)));
    desired*=std::max(0.,std::cos(error));
    if(std::abs(error)>1.05)aligning=true;
    if(std::abs(error)<.45)aligning=false;
    if(aligning)desired=0.;
    if(goal_distance<finish && remaining<finish+.1) desired=0.;
    dt=std::clamp(dt,0.,.05);
    v+=std::clamp(desired-v,-acceleration*dt,acceleration*dt);
    const double pursuit=2*std::max(v,.12)*std::sin(error)/std::max(target.norm(),.10);
    const double desired_w=std::clamp(std::abs(error)>1.?1.5*error:pursuit,-max_w,max_w);
    w+=std::clamp(desired_w-w,-2.*dt,2.*dt);
    out.v=v;out.w=w;out.phase=(goal_distance<finish && v<.02)?samples.back().t:samples[index].t;out.remaining=remaining;
    out.error=std::sqrt(best);out.yaw_error=error;
    return out;
  }
};
}
