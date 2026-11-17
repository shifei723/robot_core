#pragma once
#include "plan_manage/intent_frs.hpp"
#include "plan_manage/spatial_tracker.hpp"
#include "plan_manage/wheel_timing.hpp"

namespace scan_planner {
inline bool dynamicLocalRefreshDue(bool enabled,double speed,bool fresh,bool have_tracks,
    double now,double last,double interval) {
  return enabled && (speed<=0. || !fresh || have_tracks) && now-last>=interval;
}
struct DynamicPathConfig {
  double speed{.5},yaw_rate{1.},acceleration{.5},braking{.5},horizon{3.};
  double risk_weight{2.},deviation_weight{1.5},turn_weight{.15};
};
struct DynamicPathResult {
  bool feasible{false};
  double speed{0},offset{0},score{0},clearance{0},risk{0};
  int candidates{0};
  UniformBspline spline;
  std::vector<Eigen::Vector3d> path;
};
inline std::vector<SpatialSample> dynamicSpatialSamples(UniformBspline curve) {
  std::vector<SpatialSample> out;double arc=0.;const double duration=curve.getTimeSum();
  for(double t=0;t<duration+.04;t+=.04) {
    const double time=std::min(t,duration);const Eigen::Vector3d p=curve.evaluateDeBoorT(time);
    if(!out.empty())arc+=(p-out.back().p).head<2>().norm();
    out.push_back({p,time,arc,0.,0.});
  }
  for(size_t i=1;i+1<out.size();++i) {
    size_t a=i,b=i;while(a>0 && out[i].arc-out[a].arc<.06)--a;
    while(b+1<out.size() && out[b].arc-out[i].arc<.06)++b;
    const Eigen::Vector2d u=(out[i].p-out[a].p).head<2>(),v=(out[b].p-out[i].p).head<2>();
    const double length=.5*(u.norm()+v.norm());
    if(u.norm()>1e-5 && v.norm()>1e-5)out[i].curvature=std::atan2(u.x()*v.y()-u.y()*v.x(),u.dot(v))/length;
  }
  for(size_t i=0;i<out.size();++i) {
    size_t a=i,b=i;while(a>0 && out[i].arc-out[a].arc<.10)--a;
    while(b+1<out.size() && out[b].arc-out[i].arc<.10)++b;
    const double span=out[b].arc-out[a].arc;
    if(span>.02)out[i].curvature_rate=(out[b].curvature-out[a].curvature)/span;
  }
  return out;
}
// Offset is a smooth, compact bump. Every candidate rejoins the original path;
// no unbounded constant-heading escape or lateral robot velocity is generated.
inline std::vector<Eigen::Vector3d> boundedDynamicDetour(const std::vector<Eigen::Vector3d>& path,double offset) {
  if(path.size()<2)return path;
  std::vector<double> arc{0.};
  for(size_t i=1;i<path.size();++i)arc.push_back(arc.back()+(path[i]-path[i-1]).head<2>().norm());
  if(arc.back()<1e-6)return path;
  const double length=std::min(3.,arc.back());
  auto at=[&](double s)->Eigen::Vector3d {
    s=std::clamp(s,0.,arc.back());size_t i=1;
    while(i+1<arc.size() && arc[i]<s)++i;
    const double u=(s-arc[i-1])/std::max(1e-9,arc[i]-arc[i-1]);return path[i-1]+u*(path[i]-path[i-1]);
  };
  std::vector<Eigen::Vector3d> out;
  const int n=std::max(1,int(std::ceil(arc.back()/.075)));
  for(int k=0;k<=n;++k) {
    const double s=arc.back()*k/n;Eigen::Vector3d p=at(s);
    const Eigen::Vector2d tangent=(at(s+.25)-at(s-.25)).head<2>();
    if(s<length && tangent.norm()>1e-8) {
      const double u=std::sin(3.141592653589793*s/length);
      p.head<2>()+=offset*u*u*Eigen::Vector2d(-tangent.y(),tangent.x())/tangent.norm();
    }
    out.push_back(p);
  }
  out.front()=path.front();out.back()=path.back();return out;
}
class DynamicLocalPath {
 public:
  DynamicPathConfig config;
  // Shared follower rollouts evaluate the curve that will actually be published.
  // Far uncertain FRS is a cost; static/UNKNOWN and near CV envelopes remain hard.
  template<class Free>
  DynamicPathResult choose(const std::vector<Eigen::Vector3d>& reference,
      const Eigen::Vector3d& pose,double yaw,double measured_v,double measured_w,
      const std::vector<TrackView>& tracks,const std::vector<Eigen::Vector3d>& guide,Free free) {
    DynamicPathResult best;best.score=std::numeric_limits<double>::infinity();
    if(reference.size()<2 || !pose.allFinite() || !std::isfinite(yaw) ||
       !std::isfinite(measured_v) || !std::isfinite(measured_w))return best;
    bool planar=true;for(const auto& p:reference)if(!p.allFinite() || std::abs(p.z()-pose.z())>.005)planar=false;
    if(!planar)return best;
    const double length=(reference.back()-reference.front()).head<2>().norm();
    const std::vector<double> offsets=length>1.5 ? std::vector<double>{0.,-.25,.25,-.5,.5,-.9,.9} : std::vector<double>{0.};
    int count=0;
    for(double offset:offsets) {
      const auto path=boundedDynamicDetour(reference,offset);
      UniformBspline curve;
      if(!makeLocallyTimedWheelSpline(path,config.speed,config.acceleration,.06,curve))continue;
      auto samples=dynamicSpatialSamples(curve);bool geometry_safe=true;
      for(size_t i=0;i<samples.size();++i) {
        const auto& p=samples[i].p;
        const Eigen::Vector2d d=(samples[std::min(i+1,samples.size()-1)].p-samples[i?i-1:0].p).head<2>();
        const double heading=d.norm()>1e-8?std::atan2(d.y(),d.x()):yaw;
        if(!free(p,heading)){geometry_safe=false;break;}
        for(const auto& view:tracks)if(view.track->unknown && view.track->predict(0.,"cv").clearance(p.head<2>(),view.track->collision_radius)<0.)geometry_safe=false;
        if(!geometry_safe)break;
      }
      if(!geometry_safe)continue;
      for(double scale:{1.,.75,.5,0.}) {
        if(scale==0. && offset!=0.)continue;
        ++count;const double speed=config.speed*scale;
        SpatialTracker follower;follower.setPath(samples);follower.v=std::max(0.,measured_v);follower.w=measured_w;
        Eigen::Vector3d p=pose;double heading=yaw,pv=std::max(0.,measured_v),pw=measured_w;
        double gap=std::numeric_limits<double>::infinity(),risk=0.,deviation=0.,turn=0.;bool safe=true;
        const int steps=int(std::ceil(config.horizon/.05));const double h=config.horizon/steps;
        for(int k=0;k<=steps;++k) {
          const double t=k*h;
          if(!free(p,heading)){safe=false;break;}
          double at_risk=0.;
          for(const auto& view:tracks) {
            const auto cv=view.track->predict(std::max(0.,view.age)+t,"cv");
            const double near=.5+std::max(std::abs(measured_v),speed)/config.braking;
            double g;
            if(view.track->unknown || t<=near+1e-9)g=cv.clearance(p.head<2>(),view.track->collision_radius);
            else g=ReachBox{cv.center,view.track->position_half}.clearance(p.head<2>(),view.track->collision_radius);
            gap=std::min(gap,g);if(g<0.){safe=false;break;}
            if(!view.track->unknown)at_risk=std::max(at_risk,frsRisk(*view.track,std::max(0.,view.age),t,p.head<2>(),"frs"));
          }
          if(!safe)break;
          risk+=at_risk/(steps+1);turn+=pw*pw/(steps+1);
          double distance=std::numeric_limits<double>::infinity();
          for(const auto& q:guide)distance=std::min(distance,(q-p).head<2>().squaredNorm());
          if(std::isfinite(distance))deviation+=distance/(steps+1);
          if(k==steps)break;
          if(t>=.10) {
            auto cmd=follower.step(p.head<2>(),heading,h,speed,config.yaw_rate,config.acceleration,.1);
            if(speed==0.)cmd.v=cmd.w=0.;
            pv+=std::clamp(cmd.v-pv,-config.braking*h,config.acceleration*h);
            pw+=std::clamp(cmd.w-pw,-2.*h,2.*h);follower.v=pv;follower.w=pw;
          }
          p.x()+=pv*h*std::cos(heading+.5*pw*h);p.y()+=pv*h*std::sin(heading+.5*pw*h);heading+=pw*h;
        }
        if(!safe)continue;
        // Also certify a static/UNKNOWN braking tail from the predicted state.
        Eigen::Vector3d stop=p;double angle=heading,sv=pv,sw=pw;
        for(double t=0;t<.12+pv/config.braking+.1;t+=.025) {
          if(!free(stop,angle)){safe=false;break;}
          for(const auto& view:tracks)if(view.track->unknown && view.track->predict(0.,"cv").clearance(stop.head<2>(),view.track->collision_radius)<0.)safe=false;
          if(!safe)break;
          if(t>=.12)sv=std::max(0.,sv-config.braking*.025);
          sw+=std::clamp(-sw,-.05,.05);
          stop.x()+=sv*.025*std::cos(angle+.0125*sw);stop.y()+=sv*.025*std::sin(angle+.0125*sw);angle+=sw*.025;
        }
        if(!safe)continue;
        const double progress=(reference.back()-pose).head<2>().norm()-(reference.back()-p).head<2>().norm();
        const double score=config.risk_weight*risk+1.5*(1.-scale)*(1.-scale)+
          config.deviation_weight*deviation+config.turn_weight*turn+.15*offset*offset-.1*progress;
        if(score<best.score) {
          best.feasible=true;best.speed=speed;best.offset=offset;best.score=score;
          best.clearance=std::isfinite(gap)?gap:100.;best.risk=risk;best.path=path;best.spline=curve;
        }
      }
    }
    best.candidates=count;return best;
  }
};
}
