#pragma once
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <set>
#include <cstdint>
#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <map>
#include <string>
#include <vector>

namespace scan_planner {
// For the double integrator with an axis-aligned acceleration box, the
// projected Zonotope is exactly a rectangle. No sampled-boundary enclosure.
struct ReachBox {
  Eigen::Vector2d center, half;
  double clearance(const Eigen::Vector2d &point, double radius) const {
    return ((point-center).cwiseAbs()-half).cwiseMax(0.0).norm()-radius;
  }
};
class IntentFrs {
 public:
  Eigen::Vector2d position{0,0}, velocity{0,0}, center{0,0}, half{0.2,0.2};
  double stamp{-1};
  bool windowed_acceleration{false};
  double collision_radius{0};
  Eigen::Vector2d position_half{0,0}, velocity_half{0,0};
  bool update(double time, const Eigen::Vector2d &p, const Eigen::Vector2d &v) {
    if (!std::isfinite(time)||!p.allFinite()||!v.allFinite()||time<=stamp) return false;
    if (stamp>=0 && time-stamp<=0.5) {
      const double dt=time-stamp;
      // Least-squares inverse B, including both position and velocity residuals.
      const double b=0.5*dt*dt;
      if(windowed_acceleration) {
        velocity_history_.push_back({time,v});
        while(velocity_history_.size()>1 && time-velocity_history_[1].first>=0.35)
          velocity_history_.pop_front();
        const double span=time-velocity_history_.front().first;
        if(span>=0.25) history_.push_back((v-velocity_history_.front().second)/span);
      } else history_.push_back((b*(p-position-dt*velocity)+dt*(v-velocity))/(b*b+dt*dt));
      if(history_.size()>10) history_.pop_front();
    } else { history_.clear(); velocity_history_.clear(); velocity_history_.push_back({time,v}); }
    position=p; velocity=v; stamp=time;
    center.setZero(); half.setConstant(0.2);
    if(history_.size()>=2) {
      Eigen::Vector2d lo=history_.front(),hi=lo;
      for(const auto &a:history_) { lo=lo.cwiseMin(a); hi=hi.cwiseMax(a); }
      center=(lo+hi)/2; half=(hi-lo)/2+Eigen::Vector2d::Constant(0.05);
    }
    return true;
  }
  ReachBox predict(double horizon, const std::string &mode) const {
    const bool learned=mode=="frs";
    Eigen::Vector2d a=Eigen::Vector2d::Zero(), width=Eigen::Vector2d::Zero();
    if(learned) { a=center; width=half; }
    else if(mode=="physical") width.setConstant(2.0);
    return {position+velocity*horizon+0.5*horizon*horizon*a,
            position_half+horizon*velocity_half+0.5*horizon*horizon*width};
  }
 private:
  std::deque<Eigen::Vector2d> history_;
  std::deque<std::pair<double,Eigen::Vector2d>> velocity_history_;
};

struct TimingDecision {
  double scale{0}, nominal_clearance{0}, chosen_clearance{0};
  bool feasible{false};
  std::string nominal_obstacle;
};
struct TrackView { const IntentFrs *track; double age; std::string id; };
class FrsTracks {
 public:
  std::map<std::string,IntentFrs> tracks;
  bool update(const std::string &id,double time,const Eigen::Vector2d &p,const Eigen::Vector2d &v) {
    if(id.rfind("pedestrian_",0)!=0 || id.size()<=11 ||
       id.find_first_not_of("0123456789",11)!=std::string::npos ||
       !std::isfinite(time)||time<0||!p.allFinite()||!v.allFinite()) return false;
    return tracks[id].update(time,p,v);
  }
  bool fresh(double now,size_t minimum=1) const {
    if(tracks.size()<minimum) return false;
    for(const auto &[id,t]:tracks)
      if(t.stamp<0 || now-t.stamp>0.5 || now<t.stamp) return false;
    return true;
  }
  std::vector<TrackView> views(double now) const {
    std::vector<TrackView> result;
    for(const auto &[id,t]:tracks) result.push_back({&t,now-t.stamp,id});
    return result;
  }
};
// A complete, health-qualified snapshot has explicit lifecycle and empty-scene semantics.
struct ObstacleSample {
  std::string id;
  Eigen::Vector2d position, velocity;
  Eigen::Matrix4d covariance;
  double radius, last_observed;
  bool observed;
};
class SnapshotTracks {
 public:
  FrsTracks bank;
  bool healthy{false};
  double stamp{-1};
  uint64_t epoch{0};
  bool accept(double time, uint64_t generation, bool health,
              const std::vector<ObstacleSample>& samples, double now,
              double robot_radius=0.43, double margin=0.10) {
    // Invalid packets revoke health immediately, without partially mutating tracks.
    healthy=false;
    if(!health || !std::isfinite(time) || time<0 || time>now+0.05 || now-time>0.5 ||
       (generation==epoch && time<=stamp)) return false;
    std::set<std::string> ids;
    for(const auto& s:samples) {
      if(s.id.empty() || !ids.insert(s.id).second || !s.position.allFinite() ||
         !s.velocity.allFinite() || !std::isfinite(s.radius) || s.radius<=0 ||
         !std::isfinite(s.last_observed) || s.last_observed>time+1e-6 ||
         time-s.last_observed>1.0 || !s.covariance.allFinite() ||
         !s.covariance.isApprox(s.covariance.transpose(),1e-6)) return false;
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> es(s.covariance);
      if(es.info()!=Eigen::Success || es.eigenvalues().minCoeff() < -1e-9) return false;
    }
    auto next = generation==epoch ? bank.tracks : std::map<std::string,IntentFrs>{};
    for(auto it=next.begin();it!=next.end();) {
      if(!ids.count(it->first)) it=next.erase(it); else ++it;
    }
    for(const auto& s:samples) {
      auto found=next.find(s.id);
      // A predicted-only state cannot initialize a new acceleration history.
      if(!s.observed && found==next.end()) return false;
      auto& t=next[s.id];
      t.windowed_acceleration=true;
      if(s.observed && !t.update(time,s.position,s.velocity)) return false;
      t.collision_radius=robot_radius+margin+s.radius;
      // 3-sigma engineering margins, not deterministic coverage guarantees.
      t.position_half=3*s.covariance.diagonal().head<2>().cwiseMax(0.).cwiseSqrt();
      t.velocity_half=3*s.covariance.diagonal().tail<2>().cwiseMax(0.).cwiseSqrt();
    }
    bank.tracks=std::move(next); stamp=time; epoch=generation; healthy=true;
    return true;
  }
  bool fresh(double now) const {return healthy && now>=stamp && now-stamp<=0.5;}
};
// Identical candidate set and collision rule across CV/FRS/physical modes.
// Select the fastest safe progression along the existing geometric spline.
template<class Path>
TimingDecision chooseTiming(const std::vector<TrackView> &tracks, const std::string &mode,
                            Path point, double radius=0.83, double horizon=3.0) {
  TimingDecision result;
  if(tracks.empty()) return result;
  for(double scale: {1.0,0.5,0.0}) {
    double clearance=std::numeric_limits<double>::infinity();
    std::string limiting;
    const int steps=std::max(1,static_cast<int>(std::ceil(horizon/0.1)));
    for(int k=0;k<=steps;++k) {
      double t=horizon*k/steps;
      const Eigen::Vector2d p=point(scale,t);
      for(const auto &view:tracks) {
        const double gap=view.track->predict(view.age+t,mode).clearance(p,view.track->collision_radius>0 ? view.track->collision_radius : radius);
        if(gap<clearance) {clearance=gap;limiting=view.id;}
      }
    }
    if(scale==1) {result.nominal_clearance=clearance;result.nominal_obstacle=limiting;}
    if(clearance>=0) {
      result.scale=scale; result.chosen_clearance=clearance; result.feasible=true; return result;
    }
    result.chosen_clearance=clearance;
  }
  return result; // No safe candidate: stop and report infeasibility explicitly.
}
// Bounded engineering risk score, NOT a collision probability. A large uncertain
// FRS stays a preference, while the measured-body/CV envelope remains mandatory.
inline double frsRisk(const IntentFrs& track, double age, double ahead,
                      const Eigen::Vector2d& point, const std::string& mode) {
  const double gap=track.predict(age+ahead,mode).clearance(point,track.collision_radius);
  return std::exp(-ahead/1.5) * std::exp(-std::max(0.0,gap)/0.35);
}
template<class Path>
TimingDecision chooseSoftTiming(const std::vector<TrackView>& tracks,const std::string& mode,
                               Path point,double horizon,double hard_horizon,double weight) {
  TimingDecision result;
  if(tracks.empty()) {result.scale=1.;result.feasible=true;return result;}
  double best=std::numeric_limits<double>::infinity();
  for(double scale: {1.0,0.75,0.5,0.25,0.0}) {
    double gap=std::numeric_limits<double>::infinity(), risk=0;
    std::string limiting;
    for(int k=0;k<=30;++k) {
      const double t=horizon*k/30.;
      double at_time=0;
      for(const auto& view:tracks) {
        at_time=std::max(at_time,frsRisk(*view.track,view.age,t,point(scale,t),mode));
        const double ht=hard_horizon*k/30.;
        const double g=view.track->predict(view.age+ht,"cv").clearance(point(scale,ht),view.track->collision_radius);
        if(g<gap) {gap=g;limiting=view.id;}
      }
      risk+=at_time/31.;
    }
    if(scale==1.) {result.nominal_clearance=gap;result.nominal_obstacle=limiting;}
    const double cost=(1-scale)*(1-scale)+weight*risk;
    if(gap>=0 && cost<best) {
      best=cost;result.scale=scale;result.feasible=true;result.chosen_clearance=gap;
    }
  }
  return result;
}
template<class Path>
TimingDecision chooseTiming(const IntentFrs &track,double age,const std::string &mode,
                            Path point,double radius=0.83) {
  return chooseTiming(std::vector<TrackView>{{&track,age,"pedestrian_0"}},mode,point,radius);
}
} // namespace scan_planner
