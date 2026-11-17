#pragma once
#include "plan_manage/intent_frs.hpp"
#include <functional>
#include <stdexcept>

namespace scan_planner {
// Optional differential-drive recovery. No lateral velocity or obstacle truth.
// All commands remain subject to the controller's fresh-input and static gates.
struct EscapeConfig {
  bool enabled{false};
  double horizon{2.5}, max_speed{.5}, max_yaw_rate{1.}, acceleration{.5}, braking{.5};
  double trigger_margin{.05}, minimum_hold{.8};
};
struct EscapeResult {
  bool active{false}, triggered{false}, feasible{false};
  double v{0}, w{0}, parked_clearance{0}, chosen_clearance{0};
  int checked{0};
  std::string reason{"disabled"};
  std::vector<Eigen::Vector2d> path;
};
class ActiveEscape {
 public:
  EscapeConfig config;
  bool active() const {return active_;}
  void reset(){active_=false;safe_since_=-1;have_command_=false;}
  static double wrap(double x){return std::atan2(std::sin(x),std::cos(x));}
  // Candidate steering has bounded yaw acceleration and explicit forward-only motion.
  static std::pair<double,double> command(double yaw,double goal,double speed,
      double measured_v,double measured_w,double dt,const EscapeConfig& c) {
    const double target_w=std::clamp(2.*wrap(goal-yaw),-c.max_yaw_rate,c.max_yaw_rate);
    const double v=measured_v+std::clamp(speed-measured_v,-c.braking*dt,c.acceleration*dt);
    const double w=measured_w+std::clamp(target_w-measured_w,-2.*dt,2.*dt);
    return {std::clamp(v,0.,c.max_speed),std::clamp(w,-c.max_yaw_rate,c.max_yaw_rate)};
  }
  template<class Free>
  EscapeResult step(double now,double dt,const Eigen::Vector2d& position,double yaw,
      double measured_v,double measured_w,const Eigen::Vector2d& goal,
      const std::vector<TrackView>& tracks,const std::string& prediction,
      bool /*nominal_blocked*/,bool nominal_safe,Free free) {
    EscapeResult out;
    if(!config.enabled){reset();return out;}
    if(!position.allFinite() || !goal.allFinite() || !std::isfinite(now) ||
       !std::isfinite(yaw) || !std::isfinite(measured_v) || !std::isfinite(measured_w)) {
      reset();out.reason="invalid_input";return out;
    }
    if(tracks.empty()){reset();out.reason="clear";return out;}
    out.parked_clearance=std::numeric_limits<double>::infinity();
    bool moving_threat=false;
    for(int k=0;k<=50;++k){
      const double t=config.horizon*k/50.;
      for(const auto& view:tracks)
        out.parked_clearance=std::min(out.parked_clearance,
            view.track->predict(std::max(0.,view.age)+t,prediction).clearance(position,view.track->collision_radius));
      for(const auto& view:tracks)
        if(!view.track->unknown && view.track->velocity.norm()>.08 &&
           view.track->predict(std::max(0.,view.age)+t,prediction).clearance(position,view.track->collision_radius)<config.trigger_margin)
          moving_threat=true;
    }
    const bool danger=moving_threat && out.parked_clearance<config.trigger_margin;
    if(!active_ && !danger){out.reason="not_required";return out;}
    out.triggered=true;
    if(active_ && !danger && nominal_safe) {
      if(safe_since_<0)safe_since_=now;
      if(now-safe_since_>=.4 && now-started_>=config.minimum_hold){reset();out.reason="resume";return out;}
    } else safe_since_=-1;
    if(!free(position,yaw)){have_command_=false;out.reason="static_blocked";return out;}
    // Static/UNKNOWN checks cover the segment plus braking tail. Moving targets
    // retain the normal near-field hard horizon; far FRS remains a soft cost.
    // Recheck on every callback, not a global guarantee for unseen accelerations.
    struct Choice {double score,heading,speed,gap;std::vector<Eigen::Vector2d> path;};
    Choice best{std::numeric_limits<double>::infinity(),0.,0.,0.,{}};
    std::vector<double> headings;
    if(active_)headings.push_back(heading_);
    for(double a:{0.,-.65,.65,-1.15,1.15,-1.57,1.57})headings.push_back(yaw+a);
    for(double target:headings)for(double speed:{config.max_speed,.65*config.max_speed}) {
      ++out.checked;
      Eigen::Vector2d p=position;double angle=yaw,pv=measured_v,pw=measured_w;
      double min_gap=std::numeric_limits<double>::infinity(),risk=0.;bool safe=true;
      std::vector<Eigen::Vector2d> path{p};
      const double h=.05,tail=config.max_speed/config.braking+.15;
      const int steps=static_cast<int>(std::ceil((config.horizon+tail)/h));
      for(int k=0;k<=steps;++k) {
        const double t=k*h;
        const double hard_horizon=.5+std::max(std::abs(measured_v),speed)/config.braking;
        double gap=std::numeric_limits<double>::infinity();
        for(const auto& view:tracks) {
          const double g=view.track->predict(std::max(0.,view.age)+t,"cv").clearance(p,view.track->collision_radius);
          if(view.track->unknown || t<=hard_horizon+1e-9)gap=std::min(gap,g);
          else {
            // Beyond the mandatory uncertainty-aware near field, reject a
            // nominal physical interception too. Velocity uncertainty remains
            // in the far FRS cost rather than making all recovery infeasible.
            const auto cv=view.track->predict(std::max(0.,view.age)+t,"cv");
            gap=std::min(gap,ReachBox{cv.center,view.track->position_half}.clearance(p,view.track->collision_radius));
          }
          if(!view.track->unknown)risk+=h*frsRisk(*view.track,std::max(0.,view.age),t,p,prediction);
        }
        if(std::isfinite(gap))min_gap=std::min(min_gap,gap);
        if(gap<0. || !free(p,angle)){safe=false;break;}
        if(k==steps)break;
        if(t>=.1){auto u=command(angle,target,t<config.horizon?speed:0.,pv,pw,h,config);pv=u.first;pw=u.second;}
        p+=pv*h*Eigen::Vector2d(std::cos(angle+.5*pw*h),std::sin(angle+.5*pw*h));angle+=pw*h;path.push_back(p);
      }
      if(!safe)continue;
      const double switch_cost=active_?.65*std::abs(wrap(target-heading_)):0.;
      const double progress=(goal-position).norm()-(goal-p).norm();
      const double score=2.*risk-.7*std::min(min_gap,1.)-.3*progress+switch_cost;
      if(score<best.score)best={score,target,speed,min_gap,std::move(path)};
    }
    if(!std::isfinite(best.score)){have_command_=false;out.reason="no_safe_candidate";return out;}
    if(!active_)started_=now;
    active_=true;heading_=best.heading;
    // Integrate command limits at controller rate, not odometry message rate.
    // Reinitialise after rejection/gaps; rollouts still start from measurements.
    if(!have_command_ || now-last_command_time_>.1) {command_v_=measured_v;command_w_=measured_w;}
    const auto u=command(yaw,heading_,best.speed,command_v_,command_w_,std::clamp(dt,0.,.05),config);
    command_v_=u.first;command_w_=u.second;last_command_time_=now;have_command_=true;
    out.active=true;out.feasible=true;out.v=u.first;out.w=u.second;
    out.reason="escape";out.chosen_clearance=best.gap;out.path=std::move(best.path);return out;
  }
 private:
  bool have_command_{false};double command_v_{0},command_w_{0},last_command_time_{-1};
  bool active_{false};double heading_{0},started_{0},safe_since_{-1};
};
}
