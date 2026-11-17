#pragma once
#include "plan_manage/intent_frs.hpp"
#include <tuple>

namespace scan_planner {
struct JointRolloutConfig {
  double speed{.5},yaw_rate{1.},acceleration{.5},braking{.5},yaw_acceleration{2.};
  double step{.4},horizon{3.2},risk_weight{2.};
  int beam_width{36};
};
struct JointControl {double v{0},w{0};};
struct JointRolloutResult {
  bool feasible{false},parked_threat{false};
  double score{0},clearance{0},risk{0};int candidates{0};
  std::vector<JointControl> controls;
  std::vector<Eigen::Vector3d> path;
};
inline double jointAngle(double a){return std::atan2(std::sin(a),std::cos(a));}
// Complete input sequence, tagged with the same trajectory id as its geometric
// reference. The planner owns selection; the controller only executes a prefix.
struct JointRolloutPacket {
  double stamp{-1},step{.4};std::int64_t id{-1};bool active{false},feasible{false};
  std::vector<JointControl> controls;
  std::vector<double> encode()const {
    std::vector<double> a{1.,double(id),stamp,active?1.:0.,feasible?1.:0.,step,double(controls.size())};
    for(const auto& u:controls){a.push_back(u.v);a.push_back(u.w);}return a;
  }
  static bool decode(const std::vector<double>& a,double now,double vmax,double wmax,JointRolloutPacket& p){
    if(a.size()<7 || !std::all_of(a.begin(),a.end(),[](double x){return std::isfinite(x);}) ||
       a[0]!=1. || a[1]<0 || a[1]>1e12 || std::abs(a[1]-std::round(a[1]))>1e-6 ||
       a[2]>now+.05 || now-a[2]>.8 || (a[3]!=0. && a[3]!=1.) || (a[4]!=0. && a[4]!=1.) ||
       a[5]<.1 || a[5]>.5 || a[6]<0 || a[6]>20 || a[6]!=std::floor(a[6]) || a.size()!=7+2*size_t(a[6]))return false;
    p={};p.id=std::llround(a[1]);p.stamp=a[2];p.active=a[3];p.feasible=a[4];p.step=a[5];
    for(size_t i=7;i<a.size();i+=2){if(a[i]<0 || a[i]>vmax+1e-6 || std::abs(a[i+1])>wmax+1e-6)return false;p.controls.push_back({a[i],a[i+1]});}
    return !p.active || !p.feasible || !p.controls.empty();
  }
};
// FRS remains a soft risk. Hard rejection uses fixed UNKNOWN, near CV error
// envelopes, and far nominal body interception with current position error.
inline double jointHardGap(const TrackView& view,double t,const Eigen::Vector2d& p,double near){
  const auto cv=view.track->predict(std::max(0.,view.age)+t,"cv");
  return (view.track->unknown || t<=near+1e-9 ? cv : ReachBox{cv.center,view.track->position_half}).clearance(p,view.track->collision_radius);
}
class JointRolloutPlanner {
 public:
  JointRolloutConfig config;
  template<class Free>
  JointRolloutResult choose(const Eigen::Vector3d& pose,double yaw,double measured_v,double measured_w,
      const Eigen::Vector3d& goal,const std::vector<Eigen::Vector3d>& guide,const std::vector<TrackView>& tracks,Free free,double time=std::numeric_limits<double>::quiet_NaN()){
    JointRolloutResult result;
    if(!pose.allFinite() || !goal.allFinite() || !std::isfinite(yaw) || !std::isfinite(measured_v) || !std::isfinite(measured_w) ||
       !std::isfinite(config.speed+config.yaw_rate+config.acceleration+config.braking+config.horizon+config.step) ||
       config.speed<=0 || config.yaw_rate<=0 || config.acceleration<=0 || config.braking<=0 ||
       config.step<.1 || config.step>.5 || std::abs(config.step/.05-std::round(config.step/.05))>1e-8 ||
       config.horizon<1. || config.horizon>4. || config.beam_width<1 || config.beam_width>128)return result;
    const double h=.05;const int stages=int(std::ceil(config.horizon/config.step)),micro=int(std::round(config.step/h));
    if(!std::isfinite(time))time=previous_stamp_<0?0.:previous_stamp_+config.step;
    std::vector<JointControl> warm;
    if(previous_stamp_>=0 && time>=previous_stamp_ && time-previous_stamp_<.8 && !previous_controls_.empty()){
      const size_t shift=size_t(std::floor((time-previous_stamp_+.05)/config.step+1e-6));
      for(int i=0;i<stages;++i)warm.push_back(previous_controls_[std::min(shift+size_t(i),previous_controls_.size()-1)]);
    }
    struct Node {Eigen::Vector3d p;double yaw,v,w,cost{0},risk{0},gap{100};std::vector<JointControl> us;std::vector<Eigen::Vector3d> path;};
    Node root{};root.p=pose;root.yaw=yaw;root.v=std::clamp(measured_v,0.,config.speed);root.w=std::clamp(measured_w,-config.yaw_rate,config.yaw_rate);root.path.push_back(pose);
    const double near=.5+config.speed/config.braking;
    for(double t=0;t<=config.horizon;t+=h)for(const auto& tr:tracks)
      if(jointHardGap(tr,t,pose.template head<2>(),near)<0)result.parked_threat=true;
    std::vector<Node> beam{root};int count=0;
    for(int stage=0;stage<stages;++stage){
      std::vector<Node> next;
      for(const auto& base:beam)for(double sv:{1.,.6,.3,0.})for(double sw:{-1.,-.65,-.3,0.,.3,.65,1.}){
        ++count;Node n=base;const JointControl u{config.speed*sv,config.yaw_rate*sw};n.us.push_back(u);bool safe=true;
        if(stage==0 && !warm.empty())n.cost+=.4*(u.w-warm[0].w)*(u.w-warm[0].w)+.4*(u.v-warm[0].v)*(u.v-warm[0].v);
        for(int k=0;k<micro;++k){
          const double t=(stage*micro+k)*h;
          if(!free(n.p,n.yaw)){safe=false;break;}
          double risk=0.,clearance_cost=0.;
          for(const auto& tr:tracks){
            const double g=jointHardGap(tr,t,n.p.template head<2>(),near);n.gap=std::min(n.gap,g);
            if(g<0){safe=false;break;}
            if(!tr.track->unknown){
              risk=std::max(risk,frsRisk(*tr.track,std::max(0.,tr.age),t,n.p.template head<2>(),"frs"));
              clearance_cost=std::max(clearance_cost,std::exp(-t/3.)*std::exp(-g/.5));
            }
          }
          if(!safe)break;
          const double before=(goal-n.p).template head<2>().norm();
          double distance=100.;for(const auto& q:guide)distance=std::min(distance,(q-n.p).template head<2>().squaredNorm());
          n.risk+=h*risk/config.horizon;
          n.cost+=h*(config.risk_weight*risk+1.5*clearance_cost+.18*distance+.06*n.w*n.w+.03*(u.w-n.w)*(u.w-n.w));
          // A 0.05 s reaction interval is shared with prefix execution.
          if(t>=.05){
            const double approach=std::sqrt(2*config.braking*std::max(0.,before-.06));
            n.v+=std::clamp(std::min(u.v,approach)-n.v,-config.braking*h,config.acceleration*h);
            n.w+=std::clamp(u.w-n.w,-config.yaw_acceleration*h,config.yaw_acceleration*h);
          }
          n.p.x()+=n.v*h*std::cos(n.yaw+.5*n.w*h);n.p.y()+=n.v*h*std::sin(n.yaw+.5*n.w*h);n.yaw=jointAngle(n.yaw+n.w*h);
          n.cost-=1.5*(before-(goal-n.p).template head<2>().norm());n.path.push_back(n.p);
        }
        if(safe)next.push_back(std::move(n));
      }
      if(next.empty()){result.candidates=count;return result;}
      std::sort(next.begin(),next.end(),[](const Node& a,const Node& b){return a.cost<b.cost;});
      // Keep geometrically distinct headings/positions, not many copies of the
      // same straight rollout. Otherwise early goal reward prunes all escapes.
      beam.clear();std::set<std::tuple<int,int,int,int>> cells;std::map<int,int> heading_counts;
      std::vector<Node> seeds;
      for(auto& n:next){
        const auto first=n.us.front();
        bool continuation=!warm.empty();for(size_t i=0;i<n.us.size() && continuation;++i)continuation=n.us[i].v==warm[i].v && n.us[i].w==warm[i].w;
        if(continuation || std::all_of(n.us.begin(),n.us.end(),[&](const JointControl& u){return u.v==first.v && u.w==first.w;})){
          seeds.push_back(std::move(n));continue;
        }
        const int heading=int(std::round(n.w/.2));
        auto key=std::make_tuple(int(std::round((n.p.x()-pose.x())/.15)),int(std::round((n.p.y()-pose.y())/.15)),int(std::round(jointAngle(n.yaw-yaw)/.15)),heading);
        if(heading_counts[heading]<3 && cells.insert(key).second){++heading_counts[heading];beam.push_back(std::move(n));}
        if(int(beam.size())>=config.beam_width)break;
      }
      // Constant-control seeds preserve decisive early turns even before their
      // positions separate. A goal-greedy beam alone can prune every escape.
      for(auto& n:next){
        if(n.us.empty())continue;
        const auto first=n.us.front();
        bool continuation=!warm.empty();for(size_t i=0;i<n.us.size() && continuation;++i)continuation=n.us[i].v==warm[i].v && n.us[i].w==warm[i].w;
        if(continuation || std::all_of(n.us.begin(),n.us.end(),[&](const JointControl& u){return u.v==first.v && u.w==first.w;}))seeds.push_back(std::move(n));
      }
      for(auto& n:seeds)beam.push_back(std::move(n));
      std::sort(beam.begin(),beam.end(),[](const Node& a,const Node& b){return a.cost<b.cost;});
    }
    // Validate final sample and static/UNKNOWN braking tails. The whole FRS
    // envelope is never promoted to a hard forbidden region.
    for(auto& n:beam){
      bool safe=true;for(const auto& tr:tracks)if(jointHardGap(tr,config.horizon,n.p.template head<2>(),near)<0)safe=false;
      Eigen::Vector3d p=n.p;double a=n.yaw,v=n.v,w=n.w;
      for(double t=0;t<=.15+n.v/config.braking;t+=.025){
        if(!free(p,a)){safe=false;break;}
        for(const auto& tr:tracks)if(tr.track->unknown && jointHardGap(tr,0.,p.template head<2>(),near)<0)safe=false;
        if(t>=.1)v=std::max(0.,v-config.braking*.025);
        w+=std::clamp(-w,-.05,.05);
        p.x()+=v*.025*std::cos(a+.0125*w);p.y()+=v*.025*std::sin(a+.0125*w);a+=w*.025;
      }
      if(!safe)continue;
      result.feasible=true;result.controls=std::move(n.us);result.path=std::move(n.path);result.score=n.cost;result.risk=n.risk;result.clearance=n.gap;break;
    }
    if(result.feasible){previous_controls_=result.controls;previous_stamp_=time;}
    result.candidates=count;return result;
  }
 private:
  std::vector<JointControl> previous_controls_;double previous_stamp_{-1};
};
}
