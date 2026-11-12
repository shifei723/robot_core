#include <plan_manage/spatial_tracker.hpp>
#include <plan_manage/static_motion_safety.hpp>
#include <plan_manage/wheel_shortcut.hpp>
#include <cassert>
#include <iostream>
using namespace scan_planner;
int main(){
 auto far=[](const Eigen::Vector3d&p,double){return p.x()<1.;};
 auto near=[](const Eigen::Vector3d&p,double){return p.x()<.1;};
 assert(brakingMotionFree(Eigen::Vector3d::Zero(),0.,.5,0.,.5,0.,.5,far));
 assert(!brakingMotionFree(Eigen::Vector3d::Zero(),0.,.5,0.,.5,0.,.5,near));
 assert(!brakingMotionFree(Eigen::Vector3d::Zero(),0.,0.,0.,.4,0.,.5,near));

 assert(std::abs(spatialCollisionArrival(.25,0.,.5)-1.)<1e-9);
 assert(spatialCollisionArrival(.5,.5,.5)<1.);
 assert(spatialCollisionArrival(.5,.5,0.)==1.);
 assert(spatialCollisionArrival(0.,.5,.5)==0.);
 SpatialTracker tracker;std::vector<SpatialSample> path;
 for(int i=0;i<=500;++i) path.push_back({{i*.01,0,.25},i*.02,i*.01,0.});
 tracker.setPath(path);double x=0.,last=0.;
 for(int i=0;i<600;++i){auto cmd=tracker.step({x,0},0,.01,.5,1,.5,.15);
 assert(cmd.v>=0 && cmd.v<=.5);assert(std::abs(cmd.v-last)<=.005001);assert(std::abs(cmd.w)<=1.);
 x+=cmd.v*.01;last=cmd.v;
 if(i==300){auto v=tracker.v;tracker.setPath(path);assert(tracker.v==v);}}
 assert(x>2.5);assert(tracker.v>.49);
 for(int i=0;i<1000;++i){auto cmd=tracker.step({x,0},0,.01,.5,1,.5,.15);x+=cmd.v*.01;
 assert(std::abs(cmd.v-last)<=.005001);last=cmd.v;}
 assert(std::abs(x-5.)<.15);assert(tracker.v<.02);
 auto stopped=tracker.step({x,0},0,.01,.5,1,.5,.15);assert(stopped.phase==path.back().t);

 std::vector<Eigen::Vector3d> bend{{0,0,0},{0,1,0},{1,1,0},{1,0,0}};
 auto free=[](const Eigen::Vector3d&p,double){return !(p.x()>.4 && p.x()<.6 && p.y()<.7);};
 auto shortened=shortcutWheelPath(bend,.01,free);
 assert(shortened.size()>=3); // direct shortcut would cut through the obstacle
 auto straight=shortcutWheelPath(bend,.01,[](const auto&,double){return true;});assert(straight.size()==2);
 SpatialTracker rate_limited;auto sharp_rate=path;
 for(auto &p:sharp_rate)p.curvature_rate=40.;rate_limited.setPath(sharp_rate);
 double sx=0.;for(int i=0;i<200;++i){auto cmd=rate_limited.step({sx,0},0,.01,.5,1,.5,.15);
 sx+=cmd.v*.01;assert(cmd.v<=std::sqrt(2./40.)+1e-8);}
 SpatialTracker turn;turn.setPath(path);turn.v=.2;
 double tx=0.,ty=0.,heading=1.5;
 for(int i=0;i<800;++i){auto cmd=turn.step({tx,ty},heading,.01,.5,1,.5,.15);
 tx+=cmd.v*.01*std::cos(heading);ty+=cmd.v*.01*std::sin(heading);heading+=cmd.w*.01;}
 assert(tx>1. && std::abs(ty)<.10); // Large heading error must not orbit the carrot.
 SpatialTracker curved;std::vector<SpatialSample> route;double arc=0.;
 auto add=[&](double x,double y,double k){Eigen::Vector3d p(x,y,.25);
 if(!route.empty())arc+=(p-route.back().p).head<2>().norm();route.push_back({p,arc/.5,arc,k});};
 for(int i=0;i<=100;++i)add(i*.01,0,0);
 for(int i=1;i<=100;++i){double theta=-M_PI/2+i*M_PI/200;add(1+.6*std::cos(theta),.6+.6*std::sin(theta),1/.6);}
 for(int i=1;i<=200;++i)add(1.6,.6+i*.01,0);
 curved.setPath(route);Eigen::Vector2d pose(0,0);double yaw=0,max_error=0;
 for(int i=0;i<2000;++i){auto cmd=curved.step(pose,yaw,.01,.5,1,.5,.15);
 pose+=Eigen::Vector2d(std::cos(yaw),std::sin(yaw))*cmd.v*.01;yaw+=cmd.w*.01;max_error=std::max(max_error,cmd.error);}
 assert((pose-route.back().p.head<2>()).norm()<.16);assert(max_error<.08);
 // Persistent quarter-speed risk must converge to a useful speed cap,
 // rather than repeatedly scaling retained velocity toward zero.
 double retained=0.;
 for(int i=0;i<200;++i) {
   SpatialCommand nominal;nominal.v=std::min(.5,retained+.005);nominal.w=2*nominal.v;
   auto limited=limitSpatialRisk(nominal,.25,.5);
   assert(limited.v<=.125+1e-12);assert(std::abs(limited.w-2*limited.v)<1e-12);
   retained=limited.v;
 }
 assert(std::abs(retained-.125)<1e-12);
 SpatialCommand moving;moving.v=.3;moving.w=.6;
 auto risk_stop=limitSpatialRisk(moving,0.,.5);assert(risk_stop.v==0 && risk_stop.w==0);
 std::cout<<"Curved tracking maximum error="<<max_error<<" m\n";
 std::cout<<"PASS: bounded commands, retained speed on replanning, obstacle-preserving shortcuts\n";
}
