#include <gtest/gtest.h>
#include "plan_manage/joint_rollout.hpp"
using namespace scan_planner;
static const std::vector<Eigen::Vector3d> guide{{0,0,.25},{.5,0,.25},{1,0,.25},{1.5,0,.25},{2,0,.25},{3,0,.25},{4,0,.25}};
TEST(JointRollout, EmptySceneSelectsProgressFromZero) {
 JointRolloutPlanner p;auto r=p.choose(guide.front(),0,0,0,guide.back(),guide,{},[](const auto&,double){return true;});
 ASSERT_TRUE(r.feasible);ASSERT_EQ(r.controls.size(),8u);EXPECT_GT(r.controls.front().v,0.);EXPECT_GT(r.path.back().x(),1.);
 EXPECT_LT(std::abs(r.path.back().y()),.1);
}
TEST(JointRollout, ParkedCollisionSelectsTurningMotion) {
 JointRolloutPlanner p;IntentFrs tr;tr.position={2.,0};tr.velocity={-.6,0};tr.collision_radius=.65;
 auto r=p.choose(guide.front(),0,0,0,guide.back(),guide,{{&tr,0.,"approach"}},[](const auto&,double){return true;});
 ASSERT_TRUE(r.feasible);EXPECT_TRUE(r.parked_threat);
 bool turn=false,move=false;for(const auto& u:r.controls){turn|=std::abs(u.w)>.1;move|=u.v>.1;EXPECT_LE(u.v,.5);EXPECT_LE(std::abs(u.w),1.);}
 EXPECT_TRUE(turn&&move);EXPECT_GT(std::abs(r.path.back().y()),.5);EXPECT_GE(r.clearance,0.);
}
TEST(JointRollout, UncertainFarFrsIsSoft) {
 JointRolloutPlanner p;IntentFrs tr;tr.position={3,2};tr.velocity={0,0};tr.half={8,8};tr.collision_radius=.65;
 auto r=p.choose(guide.front(),0,0,0,guide.back(),guide,{{&tr,0.,"wide"}},[](const auto&,double){return true;});
 ASSERT_TRUE(r.feasible);EXPECT_GT(r.path.back().x(),.5);
}
TEST(JointRollout, StaticAndUnknownCannotBeBypassed) {
 JointRolloutPlanner p;auto r=p.choose(guide.front(),0,0,0,guide.back(),guide,{},[](const auto&,double){return false;});EXPECT_FALSE(r.feasible);
 IntentFrs tr;tr.position={0,0};tr.unknown=true;tr.collision_radius=.65;
 r=p.choose(guide.front(),0,0,0,guide.back(),guide,{{&tr,0.,"unknown"}},[](const auto&,double){return true;});EXPECT_FALSE(r.feasible);
}
TEST(JointRollout, MalformedOrExpiredPacketsAreRejected) {
 JointRolloutPacket p;p.id=4;p.stamp=10;p.active=p.feasible=true;p.controls={{.5,.3},{.3,-.6}};auto a=p.encode();JointRolloutPacket q;
 EXPECT_TRUE(JointRolloutPacket::decode(a,10.1,.5,1.,q));EXPECT_EQ(q.id,4);EXPECT_EQ(q.controls.size(),2u);
 EXPECT_FALSE(JointRolloutPacket::decode(a,11.,.5,1.,q));a.back()=NAN;EXPECT_FALSE(JointRolloutPacket::decode(a,10.1,.5,1.,q));
 a=p.encode();a[7]=-.1;EXPECT_FALSE(JointRolloutPacket::decode(a,10.1,.5,1.,q));
 a=p.encode();a[1]=4.5;EXPECT_FALSE(JointRolloutPacket::decode(a,10.1,.5,1.,q));
}
TEST(JointRollout, ClosedLoopFromRestEscapesAnApproachingBody) {
 JointRolloutPlanner planner;IntentFrs tr;tr.velocity={-.6,0};tr.collision_radius=.65;
 Eigen::Vector3d p=guide.front();double yaw=0,v=0,w=0,min_gap=100.;JointRolloutResult plan;
 bool turned=false;int missing=0;
 for(int k=0;k<120;++k){
  const double t=k*.05;tr.position={2.-.6*t,0.};
  min_gap=std::min(min_gap,(p.head<2>()-tr.position).norm()-.65);
  if(k%6==0){plan=planner.choose(p,yaw,v,w,guide.back(),guide,{{&tr,0.,"approach"}},[](const auto&,double){return true;},t);if(!plan.feasible)++missing;}
  const double age=(k%6)*.05;
  JointControl u=plan.feasible?plan.controls[0]:JointControl{};
  if(plan.feasible && age<.05)u={v,w};
  v+=std::clamp(u.v-v,-.025,.025);w+=std::clamp(u.w-w,-.1,.1);
  p.x()+=v*.05*std::cos(yaw+.025*w);p.y()+=v*.05*std::sin(yaw+.025*w);yaw=jointAngle(yaw+w*.05);turned|=std::abs(p.y())>.3;
 }
 EXPECT_GE(min_gap,0.);EXPECT_TRUE(turned);EXPECT_GT(p.x(),.8);EXPECT_LT(missing,4);
}
