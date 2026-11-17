#include <gtest/gtest.h>
#include "plan_manage/dynamic_local_path.hpp"
using namespace scan_planner;
static std::vector<Eigen::Vector3d> line(){return {{0,0,.25},{1,0,.25},{2,0,.25},{3,0,.25},{4,0,.25}};}
TEST(DynamicLocalPath, DetoursRejoinAndRemainBounded) {
 const auto ref=line();
 for(double d:{-.9,-.4,0.,.4,.9}){
  const auto p=boundedDynamicDetour(ref,d);ASSERT_GT(p.size(),10u);
  EXPECT_LT((p.front()-ref.front()).norm(),1e-9);EXPECT_LT((p.back()-ref.back()).norm(),1e-9);
  for(const auto& q:p){EXPECT_LE(std::abs(q.y()),std::abs(d)+1e-9);if(q.x()>=3.)EXPECT_NEAR(q.y(),0.,1e-9);}
 }
}
TEST(DynamicLocalPath, EmptySceneKeepsNominalMovingPath) {
 DynamicLocalPath planner;const auto ref=line();
 const auto d=planner.choose(ref,ref.front(),0.,0.,0.,{},ref,[](const auto&,double){return true;});
 ASSERT_TRUE(d.feasible);EXPECT_DOUBLE_EQ(d.offset,0.);EXPECT_DOUBLE_EQ(d.speed,.5);
}
TEST(DynamicLocalPath, ApproachingThreatCanSelectBoundedNonzeroDetour) {
 DynamicLocalPath planner;const auto ref=line();IntentFrs target;target.position={3.,0};target.velocity={-.5,0};target.collision_radius=.65;
 const auto d=planner.choose(ref,ref.front(),0.,0.,0.,{{&target,0.,"target"}},ref,[](const auto&,double){return true;});
 ASSERT_TRUE(d.feasible);EXPECT_GT(d.speed,0.);EXPECT_GT(std::abs(d.offset),0.);EXPECT_GE(d.clearance,0.);
 EXPECT_LE(std::abs(d.offset),.9);EXPECT_LT((d.path.back()-ref.back()).norm(),1e-9);
}
TEST(DynamicLocalPath, OccupancyAndUnknownCannotBeRemoved) {
 DynamicLocalPath planner;const auto ref=line();IntentFrs u;u.position={0,0};u.unknown=true;u.collision_radius=.5;
 auto d=planner.choose(ref,ref.front(),0.,0.,0.,{{&u,0.,"unknown"}},ref,[](const auto&,double){return true;});
 EXPECT_FALSE(d.feasible);EXPECT_TRUE(u.unknown);
 d=planner.choose(ref,ref.front(),0.,0.,0.,{},ref,[](const auto&,double){return false;});EXPECT_FALSE(d.feasible);
}
TEST(DynamicLocalPath, UnsafeInputAndHeightAreNotSilentlyChanged) {
 DynamicLocalPath planner;auto ref=line();ref.back().z()=.15;
 auto d=planner.choose(ref,ref.front(),0.,0.,0.,{},ref,[](const auto&,double){return true;});EXPECT_FALSE(d.feasible);
 ref=line();d=planner.choose(ref,ref.front(),NAN,0.,0.,{},ref,[](const auto&,double){return true;});EXPECT_FALSE(d.feasible);
}

TEST(DynamicLocalPath, CheckedWaitRefreshesEvenAfterSceneBecomesEmpty) {
 EXPECT_TRUE(dynamicLocalRefreshDue(true,0.,true,false,1.,0.,.4));
 EXPECT_FALSE(dynamicLocalRefreshDue(true,.5,true,false,1.,0.,.4));
 EXPECT_TRUE(dynamicLocalRefreshDue(true,.5,true,true,1.,0.,.4));
 EXPECT_TRUE(dynamicLocalRefreshDue(true,.5,false,false,1.,0.,.4));
 EXPECT_FALSE(dynamicLocalRefreshDue(true,0.,true,false,.1,0.,.4));
 EXPECT_FALSE(dynamicLocalRefreshDue(false,0.,false,true,1.,0.,.4));
}
