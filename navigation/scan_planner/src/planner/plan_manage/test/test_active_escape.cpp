#include <gtest/gtest.h>
#include "plan_manage/active_escape.hpp"
using namespace scan_planner;
TEST(ActiveEscape, DisabledAndSafeWaitingDoNotMove) {
  ActiveEscape m;IntentFrs t;t.position={1.85,0};t.velocity={-.5,0};t.collision_radius=.65;
  std::vector<TrackView> tracks{{&t,0,"a"}};auto free=[](const auto&,double){return true;};
  auto d=m.step(0.,.025,{0,0},0.,0.,0.,{5,0},tracks,"frs",true,false,free);
  EXPECT_FALSE(d.active);m.config.enabled=true;t.velocity={.5,0};
  d=m.step(0.,.025,{0,0},0.,0.,0.,{5,0},tracks,"frs",true,false,free);
  EXPECT_FALSE(d.active);EXPECT_FALSE(d.triggered);
}
TEST(ActiveEscape, ApproachingObstacleCanEscapeWithBoundedForwardCommand) {
  ActiveEscape m;m.config.enabled=true;IntentFrs t;t.position={1.85,0};t.velocity={-.5,0};t.collision_radius=.65;
  std::vector<TrackView> tracks{{&t,0,"a"}};
  const auto d=m.step(0.,.025,{0,0},0.,0.,0.,{5,0},tracks,"frs",true,false,[](const auto&,double){return true;});
  ASSERT_TRUE(d.active);EXPECT_LT(d.parked_clearance,0);EXPECT_GE(d.chosen_clearance,0);
  EXPECT_GT(d.v,0);EXPECT_LE(d.v,.5*.025+1e-9);EXPECT_LE(std::abs(d.w),2*.025+1e-9);
  ASSERT_GT(d.path.size(),2u);EXPECT_GT(std::abs(d.path.back().y()),.3);
}
TEST(ActiveEscape, StaticAndUnknownOccupancyCannotBeBypassed) {
  ActiveEscape m;m.config.enabled=true;IntentFrs t;t.position={1.85,0};t.velocity={-.5,0};t.collision_radius=.65;
  std::vector<TrackView> tracks{{&t,0,"a"}};
  auto d=m.step(0.,.025,{0,0},0.,0.,0.,{5,0},tracks,"frs",true,false,[](const auto& p,double){return std::abs(p.y())<.01;});
  EXPECT_FALSE(d.active);EXPECT_TRUE(d.triggered);
  IntentFrs u;u.position={0,0};u.collision_radius=.5;u.unknown=true;tracks.push_back({&u,0,"unknown"});
  d=m.step(.1,.025,{0,0},0.,0.,0.,{5,0},tracks,"frs",true,false,[](const auto&,double){return true;});
  EXPECT_FALSE(d.active);EXPECT_EQ(d.reason,"no_safe_candidate");EXPECT_TRUE(u.unknown);
}
TEST(ActiveEscape, AnticipatesBeforeStopAndDoesNotLatchForever) {
  ActiveEscape m;m.config.enabled=true;IntentFrs t;t.position={1.85,0};t.velocity={-.5,0};t.collision_radius=.65;
  std::vector<TrackView> tracks{{&t,0,"a"}};auto free=[](const auto&,double){return true;};
  auto d=m.step(0.,.025,{0,0},0.,0.,0.,{5,0},tracks,"frs",false,true,free);EXPECT_TRUE(d.triggered);
  d=m.step(.1,.025,{0,0},0.,0.,0.,{5,0},tracks,"frs",true,false,free);ASSERT_TRUE(d.active);
  tracks.clear();d=m.step(.2,.025,{0,0},0.,0.,0.,{5,0},tracks,"frs",false,true,free);EXPECT_FALSE(m.active());EXPECT_FALSE(d.active);
}
TEST(ActiveEscape, CommandsProgressBetweenSlowerOdometryUpdates) {
  ActiveEscape m;m.config.enabled=true;IntentFrs t;t.position={1.85,0};t.velocity={-.5,0};t.collision_radius=.65;
  std::vector<TrackView> tracks{{&t,0,"a"}};auto free=[](const auto&,double){return true;};
  const auto first=m.step(0.,.01,{0,0},0.,0.,0.,{5,0},tracks,"frs",true,false,free);
  const auto second=m.step(.01,.01,{0,0},0.,0.,0.,{5,0},tracks,"frs",true,false,free);
  ASSERT_TRUE(first.active);ASSERT_TRUE(second.active);
  EXPECT_GT(std::abs(second.w),std::abs(first.w));
  EXPECT_LE(std::abs(second.w-first.w),.02+1e-9);
  EXPECT_LE(second.v-first.v,.005+1e-9);
  m.reset();const auto fresh=m.step(.1,.01,{0,0},0.,0.,0.,{5,0},tracks,"frs",true,false,free);
  EXPECT_LE(std::abs(fresh.w),.02+1e-9);
}
TEST(ActiveEscape, ClosedLoopApproachMaintainsPhysicalSeparation) {
  ActiveEscape m;m.config.enabled=true;IntentFrs t;t.velocity={-.5,0};t.collision_radius=.65;
  std::vector<TrackView> tracks{{&t,0,"a"}};
  Eigen::Vector2d p{0,0};double yaw=0,v=0,w=0,min_gap=10;int active=0;
  for(int k=0;k<500;++k) {
    const double now=k*.01;t.position={1.85-.5*now,0.};
    const auto d=m.step(now,.01,p,yaw,v,w,{5,0},tracks,"cv",true,false,[](const auto&,double){return true;});
    v=d.active?d.v:0.;w=d.active?d.w:0.;active+=d.active;
    p+=v*.01*Eigen::Vector2d(std::cos(yaw+.005*w),std::sin(yaw+.005*w));yaw+=.01*w;
    min_gap=std::min(min_gap,(p-t.position).norm()-.65);
  }
  EXPECT_GT(active,0);EXPECT_GE(min_gap,-1e-6);EXPECT_GT(std::abs(p.y()),.3);
}
