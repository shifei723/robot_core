#include <gtest/gtest.h>
#include "plan_manage/intent_frs.hpp"
using namespace scan_planner;
TEST(IntentFrs, HistoriesRemainIndependentWhenMessagesInterleave) {
  FrsTracks bank;
  for(int i=0;i<12;++i) {
    double t=i*.1;
    ASSERT_TRUE(bank.update("pedestrian_1",t,{10-t,0},{-1,0}));
    ASSERT_TRUE(bank.update("pedestrian_0",t,{.5*t*t,0},{t,0}));
  }
  EXPECT_NEAR(bank.tracks.at("pedestrian_0").center.x(),1,1e-10);
  EXPECT_NEAR(bank.tracks.at("pedestrian_1").center.x(),0,1e-10);
  EXPECT_TRUE(bank.fresh(1.2,2));
  EXPECT_FALSE(bank.fresh(1.2,3));
  bank.update("pedestrian_0",2,{2,0},{2,0});
  EXPECT_FALSE(bank.fresh(2,2)); // One missing target must not disappear silently.
  EXPECT_FALSE(bank.update("pedestrian_bad",2,{0,0},{0,0}));
}
TEST(IntentFrs, EveryObstacleParticipatesAndOrderingDoesNotMatter) {
  IntentFrs far,near;far.update(0,{10,10},{0,0});near.update(0,{1.2,0},{0,0});
  auto path=[](double s,double t){return Eigen::Vector2d(.3*s*t,0);};
  EXPECT_EQ(chooseTiming(far,0,"cv",path,.4).scale,1);
  std::vector<TrackView> views{{&far,0,"pedestrian_0"},{&near,0,"pedestrian_1"}};
  auto a=chooseTiming(views,"cv",path,.4);
  EXPECT_LT(a.scale,1);EXPECT_EQ(a.nominal_obstacle,"pedestrian_1");
  std::reverse(views.begin(),views.end());
  auto b=chooseTiming(views,"cv",path,.4);
  EXPECT_EQ(a.scale,b.scale);EXPECT_DOUBLE_EQ(a.nominal_clearance,b.nominal_clearance);
}
TEST(IntentFrs, ConstantAccelerationAndExactRectangle) {
  IntentFrs f;
  for(int i=0;i<15;++i) { double t=i*.1; ASSERT_TRUE(f.update(t,{.5*t*t,0},{t,0})); }
  EXPECT_NEAR(f.center.x(),1,1e-10);
  EXPECT_NEAR(f.half.x(),.05,1e-10);
  const auto b=f.predict(2,"frs");
  EXPECT_NEAR(b.center.x(),.5*3.4*3.4,1e-10);
  EXPECT_NEAR(b.half.x(),.1,1e-10);
  // Extreme input sequences attain rectangle corners.
  EXPECT_NEAR(b.clearance(b.center+b.half,0),0,1e-10);
  EXPECT_NEAR(b.clearance(b.center+b.half+Eigen::Vector2d(1,0),0),1,1e-10);
}
TEST(IntentFrs, RejectOldAndInvalidObservationsResetAfterGap) {
  IntentFrs f;
  ASSERT_TRUE(f.update(1,{0,0},{1,0}));
  EXPECT_FALSE(f.update(1,{10,0},{1,0}));
  EXPECT_FALSE(f.update(0,{10,0},{1,0}));
  EXPECT_FALSE(f.update(2,{NAN,0},{1,0}));
  ASSERT_TRUE(f.update(3,{2,0},{1,0}));
  EXPECT_DOUBLE_EQ(f.half.x(),.2);
}
TEST(IntentFrs, SameCenterCVButUncertaintyChangesTiming) {
  IntentFrs f; f.update(0,{1.3,0},{0,0});
  auto path=[](double scale,double t) {return Eigen::Vector2d(.3*scale*t,0);};
  auto cv=chooseTiming(f,0,"cv",path,.4);
  auto frs=chooseTiming(f,0,"frs",path,.4);
  EXPECT_EQ(cv.scale,1);
  EXPECT_LT(frs.scale,cv.scale);
}
TEST(IntentFrs, NoSafeCandidateIsExplicit) {
  IntentFrs f; f.update(0,{0,0},{0,0});
  auto d=chooseTiming(f,0,"cv",[](double,double){return Eigen::Vector2d(0,0);});
  EXPECT_FALSE(d.feasible); EXPECT_EQ(d.scale,0);
}

static ObstacleSample sample() {
  ObstacleSample s; s.id="obstacle_1"; s.position={2,0}; s.velocity={0,0};
  s.covariance=Eigen::Matrix4d::Identity()*.001; s.radius=.3;
  s.last_observed=1; s.observed=true;return s;
}
TEST(SnapshotTracks, HealthyEmptyAndDropoutAreDifferent) {
  SnapshotTracks b;
  ASSERT_TRUE(b.accept(1,1,true,{sample()},1));
  EXPECT_EQ(b.bank.tracks.size(),1u);
  ASSERT_TRUE(b.accept(1.1,1,true,{},1.1));
  EXPECT_TRUE(b.fresh(1.2)); EXPECT_TRUE(b.bank.tracks.empty());
  EXPECT_FALSE(b.fresh(1.7));
  EXPECT_FALSE(b.accept(1.2,1,false,{},1.2)); EXPECT_FALSE(b.fresh(1.2));
}
TEST(SnapshotTracks, RejectCorruptArrayAtomically) {
  SnapshotTracks b; auto s=sample(); ASSERT_TRUE(b.accept(1,1,true,{s},1));
  s.covariance(0,0)=-1;
  EXPECT_FALSE(b.accept(1.1,1,true,{s},1.1));
  EXPECT_DOUBLE_EQ(b.bank.tracks.at(s.id).stamp,1);EXPECT_FALSE(b.fresh(1.1));
  s=sample(); EXPECT_FALSE(b.accept(1.2,1,true,{s,s},1.2));
}
TEST(SnapshotTracks, CoastingPreservesObservedHistoryAndEpochResets) {
  SnapshotTracks b; auto s=sample(); ASSERT_TRUE(b.accept(1,1,true,{s},1));
  s.observed=false; s.position.x()=100;
  ASSERT_TRUE(b.accept(1.1,1,true,{s},1.1));
  EXPECT_DOUBLE_EQ(b.bank.tracks.at(s.id).position.x(),2);
  EXPECT_DOUBLE_EQ(b.bank.tracks.at(s.id).stamp,1);
  ASSERT_TRUE(b.accept(1.2,2,true,{},1.2));EXPECT_TRUE(b.bank.tracks.empty());
  EXPECT_FALSE(b.accept(1.3,2,true,{s},1.3));
}
TEST(SnapshotTracks, ExtentAndMeasurementUncertaintyEnlargeCollisionRegion) {
  SnapshotTracks b; auto s=sample(); ASSERT_TRUE(b.accept(1,1,true,{s},1));
  const auto& t=b.bank.tracks.at(s.id);
  EXPECT_NEAR(t.collision_radius,.83,1e-12);
  EXPECT_GT(t.predict(1,"frs").half.x(),.2);
  auto path=[](double,double){return Eigen::Vector2d(1.1,0);};
  EXPECT_FALSE(chooseTiming(b.bank.views(1),"cv",path,.01).feasible);
}

TEST(SoftFrs, LargeOuterEnvelopeDoesNotForbidAllProgress) {
  IntentFrs f; f.update(0,{2,0},{0,0});f.half={10,10};f.collision_radius=.83;
  const std::vector<TrackView> tracks{{&f,0,"box"}};
  auto path=[](double s,double t){return Eigen::Vector2d(.3*s*t,0);};
  EXPECT_FALSE(chooseTiming(tracks,"frs",path).feasible);
  auto soft=chooseSoftTiming(tracks,"frs",path,3.,1.5,2.);
  EXPECT_TRUE(soft.feasible);EXPECT_DOUBLE_EQ(soft.scale,1.);
}
TEST(SoftFrs, HardBodyCannotBeTradedForProgress) {
  IntentFrs f;f.update(0,{.5,0},{0,0});f.collision_radius=.83;
  auto result=chooseSoftTiming(std::vector<TrackView>{{&f,0,"box"}},"frs",
    [](double s,double t){return Eigen::Vector2d(.3*s*t,0);},3.,1.5,0.);
  EXPECT_FALSE(result.feasible);EXPECT_DOUBLE_EQ(result.scale,0.);
}
TEST(SoftFrs, WindowRejectsFrameDifferenceAmplificationButRetainsAcceleration) {
  IntentFrs noisy,windowed;windowed.windowed_acceleration=true;
  for(int i=0;i<30;++i) {
    double t=i*.075;Eigen::Vector2d v((i%2 ? .1:-.1),0);
    noisy.update(t,{0,0},v);windowed.update(t,{0,0},v);
  }
  EXPECT_LT(windowed.half.x(),noisy.half.x()*.5);
  IntentFrs acceleration;acceleration.windowed_acceleration=true;
  for(int i=0;i<30;++i) {double t=i*.075;acceleration.update(t,{.5*t*t,0},{t,0});}
  EXPECT_NEAR(acceleration.center.x(),1.,1e-8);
}
TEST(SoftFrs, WeightChangesPreferenceWithoutChangingHardFeasibility) {
  IntentFrs f; f.update(0,{1.2,.9},{0,0}); f.collision_radius=.83;
  auto path=[](double scale,double t){return Eigen::Vector2d(.3*scale*t,0);};
  const std::vector<TrackView> tracks{{&f,0,"box"}};
  const auto zero=chooseSoftTiming(tracks,"frs",path,3.,1.5,0.);
  const auto weighted=chooseSoftTiming(tracks,"frs",path,3.,1.5,20.);
  EXPECT_TRUE(zero.feasible);EXPECT_TRUE(weighted.feasible);
  EXPECT_DOUBLE_EQ(zero.scale,1.);EXPECT_LT(weighted.scale,zero.scale);
  const auto empty=chooseSoftTiming(std::vector<TrackView>{},"frs",path,3.,1.5,20.);
  EXPECT_TRUE(empty.feasible);EXPECT_DOUBLE_EQ(empty.scale,1.);
  EXPECT_TRUE(std::isfinite(empty.nominal_clearance));
}

TEST(IntentFrs, HardTimingRetainsFastestSafeQuarterStep) {
  IntentFrs f; f.update(0,{1.1,0},{0,0});
  auto path=[](double scale,double t){return Eigen::Vector2d(.3*scale*t,0);};
  auto decision=chooseTiming(f,0,"cv",path,.4);
  EXPECT_TRUE(decision.feasible);
  EXPECT_DOUBLE_EQ(decision.scale,.75);
  EXPECT_GE(decision.chosen_clearance,0);
}
