#include <plan_manage/wheel_timing.hpp>
#include <cassert>
using namespace scan_planner;
int main() {
  std::vector<std::vector<Eigen::Vector3d>> cases;
  std::vector<Eigen::Vector3d> straight;
  for(int i=0;i<=200;++i) straight.emplace_back(i*.05,0,.25);
  cases.push_back(straight);
  std::vector<Eigen::Vector3d> old_points(3,straight.front());
  for(size_t i=1;i<straight.size();++i) old_points.push_back(straight[i]);
  old_points.push_back(straight.back());old_points.push_back(straight.back());
  Eigen::MatrixXd old_controls(3,old_points.size());
  for(size_t i=0;i<old_points.size();++i) old_controls.col(i)=old_points[i];
  UniformBspline old(old_controls,3,.2);old.setPhysicalLimits(.5,.5,0.);
  double ratio=1.;if(!old.checkFeasibility(ratio,false))old.lengthenTime(ratio*1.05);
  std::cout<<"Legacy 10 m straight duration="<<old.getTimeSum()<<" s\n";

  cases.push_back({{0,0,.25},{1,0,.25},{1,1,.25},{2,1,.25},{2,2,.25}});
  cases.push_back({{0,0,.25},{0,0,.25},{0,0,.35},{1,0,.35}});
  cases.push_back({{0,0,.25},{0,0,.25}});
  for(const auto &path:cases) {
    UniformBspline s;assert(makeLocallyTimedWheelSpline(path,.5,.5,.06,s));
    auto v=s.getDerivative(),a=v.getDerivative();
    assert((s.evaluateDeBoorT(0)-path.front()).norm()<1e-8);
    assert((s.evaluateDeBoorT(s.getTimeSum())-path.back()).norm()<1e-8);
    for(double t=0;t<=s.getTimeSum();t+=.001) {
      assert(v.evaluateDeBoorT(t).head<2>().norm()<=.500001);
      assert(std::abs(v.evaluateDeBoorT(t).z())<=.060001);
      assert(a.evaluateDeBoorT(t).norm()<=.500001);
    }
    std::cout<<"PASS duration="<<s.getTimeSum()<<" s\n";
    if(path.size()==201) assert(s.getTimeSum()<25.);
  }
  // A 1 cm feasible ceiling band must not be cut by cubic smoothing.
  UniformBspline low;
  assert(makeCornerPreservingWheelSpline({{0,0,.2822},{0,0,.1622},
                                          {1,0,.1622},{1,0,.2822}},.5,.5,.06,low));
  for(double t=0;t<=low.getTimeSum();t+=.001) {
    const auto p=low.evaluateDeBoorT(t);
    if(p.x()>.02 && p.x()<.98) assert(p.z()<=.1722);
    assert(low.getDerivative().evaluateDeBoorT(t).head<2>().norm()<=.500001);
    assert(std::abs(low.getDerivative().evaluateDeBoorT(t).z())<=.060001);
  }
  assert((low.evaluateDeBoorT(0)-Eigen::Vector3d(0,0,.2822)).norm()<1e-8);
  assert((low.evaluateDeBoorT(low.getTimeSum())-Eigen::Vector3d(1,0,.2822)).norm()<1e-8);
  // Global fallback preserves geometry and bounds all derivative controls.
  auto shape=low;
  low.setKnot((low.getKnot()*.05).eval());
  assert(globallyRetimeWheelSpline(low,.5,.5,.06));
  for(int k=0;k<=1000;++k) {
    const double t=low.getTimeSum()*k/1000.;
    assert(low.getDerivative().evaluateDeBoorT(t).head<2>().norm()<=.500001);
    assert(std::abs(low.getDerivative().evaluateDeBoorT(t).z())<=.060001);
    assert(low.getDerivative().getDerivative().evaluateDeBoorT(t).norm()<=.500001);
    assert((low.evaluateDeBoorT(t)-shape.evaluateDeBoorT(shape.getTimeSum()*k/1000.)).norm()<1e-8);
  }
}
