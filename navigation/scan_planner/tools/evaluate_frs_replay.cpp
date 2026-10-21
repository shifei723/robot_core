// Evaluate the production IntentFrs predictor on offline paired perception states.
#include "plan_manage/intent_frs.hpp"
#include <iostream>
#include <iomanip>
int main() {
  scan_planner::IntentFrs track;
  track.windowed_acceleration=true;
  double time,px,py,vx,vy,ppx,ppy,pvx,pvy;
  std::cout<<std::setprecision(12);
  while(std::cin>>time>>px>>py>>vx>>vy>>ppx>>ppy>>pvx>>pvy) {
    if(!track.update(time,{px,py},{vx,vy})) return 2;
    track.position_half={3*std::sqrt(ppx),3*std::sqrt(ppy)};
    track.velocity_half={3*std::sqrt(pvx),3*std::sqrt(pvy)};
    const auto box=track.predict(1.5,"frs");
    std::cout<<time<<" "<<box.center.x()<<" "<<box.center.y()<<" "<<2*box.half.x()<<" "<<2*box.half.y()<<"\n";
  }
}
