#pragma once
#include <algorithm>
#include <cmath>
#include <Eigen/Core>
namespace scan_planner {
struct HeadingCommand { double speed, yaw; };
// Resolve tracking errors in the forward path frame. A negative longitudinal
// correction stops translation; it never reverses the reference heading.
inline HeadingCommand forwardHeadingCommand(const Eigen::Vector2d &tangent,
    const Eigen::Vector2d &error, double feedforward_speed, double kp,
    double max_speed, double fallback_yaw) {
  if (tangent.norm()<1e-6) return {0.0,fallback_yaw};
  const Eigen::Vector2d unit=tangent.normalized();
  const double along=unit.dot(error);
  const double lateral=unit.x()*error.y()-unit.y()*error.x();
  const double correction=std::clamp(std::atan2(kp*lateral,
      std::max(0.12,feedforward_speed)),-0.5,0.5);
  return {std::clamp(feedforward_speed+kp*along,0.0,max_speed),
          std::atan2(unit.y(),unit.x())+correction};
}
}
