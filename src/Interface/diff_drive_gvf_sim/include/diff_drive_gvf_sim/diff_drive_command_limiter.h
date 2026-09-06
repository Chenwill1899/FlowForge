#ifndef DIFF_DRIVE_GVF_SIM_DIFF_DRIVE_COMMAND_LIMITER_H
#define DIFF_DRIVE_GVF_SIM_DIFF_DRIVE_COMMAND_LIMITER_H

#include <algorithm>

#include <geometry_msgs/Twist.h>

namespace FLAG_Race {

struct DiffDriveCommandLimit {
  double max_v = 0.8;
  double min_w = -0.5235;
  double max_w = 0.5235;
};

inline double clampDiffDriveValue(double value, double lower, double upper) {
  if (lower > upper) std::swap(lower, upper);
  return std::max(lower, std::min(value, upper));
}

inline geometry_msgs::Twist limitDiffDriveCommand(
    const geometry_msgs::Twist& cmd,
    const DiffDriveCommandLimit& limit) {
  geometry_msgs::Twist limited = cmd;
  limited.linear.x = clampDiffDriveValue(cmd.linear.x, 0.0,
                                         std::max(0.0, limit.max_v));
  limited.linear.y = 0.0;
  limited.angular.z =
      clampDiffDriveValue(cmd.angular.z, limit.min_w, limit.max_w);
  return limited;
}

}  // namespace FLAG_Race

#endif  // DIFF_DRIVE_GVF_SIM_DIFF_DRIVE_COMMAND_LIMITER_H
