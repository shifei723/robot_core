#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdlib.h>

#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/uniform_bspline.h>
#include <plan_env/grid_map.h>
#include <plan_manage/plan_container.hpp>
#include <rclcpp/rclcpp.hpp>
#include <plan_manage/intent_frs.hpp>
#include <plan_manage/dynamic_local_path.hpp>
#include <plan_manage/joint_rollout.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <scan_planner_msgs/msg/dynamic_obstacle_array.hpp>
#include <traj_utils/planning_visualization.h>

namespace scan_planner
{

  // Fast Planner Manager
  // Key algorithms of mapping and planning are called

  class SCANPlannerManager
  {
    // SECTION stable
  public:
    SCANPlannerManager();
    ~SCANPlannerManager();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* main planning interface */
    bool reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                       Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool flag_polyInit, bool flag_randomPolyTraj);
    bool trajectoryCollision(const Eigen::Vector3d &position, double yaw);
    bool EmergencyStop(Eigen::Vector3d stop_pos);
    bool planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                        const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    bool planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                 const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);

    void initPlanModules(rclcpp::Node *node, PlanningVisualization::Ptr vis = nullptr);

    bool canReuseStaticCorridor() const {
      if(dynamic_tracks_.fresh(node_->now().seconds()))
        for(const auto& [id,t]:dynamic_tracks_.bank.tracks) if(t.unknown) return false;
      return dynamic_spatial_weight_<=0. ||
          (dynamic_tracks_.fresh(node_->now().seconds()) && dynamic_tracks_.bank.tracks.empty());
    }
    void setExecutionState(const Eigen::Vector3d& pose,double yaw,double speed,double yaw_rate,double stamp) {
      if(pose.allFinite() && std::isfinite(yaw) && std::isfinite(speed) && std::isfinite(yaw_rate)) {
        execution_pose_=pose;execution_yaw_=yaw;execution_speed_=speed;execution_yaw_rate_=yaw_rate;execution_stamp_=stamp;
      }
    }
    bool dynamicLocalPlanDue(double interval) const {
      if(joint_rollout_enabled_)return node_->now().seconds()-last_local_selection_>=interval;
      return dynamicLocalRefreshDue(dynamic_local_enabled_,last_local_speed_,dynamic_tracks_.fresh(node_->now().seconds()),
          !dynamic_tracks_.bank.tracks.empty(),node_->now().seconds(),last_local_selection_,interval);
    }
    bool jointRolloutActive()const{return joint_rollout_enabled_ && joint_active_ && joint_id_==local_data_.traj_id_;}
    const std::vector<Eigen::Vector3d>& jointSafetyPath()const{return joint_path_;}
    double jointInputStamp()const{return joint_stamp_;}
    PlanParameters pp_;
    LocalTrajData local_data_;
    GlobalTrajData global_data_;
    GridMap::Ptr grid_map_;

  private:
    rclcpp::Node *node_{nullptr};
    /* main planning algorithms & modules */
    PlanningVisualization::Ptr visualization_;

    BsplineOptimizer::Ptr bspline_optimizer_rebound_;

    int continuous_failures_count_{0};
    bool dynamic_local_enabled_{false};
    double last_local_selection_{-1},last_local_speed_{0};
    DynamicLocalPath dynamic_local_;
    bool joint_rollout_enabled_{false};
    JointRolloutPlanner joint_rollout_;
    bool joint_active_{false};std::int64_t joint_id_{-1};double joint_stamp_{-1};
    std::vector<Eigen::Vector3d> joint_path_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_rollout_pub_;
    Eigen::Vector3d execution_pose_{Eigen::Vector3d::Zero()};
    double execution_yaw_{0},execution_speed_{0},execution_yaw_rate_{0},execution_stamp_{-1};
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr local_speed_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr local_decision_pub_;
    bool wheel_grid_trajectory_{false};
    double wheel_search_timeout_{1.0};
    double dynamic_spatial_weight_{0.0};
    SnapshotTracks dynamic_tracks_;
    rclcpp::Subscription<scan_planner_msgs::msg::DynamicObstacleArray>::SharedPtr dynamic_sub_;
    bool planWheelTrajectory(const Eigen::Vector3d &start, const Eigen::Vector3d &goal);

    void updateTrajInfo(const UniformBspline &position_traj, const rclcpp::Time time_now);
    bool checkDynamicFeasibility(UniformBspline position_traj);

    void reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio, Eigen::MatrixXd &ctrl_pts, double &dt,
                        double &time_inc);

    bool refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points);

    // !SECTION stable

    // SECTION developing

  public:
    typedef unique_ptr<SCANPlannerManager> Ptr;

    // !SECTION
  };
} // namespace scan_planner

#endif
