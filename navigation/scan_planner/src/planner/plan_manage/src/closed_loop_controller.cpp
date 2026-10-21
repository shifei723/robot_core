#include <scan_planner_msgs/msg/dynamic_obstacle_array.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <sstream>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scan_planner_msgs/msg/bspline.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "plan_manage/height_safety.hpp"
#include "plan_manage/heading_tracker.hpp"
#include "plan_manage/intent_frs.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>

#include "bspline_opt/uniform_bspline.h"

namespace scan_planner
{
class ClosedLoopController : public rclcpp::Node
{
public:
  ClosedLoopController() : Node("closed_loop_controller")
  {
    time_forward_ = declare_parameter<double>("time_forward", 0.8);
    heading_error_threshold_ = declare_parameter<double>("heading_error_threshold", 0.8);
    kp_pos_ = declare_parameter<double>("kp_pos", 0.8);
    kp_yaw_ = declare_parameter<double>("kp_yaw", 1.5);
    max_vx_ = declare_parameter<double>("max_vx", 0.75);
    max_vy_ = declare_parameter<double>("max_vy", 0.35);
    max_vyaw_ = std::min(declare_parameter<double>("max_vyaw", 1.0), kMaxVYawLimit);
    finish_dist_ = declare_parameter<double>("finish_dist", 0.15);
    leg_height_min_ = declare_parameter<double>("leg_height_min", 0.13);
    leg_height_max_ = declare_parameter<double>("leg_height_max", 0.37);
    leg_height_default_ = declare_parameter<double>("leg_height_default", 0.25);
    // base_link 世界高度 = 腿高 + 偏置 (轮半径 0.0922 - 高电机高 0.06)
    base_z_offset_ = declare_parameter<double>("base_z_offset", 0.0322);

    height_safety_enabled_ = declare_parameter<bool>("height_safety_enabled", false);
    prior_required_=declare_parameter<bool>("static_prior_required",false);
    if(prior_required_) prior_applied_sub_=create_subscription<sensor_msgs::msg::PointCloud2>("grid_map/static_prior_applied",1,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        const double stamp=rclcpp::Time(msg->header.stamp).seconds();
        if(msg->header.frame_id=="world" && stamp<=now().seconds()+.05 && now().seconds()-stamp<.8)
          prior_applied_stamp_=stamp;
      });
    dynamic_mode_=declare_parameter<std::string>("dynamic_prediction", "off");
    dynamic_input_=declare_parameter<std::string>("dynamic_input", "legacy");
    dynamic_horizon_=declare_parameter<double>("dynamic_horizon",3.0);
    dynamic_soft_=declare_parameter<bool>("dynamic_soft_constraint",false);
    dynamic_weight_=declare_parameter<double>("dynamic_frs_weight",2.0);
    if(!std::isfinite(dynamic_weight_) || dynamic_weight_<0) throw std::invalid_argument("negative FRS weight");
    if(!std::isfinite(dynamic_horizon_) || dynamic_horizon_<0.5 || dynamic_horizon_>3.0)
      throw std::invalid_argument("dynamic_horizon must be within [0.5,3.0]");
    if(dynamic_input_!="legacy" && dynamic_input_!="array") throw std::invalid_argument("dynamic_input must be legacy/array");
    dynamic_min_tracks_=declare_parameter<int>("dynamic_min_tracks",1);
    if(dynamic_min_tracks_<1) throw std::invalid_argument("dynamic_min_tracks must be positive");
    if(dynamic_mode_!="off" && dynamic_mode_!="cv" && dynamic_mode_!="frs" && dynamic_mode_!="physical")
      throw std::invalid_argument("dynamic_prediction must be off/cv/frs/physical");
    dynamic_log_=create_publisher<std_msgs::msg::String>("dynamic_navigation/decision",10);
    if(dynamic_mode_!="off" && dynamic_input_=="array") {
      obstacle_sub_=create_subscription<scan_planner_msgs::msg::DynamicObstacleArray>("/dynamic_obstacles/tracks",10,
        [this](scan_planner_msgs::msg::DynamicObstacleArray::ConstSharedPtr msg) {
          std::vector<ObstacleSample> samples;
          for(const auto& o:msg->obstacles) {
            ObstacleSample s; s.id=o.id; s.position={o.position.x,o.position.y};
            s.velocity={o.velocity.x,o.velocity.y}; s.radius=o.radius; s.observed=o.observed;
            s.last_observed=rclcpp::Time(o.last_observed).seconds();
            for(int i=0;i<4;++i) for(int j=0;j<4;++j) s.covariance(i,j)=o.covariance[i*4+j];
            samples.push_back(s);
          }
          obstacle_tracks_.accept(rclcpp::Time(msg->header.stamp).seconds(),msg->epoch,
            msg->healthy && msg->header.frame_id=="world",samples,now().seconds());
        });
    }
    if(dynamic_mode_!="off" && dynamic_input_=="legacy") {
      pedestrian_sub_=create_subscription<nav_msgs::msg::Odometry>("dynamic_navigation/pedestrian",10,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          // Independent histories keyed by stable tracker identity.
          if(msg->header.frame_id!="world") return;
          const double stamp=rclcpp::Time(msg->header.stamp).seconds();
          if(stamp>now().seconds()+0.05) return;
          pedestrians_.update(msg->child_frame_id,stamp,{msg->pose.pose.position.x,msg->pose.pose.position.y},
                                    {msg->twist.twist.linear.x,msg->twist.twist.linear.y});
        });
    }
    stable_heading_ = declare_parameter<bool>("stable_heading", true);
    height_rate_ = declare_parameter<double>("height_command_rate", 0.08);
    height_tolerance_ = declare_parameter<double>("height_tolerance", 0.008);
    data_timeout_ = declare_parameter<double>("data_timeout", 0.8);
    state_pub_ = create_publisher<std_msgs::msg::String>("height_control/state", 10);
    if (height_safety_enabled_) {
      planner_status_sub_ = create_subscription<std_msgs::msg::String>("planning/status", 10,
          [this](std_msgs::msg::String::ConstSharedPtr msg) {
            if (msg->data=="NO_PATH") no_path_=true;
            else if (msg->data=="GEN_NEW_TRAJ" || msg->data=="EXEC_TRAJ") no_path_=false;
          });
      map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>("grid_map/occupancy",
          rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
            if (msg->header.frame_id != "world") return;
            std::vector<Eigen::Vector3d> occupied;
            try {
              sensor_msgs::PointCloud2ConstIterator<float> x(*msg,"x"), y(*msg,"y"), z(*msg,"z");
              for (;x!=x.end();++x,++y,++z)
                if (std::isfinite(*x)&&std::isfinite(*y)&&std::isfinite(*z))
                  occupied.emplace_back(*x,*y,*z);
            } catch (const std::runtime_error &) { return; }
            safety_.occupied=std::move(occupied); map_time_=now(); have_map_=true;
          });
      cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>("cloud",
          rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr) {
            cloud_time_=now(); have_cloud_=true;
          });
    }
    bspline_sub_ = create_subscription<scan_planner_msgs::msg::Bspline>(
        "planning/bspline", 10,
        std::bind(&ClosedLoopController::bsplineCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&ClosedLoopController::odomCallback, this, std::placeholders::_1));
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 20);
    leg_height_pub_ = create_publisher<std_msgs::msg::Float32>("leg_height_cmd", 10);
    execution_frozen_pub_ = create_publisher<std_msgs::msg::Bool>("planning/go2_execution_frozen", 10);
    cmd_timer_ = create_wall_timer(std::chrono::milliseconds(10),
                                   std::bind(&ClosedLoopController::cmdCallback, this));
    last_update_time_ = now();
    RCLCPP_INFO(get_logger(), "Closed-loop controller ready");
  }

private:
  static constexpr double kMaxVYawLimit = 1.0;

  static double normalizeAngle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  static Eigen::Vector2d clampNorm(const Eigen::Vector2d &value, double max_norm)
  {
    const double norm = value.norm();
    return (norm <= max_norm || norm < 1e-6) ? value : value / norm * max_norm;
  }

  double estimateDesiredYaw(double t_cur, const Eigen::Vector3d &pos_des) const
  {
    const double t_look = std::min(traj_duration_, t_cur + time_forward_);
    Eigen::Vector3d direction = traj_[0].evaluateDeBoorT(t_look) - pos_des;
    if (direction.head<2>().squaredNorm() < 1e-4)
      direction = traj_[1].evaluateDeBoorT(t_cur);
    return direction.head<2>().squaredNorm() < 1e-4
        ? odom_yaw_ : std::atan2(direction.y(), direction.x());
  }

  void publishStop(double yaw_rate = 0.0)
  {
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = std::clamp(yaw_rate, -max_vyaw_, max_vyaw_);
    cmd_vel_pub_->publish(cmd);
  }

  // 入参为轨迹的 base_link 世界高度, 换算成五连杆腿高后下发
  void publishLegHeightFromBaseZ(double base_z)
  {
    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(
        std::clamp(base_z - base_z_offset_, leg_height_min_, leg_height_max_));
    leg_height_pub_->publish(msg);
    last_height_cmd_=msg.data;
  }

  void publishExecutionFrozen(bool frozen)
  {
    std_msgs::msg::Bool msg;
    msg.data = frozen;
    execution_frozen_pub_->publish(msg);
  }

  void bsplineCallback(const scan_planner_msgs::msg::Bspline::ConstSharedPtr msg)
  {
    if (msg->order != 3 || msg->pos_pts.size() < 6 ||
        msg->knots.size() != msg->pos_pts.size() + msg->order + 1 ||
        !std::all_of(msg->pos_pts.begin(),msg->pos_pts.end(),[](const auto &p) {
          return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z); }) ||
        !std::all_of(msg->knots.begin(),msg->knots.end(),[](double t){return std::isfinite(t);}) ||
        !std::is_sorted(msg->knots.begin(),msg->knots.end()) ||
        msg->knots[msg->pos_pts.size()]<=msg->knots[msg->order])
    {
      RCLCPP_WARN(get_logger(), "Ignoring invalid B-spline");
      return;
    }
    Eigen::MatrixXd points(3, msg->pos_pts.size());
    for (size_t i = 0; i < msg->pos_pts.size(); ++i)
      points.col(i) << msg->pos_pts[i].x, msg->pos_pts[i].y, msg->pos_pts[i].z;
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t i = 0; i < msg->knots.size(); ++i) knots(i) = msg->knots[i];
    UniformBspline position(points, msg->order, 0.1);
    position.setKnot(knots);
    traj_ = {position, position.getDerivative()};
    traj_.push_back(traj_[1].getDerivative());
    traj_duration_ = traj_[0].getTimeSum();
    traj_id_ = msg->traj_id;
    exec_time_ = 0.0;
    last_update_time_ = now();
    receive_traj_ = true;
    RCLCPP_INFO(get_logger(), "Received trajectory %lld, duration %.3fs",
                static_cast<long long>(traj_id_), traj_duration_);
  }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    odom_pos_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    odom_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
    if (!odom_pos_.allFinite() || !std::isfinite(odom_yaw_)) { have_odom_=false; return; }
    if (!have_odom_) last_height_cmd_=std::clamp(odom_pos_.z()-base_z_offset_,leg_height_min_,leg_height_max_);
    have_odom_ = true;
    odom_time_=now();
  }

  void legacyCmdCallback()
  {
    if (!receive_traj_ || !have_odom_)
    {
      publishExecutionFrozen(false);
      publishStop();
      publishLegHeightFromBaseZ(leg_height_default_ + base_z_offset_);
      return;
    }
    const auto current_time = now();
    double dt = (current_time - last_update_time_).seconds();
    if (dt < 0.0 || dt > 0.2) dt = 0.0;
    const double t_eval = std::min(exec_time_, traj_duration_);
    Eigen::Vector3d pos_des = traj_[0].evaluateDeBoorT(t_eval);
    const double yaw_error = normalizeAngle(estimateDesiredYaw(t_eval, pos_des) - odom_yaw_);
    const double yaw_command = std::clamp(kp_yaw_ * yaw_error, -max_vyaw_, max_vyaw_);
    if (std::abs(yaw_error) > heading_error_threshold_)
    {
      publishExecutionFrozen(true);
      publishStop(yaw_command);
      publishLegHeightFromBaseZ(pos_des.z());
      last_update_time_ = current_time;
      return;
    }

    publishExecutionFrozen(false);
    exec_time_ = std::min(traj_duration_, exec_time_ + dt);
    last_update_time_ = current_time;
    pos_des = traj_[0].evaluateDeBoorT(exec_time_);
    const Eigen::Vector3d vel_des = traj_[1].evaluateDeBoorT(exec_time_);
    const Eigen::Vector2d pos_error(pos_des.x() - odom_pos_.x(), pos_des.y() - odom_pos_.y());
    const Eigen::Vector2d vel_world = clampNorm(
        Eigen::Vector2d(vel_des.x(), vel_des.y()) + kp_pos_ * pos_error,
        std::max(max_vx_, max_vy_));
    const double c = std::cos(odom_yaw_);
    const double s = std::sin(odom_yaw_);
    geometry_msgs::msg::Twist command;
    command.linear.x = std::clamp(c * vel_world.x() + s * vel_world.y(), -max_vx_, max_vx_);
    command.linear.y = std::clamp(-s * vel_world.x() + c * vel_world.y(), -max_vy_, max_vy_);
    command.angular.z = yaw_command;
    if (exec_time_ >= traj_duration_ && pos_error.norm() < finish_dist_)
      command = geometry_msgs::msg::Twist();
    cmd_vel_pub_->publish(command);
    publishLegHeightFromBaseZ(pos_des.z());
  }

  void reportState(const std::string &state) {
    std_msgs::msg::String msg; msg.data=state; state_pub_->publish(msg);
  }

  void cmdCallback()
  {
    if (!height_safety_enabled_ && dynamic_mode_=="off") { legacyCmdCallback(); return; }
    const auto current=now();
    double dt=(current-last_update_time_).seconds();
    last_update_time_=current;
    dt=std::clamp(dt,0.0,0.05);
    auto hold = [&](const std::string &state) {
      publishStop(); publishExecutionFrozen(true);
      // Hold the last commanded posture: never stand up blindly on a timeout.
      publishLegHeightFromBaseZ(last_height_cmd_+base_z_offset_); reportState(state);
    };
    if (!have_odom_ || (current-odom_time_).seconds()>data_timeout_) { hold("WAIT_ODOM"); return; }
    if (height_safety_enabled_ && (!have_map_ || !have_cloud_ ||
        (current-map_time_).seconds()>data_timeout_ || (current-cloud_time_).seconds()>data_timeout_)) {
      hold("WAIT_MAP"); return;
    }
    if(prior_required_ && (prior_applied_stamp_<0 || current.seconds()-prior_applied_stamp_>.8)) {
      hold("WAIT_STATIC_PRIOR");return;
    }
    if (no_path_) { hold("NO_PATH"); return; }
    if (!receive_traj_) { hold("IDLE"); return; }

    const double t=std::min(exec_time_,traj_duration_);
    const Eigen::Vector3d desired=traj_[0].evaluateDeBoorT(t);
    const Eigen::Vector3d feedforward=traj_[1].evaluateDeBoorT(t);
    const Eigen::Vector2d error=desired.head<2>()-odom_pos_.head<2>();
    // Steer towards tracking correction as well as the tangent: a differential
    // drive cannot execute the lateral velocity discarded by max_vy=0.
    const Eigen::Vector2d velocity=feedforward.head<2>()+kp_pos_*error;
    double yaw_error=normalizeAngle(estimateDesiredYaw(t,desired)-odom_yaw_);
    if (velocity.norm()>0.02) yaw_error=normalizeAngle(std::atan2(velocity.y(),velocity.x())-odom_yaw_);
    double w=std::clamp(kp_yaw_*yaw_error,-max_vyaw_,max_vyaw_);
    double v=std::clamp(velocity.norm()*std::max(0.0,std::cos(yaw_error)),0.0,max_vx_);
    if (stable_heading_) {
      // A spatially meaningful forward secant survives vertical-only spline
      // segments and near-zero derivatives at replan boundaries.
      Eigen::Vector2d tangent=Eigen::Vector2d::Zero();
      for (double ahead=0.1;ahead<=4.0;ahead+=0.1) {
        tangent=(traj_[0].evaluateDeBoorT(std::min(t+ahead,traj_duration_))-desired).head<2>();
        if (tangent.norm()>=0.08) break;
      }
      if (tangent.norm()<0.005) {
        tangent=feedforward.head<2>();
        if (tangent.norm()<0.005) tangent=Eigen::Vector2d(std::cos(odom_yaw_),std::sin(odom_yaw_));
      }
      const auto tracking=forwardHeadingCommand(tangent,error,feedforward.head<2>().norm(),
                                                kp_pos_,max_vx_,odom_yaw_);
      yaw_error=normalizeAngle(tracking.yaw-odom_yaw_);
      w=std::clamp(kp_yaw_*yaw_error,-max_vyaw_,max_vyaw_);
      v=tracking.speed*std::max(0.0,std::cos(yaw_error));
    }
    bool frozen=false;
    if (std::abs(yaw_error)>heading_error_threshold_) { v=0; frozen=true; }
    if (error.norm()>0.12) frozen=true;

    double target_z=desired.z();
    // Lower before the leading body edge reaches the lintel. Do not restore
    // height until the *trailing* edge has left it (current footprint cap).
    for(double ahead=0.1;ahead<=1.0;ahead+=0.1)
      target_z=std::min(target_z,traj_[0].evaluateDeBoorT(std::min(t+ahead,traj_duration_)).z());
    if (height_safety_enabled_) {
      double cap=safety_.sweptMaxBaseHeight(odom_pos_,odom_yaw_,v,w,0.6);
      if (cap<leg_height_min_+base_z_offset_) {
        v=0; frozen=true;
        cap=safety_.sweptMaxBaseHeight(odom_pos_,odom_yaw_,0,w,0.6);
        if (cap<odom_pos_.z()) { hold("BLOCKED"); return; }
      }
      target_z=std::min(target_z,cap-0.005);
    }
    target_z=std::clamp(target_z,leg_height_min_+base_z_offset_,leg_height_max_+base_z_offset_);
    const double target_h=target_z-base_z_offset_;
    const double h=last_height_cmd_+std::clamp(target_h-last_height_cmd_,-height_rate_*dt,height_rate_*dt);
    publishLegHeightFromBaseZ(h+base_z_offset_);
    std::string state=frozen ? "ALIGNING" : "TRACKING";
    if (stable_heading_) {
      if (odom_pos_.z()>target_z+height_tolerance_) lowering_=true;
      if (odom_pos_.z()<=target_z+0.003) lowering_=false;
    }
    if (odom_pos_.z()>target_z+height_tolerance_ || (stable_heading_ && lowering_)) {
      v=0; w=0; frozen=true; state="LOWERING";
    }
    if (height_safety_enabled_) {
      // Final check at measured height; lowering the command is not proof that
      // the physical body has reached that height yet.
      if (safety_.sweptMaxBaseHeight(odom_pos_,odom_yaw_,v,w,0.3)<odom_pos_.z()) {
        v=0; w=0; frozen=true; state="LOWERING";
      }
    }
    const Eigen::Vector2d endpoint=traj_[0].evaluateDeBoorT(traj_duration_).head<2>();
    const bool terminal_capture=stable_heading_ &&
        (endpoint-odom_pos_.head<2>()).norm()<std::min(0.01,finish_dist_) &&
        (endpoint-desired.head<2>()).norm()<0.03;
    if ((t>=traj_duration_ && error.norm()<finish_dist_) || terminal_capture) {
      v=0;w=0;state="GOAL_REACHED";
      if (terminal_capture) { exec_time_=traj_duration_; frozen=false; }
    }
    double progress_scale=1.0;
    if(dynamic_mode_!="off") {
      const auto tracks=(dynamic_input_=="array" ? obstacle_tracks_.bank : pedestrians_).views(current.seconds());
      const bool stale=dynamic_input_=="array" ? !obstacle_tracks_.fresh(current.seconds()) :
        !pedestrians_.fresh(current.seconds(),dynamic_min_tracks_);
      TimingDecision decision;
      const double before_v=v;
      const auto clock_start=std::chrono::steady_clock::now();
      if(!stale) {
        auto candidate=[&](double scale,double ahead) {
          // Freeze candidate progression when height/alignment already requires a hold.
          const double advance=frozen ? 0.0 : scale*ahead;
          Eigen::Vector2d point=odom_pos_.head<2>()+
            (traj_[0].evaluateDeBoorT(std::min(t+advance,traj_duration_))-desired).head<2>();
          return point;
        };
        const double hard_horizon=0.5+std::abs(before_v)/0.3; // latency + braking at validated 0.3 m/s^2
        decision=dynamic_soft_ ? chooseSoftTiming(tracks,dynamic_mode_,candidate,dynamic_horizon_,hard_horizon,dynamic_weight_)
          : chooseTiming(tracks,dynamic_mode_,candidate,0.83,dynamic_horizon_);
      }
      if(!stale && dynamic_input_=="array" && tracks.empty()) { decision.scale=1.; decision.feasible=true; }
      progress_scale=stale ? 0.0 : decision.scale;
      v*=progress_scale; w*=progress_scale;
      if(progress_scale==0) { frozen=true; state=stale ? "WAIT_TRACK" : "DYNAMIC_WAIT"; }
      else if(progress_scale<1) state="DYNAMIC_SLOW";
      // Independent reactive layer, identical in all prediction modes. Count separately.
      const bool shield=std::any_of(tracks.begin(),tracks.end(),[&](const auto &view) {
        return view.age>=0 && view.age<=0.5 && view.track->predict(view.age,"cv").clearance(odom_pos_.head<2>(),
          view.track->collision_radius>0 ? view.track->collision_radius-0.05 : 0.78)<0;
      });
      if(shield) { v=0;w=0;frozen=true;progress_scale=0;state="DYNAMIC_SHIELD"; }
      const double elapsed=std::chrono::duration<double,std::milli>(
          std::chrono::steady_clock::now()-clock_start).count();
      if(current.seconds()-last_dynamic_log_>=0.099) {
        last_dynamic_log_=current.seconds();
        std::ostringstream out; out.precision(17);
        out<<"{\"stamp\":"<<current.seconds()<<",\"mode\":\""<<dynamic_mode_
           <<"\",\"scale\":"<<progress_scale<<",\"stale\":"<<stale
           <<",\"feasible\":"<<decision.feasible<<",\"shield\":"<<shield
           <<",\"nominal_clearance\":"<<decision.nominal_clearance
           <<",\"chosen_clearance\":"<<decision.chosen_clearance
           <<",\"decision_ms\":"<<elapsed<<",\"before_v\":"<<before_v
           <<",\"v\":"<<v<<",\"x\":"<<odom_pos_.x()<<",\"y\":"<<odom_pos_.y()
           <<",\"soft_constraint\":"<<dynamic_soft_<<",\"frs_weight\":"<<dynamic_weight_<<",\"horizon\":"<<dynamic_horizon_<<",\"track_count\":"<<tracks.size()<<",\"nominal_obstacle\":\""<<decision.nominal_obstacle<<"\"";
        auto boxes=[&](const TrackView &view) {
          out<<"[";
          for(int k=1;k<=30;++k) {
            const auto box=view.track->predict(std::max(0.0,view.age)+k*.1,dynamic_mode_);
            if(k>1) out<<",";
            out<<"["<<k*.1<<","<<box.center.x()<<","<<box.center.y()<<","<<box.half.x()<<","<<box.half.y()<<"]";
          }
          out<<"]";
        };
        // Legacy first-track field retained for single-pedestrian result readers.
        out<<",\"boxes\":"; if(tracks.empty())out<<"[]";else boxes(tracks.front());
        out<<",\"tracks\":[";
        for(size_t i=0;i<tracks.size();++i) {
          if(i)out<<",";
          out<<"{\"id\":\""<<tracks[i].id<<"\",\"age\":"<<tracks[i].age<<",\"boxes\":";
          boxes(tracks[i]);out<<"}";
        }
        out<<"]";
        for(const auto &name : {std::string("nominal_path"),std::string("chosen_path")}) {
          out<<",\""<<name<<"\":[";
          for(int k=0;k<=30;++k) {
            const double scale=name=="nominal_path" ? 1.0 : progress_scale;
            const Eigen::Vector2d point=odom_pos_.head<2>()+
              (traj_[0].evaluateDeBoorT(std::min(t+scale*k*.1,traj_duration_))-desired).head<2>();
            if(k)out<<",";
            out<<"["<<point.x()<<","<<point.y()<<"]";
          }
          out<<"]";
        }
        out<<"}"; std_msgs::msg::String msg;msg.data=out.str();dynamic_log_->publish(msg);
      }
    }
    geometry_msgs::msg::Twist command;
    command.linear.x=v; command.angular.z=w;
    cmd_vel_pub_->publish(command);
    publishExecutionFrozen(frozen); reportState(state);
    if(!frozen) exec_time_=std::min(traj_duration_,exec_time_+dt*progress_scale);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr execution_frozen_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr leg_height_pub_;
  rclcpp::Subscription<scan_planner_msgs::msg::Bspline>::SharedPtr bspline_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;
  HeightSafety safety_;
  FrsTracks pedestrians_;
  SnapshotTracks obstacle_tracks_;
  std::string dynamic_input_;
  rclcpp::Subscription<scan_planner_msgs::msg::DynamicObstacleArray>::SharedPtr obstacle_sub_;
  int dynamic_min_tracks_{1};
  std::string dynamic_mode_{"off"};
  double last_dynamic_log_{0};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pedestrian_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr dynamic_log_;
  bool no_path_{false},stable_heading_{true},lowering_{false};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr planner_status_sub_;
  bool height_safety_enabled_{false}, have_map_{false}, have_cloud_{false};
  bool prior_required_{false};
  double prior_applied_stamp_{-1};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr prior_applied_sub_;
  bool dynamic_soft_{false};
  double dynamic_weight_{2.0};
  double dynamic_horizon_{3.0};
  double height_rate_{0.08},height_tolerance_{0.008},data_timeout_{0.8},last_height_cmd_{0.25};
  rclcpp::Time odom_time_{0,0,RCL_ROS_TIME},map_time_{0,0,RCL_ROS_TIME},cloud_time_{0,0,RCL_ROS_TIME};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_,cloud_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  bool receive_traj_{false};
  bool have_odom_{false};
  std::vector<UniformBspline> traj_;
  double traj_duration_{0.0};
  std::int64_t traj_id_{0};
  Eigen::Vector3d odom_pos_{Eigen::Vector3d::Zero()};
  double odom_yaw_{0.0};
  double exec_time_{0.0};
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  double time_forward_, heading_error_threshold_, kp_pos_, kp_yaw_;
  double max_vx_, max_vy_, max_vyaw_, finish_dist_;
  double leg_height_min_, leg_height_max_, leg_height_default_;
  double base_z_offset_;
};
}  // namespace scan_planner

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_planner::ClosedLoopController>());
  rclcpp::shutdown();
  return 0;
}
