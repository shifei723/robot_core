#include <sstream>
// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <plan_manage/wheel_timing.hpp>
#include <plan_manage/wheel_shortcut.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

namespace scan_planner
{
  namespace
  {
    void applyCeilingHeightReference(std::vector<Eigen::Vector3d> &points,
                                     const double start_z, const double target_z,
                                     const GridMap::Ptr &grid_map,
                                     const double height_min, const double height_max,
                                     const double body_top_offset, const double ceiling_margin,
                                     const bool allow_unverified_clearance)
    {
      if (points.empty())
        return;

      if (points.size() == 1)
      {
        points.front()(2) = std::clamp(start_z, height_min, height_max);
        return;
      }

      std::vector<double> accumulated_xy_length(points.size(), 0.0);
      for (size_t i = 1; i < points.size(); ++i)
      {
        accumulated_xy_length[i] = accumulated_xy_length[i - 1] +
                                   (points[i].head<2>() - points[i - 1].head<2>()).norm();
      }

      const double total_xy_length = accumulated_xy_length.back();
      for (size_t i = 0; i < points.size(); ++i)
      {
        const double ratio = total_xy_length > 1e-6
                                 ? accumulated_xy_length[i] / total_xy_length
                                 : static_cast<double>(i) / static_cast<double>(points.size() - 1);
        double z_ref = start_z + ratio * (target_z - start_z);

        const size_t prev = i == 0 ? i : i - 1;
        const size_t next = std::min(i + 1, points.size() - 1);
        const Eigen::Vector2d direction = points[next].head<2>() - points[prev].head<2>();
        const double yaw = direction.squaredNorm() < 1e-8 ? 0.0 : std::atan2(direction(1), direction(0));
        bool clearance_verified = false;
        const double z_ceil = grid_map->queryCeilingHeight(points[i], yaw, &clearance_verified);
        if ((clearance_verified || allow_unverified_clearance) && std::isfinite(z_ceil))
          z_ref = std::min(z_ref, z_ceil - body_top_offset - ceiling_margin);

        points[i](2) = std::clamp(z_ref, height_min, height_max);
      }

      points.front()(2) = std::clamp(start_z, height_min, height_max);
      points.back()(2) = std::clamp(target_z, height_min, height_max);
    }
  } // namespace

  // SECTION interfaces for setup and query

  SCANPlannerManager::SCANPlannerManager() {}

  SCANPlannerManager::~SCANPlannerManager() { std::cout << "des manager" << std::endl; }

  void SCANPlannerManager::initPlanModules(rclcpp::Node *node, PlanningVisualization::Ptr vis)
  {
    node_ = node;
    /* read algorithm parameters */
    const auto get_double = [node](const std::string &name, double default_value) {
      if (!node->has_parameter(name)) node->declare_parameter<double>(name, default_value);
      return node->get_parameter(name).as_double();
    };
    wheel_grid_trajectory_ = node_->declare_parameter<bool>("manager.wheel_grid_trajectory", false);
    node_->declare_parameter<bool>("manager.local_wheel_timing", false);
    node_->declare_parameter<bool>("manager.shortcut_wheel_path",false);
    node_->declare_parameter<double>("manager.execution_radius", 0.30);
    node_->declare_parameter<double>("manager.execution_offset", 0.18);
    const double minimum_height=node_->declare_parameter<double>("manager.execution_height_min",.1722);
    const double maximum_height=node_->declare_parameter<double>("manager.execution_height_max",.3922);
    const double height_anchor=node_->declare_parameter<double>("manager.height_lattice_anchor",-1.);
    const double search_heuristic=node_->declare_parameter<double>("manager.wheel_search_heuristic",0.);
    if(!std::isfinite(search_heuristic) || (search_heuristic!=0. && search_heuristic<1.))
      throw std::invalid_argument("wheel_search_heuristic must be zero (automatic) or >=1");
    if(!std::isfinite(minimum_height) || !std::isfinite(maximum_height) || minimum_height>=maximum_height ||
       !std::isfinite(height_anchor) || (height_anchor>=0. && (height_anchor<minimum_height || height_anchor>maximum_height)))
      throw std::invalid_argument("invalid wheel execution height range or lattice anchor");
    wheel_search_timeout_ = node_->declare_parameter<double>("manager.wheel_search_timeout", 1.0);
    dynamic_spatial_weight_ = get_double("manager.dynamic_frs_weight",0.0);
    node_->declare_parameter<double>("manager.dynamic_safety_margin",.1);
    dynamic_local_enabled_=node_->declare_parameter<bool>("manager.dynamic_local_planning",false);
    joint_rollout_enabled_=node_->declare_parameter<bool>("manager.joint_rollout_planning",false);
    if(joint_rollout_enabled_)dynamic_local_enabled_=true;
    joint_rollout_pub_=node_->create_publisher<std_msgs::msg::Float64MultiArray>("planning/joint_rollout",rclcpp::QoS(8).reliable().transient_local());
    dynamic_local_.config.braking=node_->declare_parameter<double>("manager.dynamic_local_braking",.5);
    dynamic_local_.config.yaw_rate=node_->declare_parameter<double>("manager.dynamic_local_yaw_rate",1.);
    if(dynamic_local_.config.braking<=0 || dynamic_local_.config.yaw_rate<=0)
      throw std::invalid_argument("invalid dynamic local planning limits");
    local_speed_pub_=node_->create_publisher<std_msgs::msg::Float64MultiArray>("planning/local_speed_limit",rclcpp::QoS(8).reliable().transient_local());
    local_decision_pub_=node_->create_publisher<std_msgs::msg::String>("planning/dynamic_local_decision",10);
    dynamic_sub_=node_->create_subscription<scan_planner_msgs::msg::DynamicObstacleArray>(
      "/dynamic_obstacles/tracks",rclcpp::QoS(1),[this](scan_planner_msgs::msg::DynamicObstacleArray::ConstSharedPtr msg) {
        std::vector<ObstacleSample> samples;
        for(const auto& o:msg->obstacles) {
          ObstacleSample s; s.id=o.id;s.position={o.position.x,o.position.y};s.velocity={o.velocity.x,o.velocity.y};
          s.radius=o.radius;s.observed=o.observed;s.unknown=o.state==scan_planner_msgs::msg::DynamicObstacle::UNKNOWN;s.last_observed=rclcpp::Time(o.last_observed).seconds();
          for(int i=0;i<4;++i) for(int j=0;j<4;++j) s.covariance(i,j)=o.covariance[i*4+j];
          samples.push_back(s);
        }
        dynamic_tracks_.accept(rclcpp::Time(msg->header.stamp).seconds(),msg->epoch,
          msg->healthy && msg->header.frame_id=="world",samples,node_->now().seconds(),
          node_->get_parameter("manager.execution_radius").as_double()+
            std::abs(node_->get_parameter("manager.execution_offset").as_double()),node_->get_parameter("manager.dynamic_safety_margin").as_double());
      });
    pp_.max_vel_ = get_double("manager.max_vel", -1.0);
    pp_.max_acc_ = get_double("manager.max_acc", -1.0);
    pp_.max_jerk_ = get_double("manager.max_jerk", -1.0);
    pp_.vel_tolerance_ = get_double("optimization.vel_tolerance", 1.0);
    pp_.acc_tolerance_ = get_double("optimization.acc_tolerance", 1.0);
    pp_.feasibility_tolerance_ = get_double("manager.feasibility_tolerance", 0.0);
    pp_.ctrl_pt_dist = get_double("manager.control_points_distance", -1.0);
    pp_.planning_horizon_ = get_double("manager.planning_horizon", 5.0);

    const double tracking_margin=get_double("manager.static_tracking_margin",0.0);
    if(wheel_grid_trajectory_ && dynamic_spatial_weight_<=0. && tracking_margin>0. &&
       node_->get_parameter("manager.shortcut_wheel_path").as_bool()) {
      const double physical_radius=node_->has_parameter("manager.execution_radius")?
          node_->get_parameter("manager.execution_radius").as_double():.30;
      const double physical_offset=node_->has_parameter("manager.execution_offset")?
          node_->get_parameter("manager.execution_offset").as_double():.18;
      const double radius=get_double("grid_map.double_cylinder_radius",.25);
      const double offset=get_double("grid_map.double_cylinder_offset",.18);
      const double required=physical_radius+std::abs(physical_offset)+tracking_margin;
      if(radius+std::abs(offset)<required) {
        node_->set_parameters({rclcpp::Parameter("grid_map.double_cylinder_radius",required),
                               rclcpp::Parameter("grid_map.double_cylinder_offset",0.0)});
        RCLCPP_INFO(node_->get_logger(),"Static tracking search radius %.3f m",required);
      }
    }
    local_data_.traj_id_ = 0;
    grid_map_.reset(new GridMap);
    grid_map_->initMap(node_);

    bspline_optimizer_rebound_.reset(new BsplineOptimizer);
    bspline_optimizer_rebound_->setParam(node_);
    bspline_optimizer_rebound_->setEnvironment(grid_map_);
    bspline_optimizer_rebound_->a_star_.reset(new AStar);
    bspline_optimizer_rebound_->a_star_->initGridMap(grid_map_, wheel_grid_trajectory_ ? Eigen::Vector3i(200, 200, 20) : Eigen::Vector3i(100, 100, 100));

    visualization_ = vis;
  }

  // !SECTION

  // SECTION rebond replanning

  bool SCANPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj)
  {

    if (wheel_grid_trajectory_) return planWheelTrajectory(start_pt, local_target_pt);
    static int count = 0;
    std::cout << endl
              << "[rebo replan]: -------------------------------------" << count++ << std::endl;
    cout.precision(3);
    cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << "\ngoal:" << local_target_pt.transpose() << ", " << local_target_vel.transpose()
         << endl;

    if ((start_pt - local_target_pt).norm() < 0.2)
    {
      cout << "Close to goal" << endl;
      continuous_failures_count_++;
      return false;
    }

    auto t_start = std::chrono::steady_clock::now();
    double t_init = 0.0, t_opt = 0.0, t_refine = 0.0;

    /*** STEP 1: INIT ***/
    double ts = (start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.2 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
    vector<Eigen::Vector3d> point_set, start_end_derivatives;
    static bool flag_first_call = true, flag_force_polynomial = false;
    bool flag_regenerate = false;
    do
    {
      point_set.clear();
      start_end_derivatives.clear();
      flag_regenerate = false;

      if (flag_first_call || flag_polyInit || flag_force_polynomial /*|| ( start_pt - local_target_pt ).norm() < 1.0*/) // Initial path generated from a min-snap traj by order.
      {
        flag_first_call = false;
        flag_force_polynomial = false;

        PolynomialTraj gl_traj;

        double dist = (start_pt - local_target_pt).norm();
        double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist ? sqrt(dist / pp_.max_acc_) : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;

        if (!flag_randomPolyTraj)
        {
          gl_traj = PolynomialTraj::one_segment_traj_gen(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), time);
        }
        else
        {
          Eigen::Vector3d horizon_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
          Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizon_dir)).normalized();
          Eigen::Vector3d random_inserted_pt = (start_pt + local_target_pt) / 2 +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * horizon_dir * 0.8 * (-0.978 / (continuous_failures_count_ + 0.989) + 0.989) +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * vertical_dir * 0.4 * (-0.978 / (continuous_failures_count_ + 0.989) + 0.989);
          Eigen::MatrixXd pos(3, 3);
          pos.col(0) = start_pt;
          pos.col(1) = random_inserted_pt;
          pos.col(2) = local_target_pt;
          Eigen::VectorXd t(2);
          t(0) = t(1) = time / 2;
          gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), t);
        }

        double t;
        bool flag_too_far;
        ts *= 1.5; // ts will be divided by 1.5 in the next
        do
        {
          ts /= 1.5;
          point_set.clear();
          flag_too_far = false;
          Eigen::Vector3d last_pt = gl_traj.evaluate(0);
          for (t = 0; t < time; t += ts)
          {
            Eigen::Vector3d pt = gl_traj.evaluate(t);
            if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5)
            {
              flag_too_far = true;
              break;
            }
            last_pt = pt;
            point_set.push_back(pt);
          }
        } while (flag_too_far || point_set.size() < 7); // To make sure the initial path has enough points.
        t -= ts;
        start_end_derivatives.push_back(gl_traj.evaluateVel(0));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
        start_end_derivatives.push_back(gl_traj.evaluateAcc(t));
      }
      else // Initial path generated from previous trajectory.
      {

        double t;
        double t_cur = (node_->now() - local_data_.start_time_).seconds();

        vector<double> pseudo_arc_length;
        vector<Eigen::Vector3d> segment_point;
        pseudo_arc_length.push_back(0.0);
        for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)
        {
          segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t));
          if (t > t_cur)
          {
            pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
          }
        }
        t -= ts;

        double poly_time = (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt).norm() / pp_.max_vel_ * 2;
        if (poly_time > ts)
        {
          PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(local_data_.position_traj_.evaluateDeBoorT(t),
                                                                        local_data_.velocity_traj_.evaluateDeBoorT(t),
                                                                        local_data_.acceleration_traj_.evaluateDeBoorT(t),
                                                                        local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);

          for (t = ts; t < poly_time; t += ts)
          {
            if (!pseudo_arc_length.empty())
            {
              segment_point.push_back(gl_traj.evaluate(t));
              pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
            }
            else
            {
              RCLCPP_ERROR(node_->get_logger(), "pseudo_arc_length is empty; aborting replan");
              continuous_failures_count_++;
              return false;
            }
          }
        }

        double sample_length = 0;
        double cps_dist = pp_.ctrl_pt_dist * 1.5; // cps_dist will be divided by 1.5 in the next
        size_t id = 0;
        do
        {
          cps_dist /= 1.5;
          point_set.clear();
          sample_length = 0;
          id = 0;
          while ((id <= pseudo_arc_length.size() - 2) && sample_length <= pseudo_arc_length.back())
          {
            if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1])
            {
              point_set.push_back((sample_length - pseudo_arc_length[id]) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id + 1] +
                                  (pseudo_arc_length[id + 1] - sample_length) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id]);
              sample_length += cps_dist;
            }
            else
              id++;
          }
          point_set.push_back(local_target_pt);
        } while (point_set.size() < 7); // If the start point is very close to end point, this will help

        start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(local_data_.acceleration_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());

        if (point_set.size() > pp_.planning_horizon_ / pp_.ctrl_pt_dist * 3) // The initial path is abnormally too long!
        {
          flag_force_polynomial = true;
          flag_regenerate = true;
        }
      }
    } while (flag_regenerate);

    const auto get_height_param = [this](const std::string &name, double default_value) {
      return node_->has_parameter(name) ? node_->get_parameter(name).as_double()
                                        : default_value;
    };
    const bool allow_unverified_clearance =
        node_->has_parameter("grid_map.allow_unverified_ceiling_clearance") &&
        node_->get_parameter("grid_map.allow_unverified_ceiling_clearance").as_bool();
    applyCeilingHeightReference(
        point_set, start_pt(2), local_target_pt(2), grid_map_,
        get_height_param("optimization.height_min", 0.17),
        get_height_param("optimization.height_max", 0.40),
        get_height_param("optimization.body_top_offset", 0.328),
        get_height_param("optimization.ceiling_margin", 0.03),
        allow_unverified_clearance);

    Eigen::MatrixXd ctrl_pts;
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

    vector<vector<Eigen::Vector3d>> a_star_paths;
    a_star_paths = bspline_optimizer_rebound_->initControlPoints(ctrl_pts, true);

    t_init = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();

    static int vis_id = 0;
    visualization_->displayInitPathList(point_set, 0.2, 0);
    visualization_->displayAStarList(a_star_paths, vis_id);

    t_start = std::chrono::steady_clock::now();

    /*** STEP 2: OPTIMIZE ***/
    bool flag_step_1_success = bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
    cout << "first_optimize_step_success=" << flag_step_1_success << endl;
    if (!flag_step_1_success)
    {
      // visualization_->displayOptimalList( ctrl_pts, vis_id );
      continuous_failures_count_++;
      return false;
    }
    //visualization_->displayOptimalList( ctrl_pts, vis_id );

    t_opt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
    t_start = std::chrono::steady_clock::now();

    /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
    UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
    pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

    double ratio;
    bool flag_step_2_success = true;
    if (!pos.checkFeasibility(ratio, false))
    {
      cout << "Need to reallocate time." << endl;

      Eigen::MatrixXd optimal_control_points;
      flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
      if (flag_step_2_success)
        pos = UniformBspline(optimal_control_points, 3, ts);
    }

    if (!flag_step_2_success || !checkDynamicFeasibility(pos))
    {
      printf("\033[34mThis refined trajectory is unsafe or dynamically infeasible. Skip publishing it.\n\033[0m");
      continuous_failures_count_++;
      return false;
    }

    t_refine = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();

    // save planned results
    updateTrajInfo(pos, node_->now());

    cout << "total time:\033[42m" << (t_init + t_opt + t_refine)
         << "\033[0m,optimize:" << (t_init + t_opt) << ",refine:" << t_refine << endl;

    // success. YoY
    continuous_failures_count_ = 0;
    return true;
  }


  // Wheel mode uses a collision-checked grid corridor rather than an unconstrained
  // XY optimizer whose ceiling term has no XY gradient at doorway discontinuities.
  bool SCANPlannerManager::trajectoryCollision(const Eigen::Vector3d &p, double yaw)
  {
    if(dynamic_tracks_.fresh(node_->now().seconds()))
      for(const auto& [id,track]:dynamic_tracks_.bank.tracks)
        if(track.unknown && track.predict(0,"cv").clearance(p.head<2>(),track.collision_radius)<0) return true;
    if (!wheel_grid_trajectory_) return grid_map_->getInflateOccupancyAdaptive(p,yaw)!=0;
    const double ceiling=grid_map_->queryCeilingHeight(p,yaw,nullptr,
        node_->get_parameter("manager.execution_radius").as_double(),
        node_->get_parameter("manager.execution_offset").as_double());
    return !grid_map_->isInMap(p) ||
           (node_->get_parameter("grid_map.world_bound_enable").as_bool() && !grid_map_->isInWorldBound(p)) ||
           p.z()<node_->get_parameter("manager.execution_height_min").as_double()-1e-6 ||
           p.z()>node_->get_parameter("manager.execution_height_max").as_double()+1e-6 ||
           (std::isfinite(ceiling) && p.z()+0.328+0.05>ceiling+1e-6);
  }

  bool SCANPlannerManager::planWheelTrajectory(const Eigen::Vector3d &start,
                                               const Eigen::Vector3d &goal)
  {
    struct Profile {
      rclcpp::Logger logger;
      std::chrono::steady_clock::time_point begin=std::chrono::steady_clock::now();
      ~Profile(){RCLCPP_INFO(logger,"WHEEL_PLAN_MS %.3f",
        std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count());}
    } profile{node_->get_logger()};
    auto search = bspline_optimizer_rebound_->a_star_;
    const double height_anchor=node_->get_parameter("manager.height_lattice_anchor").as_double();
    search->height_lattice_anchor=height_anchor>=0. ? height_anchor : std::numeric_limits<double>::quiet_NaN();
    // Always obtain a hard-safe geometry seed. Spatial FRS is a preference,
    // so exhausting its refinement budget must not erase a feasible corridor.
    const double configured_heuristic=node_->get_parameter("manager.wheel_search_heuristic").as_double();
    search->heuristic_weight=configured_heuristic>0. ? configured_heuristic : 1.8;
    const auto views=dynamic_tracks_.fresh(node_->now().seconds()) ?
      dynamic_tracks_.bank.views(node_->now().seconds()) : std::vector<TrackView>{};
    const auto hard_cost=[views](const Eigen::Vector3d& p) {
      for(const auto& v:views) if(v.track->unknown &&
          v.track->predict(0,"cv").clearance(p.head<2>(),v.track->collision_radius)<0)
        return std::numeric_limits<double>::infinity();
      return 0.;
    };
    search->soft_cost=hard_cost;
    if(search->AstarSearch(grid_map_->getResolution(),start,goal,wheel_search_timeout_)!=ASTAR_RET::SUCCESS)
      return false;
    auto path=search->getPath();
    const double remaining=wheel_search_timeout_-std::chrono::duration<double>(
      std::chrono::steady_clock::now()-profile.begin).count();
    if(dynamic_spatial_weight_>0 && remaining>.005 &&
       std::any_of(views.begin(),views.end(),[](const auto& v){return !v.track->unknown;})) {
      search->soft_cost=[views,this,hard_cost](const Eigen::Vector3d& p) {
        if(!std::isfinite(hard_cost(p))) return std::numeric_limits<double>::infinity();
        double risk=0.;
        for(const auto& v:views) if(!v.track->unknown) for(double t:{0.,.5,1.,1.5})
          risk=std::max(risk,frsRisk(*v.track,v.age,t,p.head<2>(),"frs"));
        return dynamic_spatial_weight_*risk;
      };
      if(search->AstarSearch(grid_map_->getResolution(),start,goal,remaining)==ASTAR_RET::SUCCESS)
        path=search->getPath();
      else RCLCPP_INFO(node_->get_logger(),"WHEEL_RISK_BUDGET: retain hard-safe geometry seed");
    }
    if (path.empty()) return false;
    if ((path.front() - start).norm() > (path.back() - start).norm())
      std::reverse(path.begin(), path.end());
    path.insert(path.begin(), start);
    path.push_back(goal);
    const auto raw_path=path;
    bool planar=true;
    for(const auto &p:path) if(std::abs(p.z()-start.z())>.005) planar=false;
    if(node_->get_parameter("manager.shortcut_wheel_path").as_bool() &&
       canReuseStaticCorridor() && planar) {
      path=shortcutWheelPath(path,grid_map_->getResolution()*.4,
        [this](const Eigen::Vector3d &p,double yaw){return grid_map_->getInflateOccupancyAdaptive(p,yaw)==0;});
    }
    UniformBspline spline;
    DynamicPathResult dynamic_choice;
    bool dynamic_selected=false;
    double planned_speed=pp_.max_vel_;
    std::string dynamic_reason="disabled";
    const auto selection_begin=std::chrono::steady_clock::now();
    if(dynamic_local_enabled_ && planar && !joint_rollout_enabled_) {
      planned_speed=0.;dynamic_reason="stale_inputs";
      const double now=node_->now().seconds();
      if(dynamic_tracks_.fresh(now) && execution_stamp_>=0 && now-execution_stamp_<.5 && now>=execution_stamp_-.05) {
        if(views.empty()) {planned_speed=pp_.max_vel_;dynamic_choice.feasible=true;dynamic_reason="empty_scene";}
        else {
          std::vector<Eigen::Vector3d> guide;
          const double duration=global_data_.global_duration_;
          if(std::isfinite(duration) && duration>0.)
            for(double t=0;t<=duration;t+=std::max(.1,duration/160.))guide.push_back(global_data_.getPosition(t));
          if(guide.empty())guide=path;
          dynamic_local_.config.speed=pp_.max_vel_;
          dynamic_local_.config.acceleration=pp_.max_acc_;
          dynamic_choice=dynamic_local_.choose(path,start,execution_yaw_,execution_speed_,execution_yaw_rate_,views,guide,
            [this](const Eigen::Vector3d& p,double yaw){return !trajectoryCollision(p,yaw);});
          if(dynamic_choice.feasible) {
            dynamic_selected=true;path=dynamic_choice.path;spline=dynamic_choice.spline;
            planned_speed=dynamic_choice.speed;dynamic_reason=planned_speed>0.?"moving_candidate":"checked_wait";
          } else dynamic_reason="no_safe_candidate";
        }
      }
    } else if(dynamic_local_enabled_)dynamic_reason="height_reference_preserved";
    const double selection_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-selection_begin).count();
    if(!dynamic_selected) {
    if (node_->get_parameter("manager.local_wheel_timing").as_bool()) {
      if (!makeLocallyTimedWheelSpline(path,pp_.max_vel_,pp_.max_acc_,0.06,spline)) {
        RCLCPP_WARN(node_->get_logger(), "Wheel time allocation failed feasibility bounds");
        return false;
      }
    } else {
    const double dt = 0.20;
    const double speed = std::min(pp_.max_vel_, 0.30);
    const double height_rate = 0.06;
    std::vector<Eigen::Vector3d> points(3, start);
    for (size_t i = 1; i < path.size(); ++i) {
      const Eigen::Vector3d delta = path[i] - path[i-1];
      const double duration = std::max(delta.head<2>().norm() / speed,
                                       std::abs(delta.z()) / height_rate);
      const int steps = std::max(1, static_cast<int>(std::ceil(duration / dt)));
      for (int k = 1; k <= steps; ++k)
        points.push_back(path[i-1] + delta * (static_cast<double>(k) / steps));
    }
    points.push_back(goal); points.push_back(goal);
    Eigen::MatrixXd controls(3, points.size());
    for (size_t i=0; i<points.size(); ++i) controls.col(i)=points[i];
    spline=UniformBspline(controls, 3, dt);
    spline.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, 0.0);
    double ratio=1.0;
    if (!spline.checkFeasibility(ratio, false)) spline.lengthenTime(ratio * 1.05);
    }
    }
    if(!dynamic_selected && path.size()<raw_path.size() && node_->get_parameter("manager.local_wheel_timing").as_bool()) {
      bool safe=true;
      for(double t=0;t<=spline.getTimeSum();t+=.02) {
        const Eigen::Vector3d p=spline.evaluateDeBoorT(t);
        const Eigen::Vector3d v=spline.getDerivative().evaluateDeBoorT(t);
        if(trajectoryCollision(p,std::atan2(v.y(),v.x()))) {safe=false;break;}
      }
      if(!safe) {
        path=raw_path;
        if(!makeLocallyTimedWheelSpline(path,pp_.max_vel_,pp_.max_acc_,.06,spline)) return false;
      }
    }
    auto certified=[&](UniformBspline& curve) {
      const double duration=curve.getTimeSum();
      for(double t=0;t<=duration;t+=.02) {
        const Eigen::Vector3d p=curve.evaluateDeBoorT(t);
        const Eigen::Vector3d d=curve.evaluateDeBoorT(std::min(duration,t+.05))-p;
        const double yaw=d.head<2>().norm()>1e-6 ? std::atan2(d.y(),d.x()) : 0.;
        if(trajectoryCollision(p,yaw)) {
          RCLCPP_WARN_THROTTLE(node_->get_logger(),*node_->get_clock(),2000,
            "WHEEL_CURVE_REJECT xyz=%.4f %.4f %.4f ceiling=%.4f",p.x(),p.y(),p.z(),
            grid_map_->queryCeilingHeight(p,yaw,nullptr,node_->get_parameter("manager.execution_radius").as_double(),node_->get_parameter("manager.execution_offset").as_double()));
          return false;
        }
      }
      return true;
    };
    if(!certified(spline)) {
      if(dynamic_selected)return false;
      // Search waypoints are safe but a smoothed height transition can cut a
      // narrow ceiling corner. Descend at the current pose, traverse low, then
      // restore at the goal. All segments and the final curve are checked.
      const double low=node_->get_parameter("manager.execution_height_min").as_double();
      std::vector<Eigen::Vector3d> fallback{start,Eigen::Vector3d(start.x(),start.y(),low)};
      for(const auto& p:raw_path) fallback.emplace_back(p.x(),p.y(),low);
      fallback.emplace_back(goal.x(),goal.y(),low);fallback.push_back(goal);
      auto free=[this](const Eigen::Vector3d& p,double yaw){return !trajectoryCollision(p,yaw);};
      fallback=shortcutWheelPath(fallback,grid_map_->getResolution()*.4,free);
      // Adjacent raw segments also need certification; a failed shortcut does
      // not mean its minimum-height fallback is traversable.
      for(size_t i=1;i<fallback.size();++i) {
        const Eigen::Vector3d delta=fallback[i]-fallback[i-1];
        const int n=std::max(1,int(std::ceil(delta.norm()/(grid_map_->getResolution()*.4))));
        const double yaw=std::atan2(delta.y(),delta.x());
        for(int k=0;k<=n;++k) if(!free(fallback[i-1]+delta*(double(k)/n),yaw)) {
          const Eigen::Vector3d bad=fallback[i-1]+delta*(double(k)/n);
          RCLCPP_WARN_THROTTLE(node_->get_logger(),*node_->get_clock(),2000,
            "WHEEL_FALLBACK_REJECT xyz=%.4f %.4f %.4f ceiling=%.4f",bad.x(),bad.y(),bad.z(),
            grid_map_->queryCeilingHeight(bad,yaw,nullptr,node_->get_parameter("manager.execution_radius").as_double(),node_->get_parameter("manager.execution_offset").as_double()));
          return false;
        }
      }
      if(!makeCornerPreservingWheelSpline(fallback,pp_.max_vel_,pp_.max_acc_,.06,spline) ||
         !certified(spline)) return false;
      path=std::move(fallback);
      RCLCPP_INFO(node_->get_logger(),"WHEEL_HEIGHT_FALLBACK: certified corner-preserving corridor");
    }
    JointRolloutResult joint_choice;
    JointRolloutPacket joint_packet; joint_packet.stamp=node_->now().seconds();
    if(joint_rollout_enabled_){
      planned_speed=pp_.max_vel_;
      const bool fresh=dynamic_tracks_.fresh(joint_packet.stamp) && execution_stamp_>=0 &&
        joint_packet.stamp-execution_stamp_<.35 && joint_packet.stamp>=execution_stamp_-.05;
      joint_packet.active=planar && (!fresh || !views.empty());
      if(joint_packet.active && fresh){
        joint_rollout_.config.speed=pp_.max_vel_;joint_rollout_.config.acceleration=pp_.max_acc_;
        joint_rollout_.config.yaw_rate=dynamic_local_.config.yaw_rate;
        joint_rollout_.config.braking=dynamic_local_.config.braking;
        const double radius=node_->get_parameter("manager.execution_radius").as_double();
        const double offset=node_->get_parameter("manager.execution_offset").as_double();
        const double low=node_->get_parameter("manager.execution_height_min").as_double();
        const double high=node_->get_parameter("manager.execution_height_max").as_double();
        const bool world_bound=node_->get_parameter("grid_map.world_bound_enable").as_bool();
        const double resolution=grid_map_->getResolution(),tile=std::max(.08,resolution);
        std::map<std::pair<int,int>,bool> certificates;
        // Cache only conservative FREE certificates. An enlarged circle covers
        // both cylinders, every pose/yaw in a tile, and voxel sampling error.
        // A failed certificate falls back to the exact original body query.
        auto free=[&](const Eigen::Vector3d& p,double yaw){
          if(!grid_map_->isInMap(p) || (world_bound && !grid_map_->isInWorldBound(p)) || p.z()<low-1e-6 || p.z()>high+1e-6)return false;
          const std::pair<int,int> key{int(std::floor(p.x()/tile)),int(std::floor(p.y()/tile))};
          auto found=certificates.find(key);
          if(found==certificates.end()){
            Eigen::Vector3d q((key.first+.5)*tile,(key.second+.5)*tile,p.z());
            const double enlarged=radius+std::abs(offset)+std::sqrt(.5)*tile+std::sqrt(2.)*resolution;
            const double ceiling=grid_map_->queryCeilingHeight(q,0.,nullptr,enlarged,0.);
            bool certified=!std::isfinite(ceiling) || q.z()+.328+.05<=ceiling+1e-6;
            for(double x:{-.5,.5})for(double y:{-.5,.5}){
              Eigen::Vector3d corner=q+Eigen::Vector3d(x*tile,y*tile,0.);
              certified=certified && grid_map_->isInMap(corner) && (!world_bound || grid_map_->isInWorldBound(corner));
            }
            found=certificates.emplace(key,certified).first;
          }
          if(found->second)return true;
          const double ceiling=grid_map_->queryCeilingHeight(p,yaw,nullptr,radius,offset);
          return !std::isfinite(ceiling) || p.z()+.328+.05<=ceiling+1e-6;
        };
        joint_choice=joint_rollout_.choose(execution_pose_,execution_yaw_,execution_speed_,execution_yaw_rate_,
          path.back(),path,views,free,joint_packet.stamp);
        joint_packet.controls=joint_choice.controls;joint_packet.feasible=joint_choice.feasible;
      } else joint_packet.feasible=!joint_packet.active;
      dynamic_reason=!planar?"height_reference_preserved":!joint_packet.active?"empty_scene":!fresh?"stale_inputs":
        joint_choice.feasible?(joint_choice.parked_threat?"joint_escape":"joint_tracking"):"joint_no_safe_sequence";
    }
    updateTrajInfo(spline, node_->now());
    if(joint_rollout_enabled_){
      joint_packet.id=local_data_.traj_id_;joint_packet.step=joint_rollout_.config.step;
      joint_id_=joint_packet.id;joint_active_=joint_packet.active;joint_stamp_=joint_packet.stamp;
      joint_path_=joint_choice.feasible?joint_choice.path:std::vector<Eigen::Vector3d>{};
      std_msgs::msg::Float64MultiArray msg;msg.data=joint_packet.encode();joint_rollout_pub_->publish(msg);
    }
    if(dynamic_local_enabled_) {
      last_local_selection_=node_->now().seconds();last_local_speed_=planned_speed;
      std_msgs::msg::Float64MultiArray limit;
      limit.data={static_cast<double>(local_data_.traj_id_),node_->now().seconds(),planned_speed};local_speed_pub_->publish(limit);
      std::ostringstream log;log.precision(17);
      log<<"{\"stamp\":"<<node_->now().seconds()<<",\"traj_id\":"<<local_data_.traj_id_
         <<",\"enabled\":true,\"feasible\":"<<((joint_rollout_enabled_?joint_packet.feasible:dynamic_choice.feasible)?"true":"false")
         <<",\"reason\":\""<<dynamic_reason<<"\",\"speed\":"<<(joint_rollout_enabled_ && joint_packet.active?(joint_choice.controls.empty()?0.:joint_choice.controls.front().v):planned_speed)
         <<",\"offset\":"<<dynamic_choice.offset<<",\"candidates\":"<<(joint_rollout_enabled_?joint_choice.candidates:dynamic_choice.candidates)
         <<",\"risk\":"<<(joint_rollout_enabled_?joint_choice.risk:dynamic_choice.risk)<<",\"clearance\":"<<(joint_rollout_enabled_?joint_choice.clearance:dynamic_choice.clearance)
         <<",\"selection_ms\":"<<(joint_rollout_enabled_?std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-selection_begin).count():selection_ms)
         <<",\"joint_rollout\":"<<(joint_rollout_enabled_?"true":"false")
         <<",\"joint_active\":"<<(joint_packet.active?"true":"false")<<",\"joint_input_stamp\":"<<joint_packet.stamp
         <<",\"joint_feasible\":"<<(joint_choice.feasible?"true":"false")
         <<",\"parked_threat\":"<<(joint_choice.parked_threat?"true":"false")
         <<",\"joint_candidates\":"<<joint_choice.candidates<<",\"joint_risk\":"<<joint_choice.risk
         <<",\"joint_clearance\":"<<joint_choice.clearance<<",\"controls\":[";
      for(size_t i=0;i<joint_choice.controls.size();++i){if(i)log<<",";log<<"["<<joint_choice.controls[i].v<<","<<joint_choice.controls[i].w<<"]";}
      log<<"],\"path\":[";
      const auto& logged_path=joint_rollout_enabled_&&!joint_choice.path.empty()?joint_choice.path:path;
      for(size_t i=0;i<logged_path.size();++i){if(i)log<<",";log<<"["<<logged_path[i].x()<<","<<logged_path[i].y()<<"]";}
      log<<"]}";std_msgs::msg::String message;message.data=log.str();local_decision_pub_->publish(message);
    }
    visualization_->displayInitPathList(path, 0.08, 0);
    continuous_failures_count_=0;
    return true;
  }

  bool SCANPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    Eigen::MatrixXd control_points(3, 6);
    for (int i = 0; i < 6; i++)
    {
      control_points.col(i) = stop_pos;
    }

    updateTrajInfo(UniformBspline(control_points, 3, 1.0), node_->now());

    return true;
  }

  bool SCANPlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                  const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    if (waypoints.empty())
      return false;

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);

    for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
    {
      points.push_back(waypoints[wp_i]);
    }

    double total_len = 0;
    for (size_t i = 0; i < points.size() - 1; i++)
    {
      total_len += (points[i + 1] - points[i]).norm();
    }

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    double dist_thresh = max(total_len / 8, 4.0);

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // for ( int i=0; i<inter_points.size(); i++ )
    // {
    //   cout << inter_points[i].transpose() << endl;
    // }

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, pos.col(1), end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = node_->now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool SCANPlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);
    points.push_back(end_pos);

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    const double dist_thresh = 4.0;

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = node_->now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool SCANPlannerManager::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
  {
    double t_inc;

    Eigen::MatrixXd ctrl_pts; // = traj.getControlPoint()

    // std::cout << "ratio: " << ratio << std::endl;
    reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

    traj = UniformBspline(ctrl_pts, 3, ts);

    double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
    bspline_optimizer_rebound_->ref_pts_.clear();
    for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
      bspline_optimizer_rebound_->ref_pts_.push_back(traj.evaluateDeBoorT(t));

    bool success = bspline_optimizer_rebound_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

    return success;
  }

  void SCANPlannerManager::updateTrajInfo(const UniformBspline &position_traj, const rclcpp::Time time_now)
  {
    local_data_.start_time_ = time_now;
    local_data_.position_traj_ = position_traj;
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();
    local_data_.traj_id_ += 1;
  }

  bool SCANPlannerManager::checkDynamicFeasibility(UniformBspline position_traj)
  {
    UniformBspline vel_traj = position_traj.getDerivative();
    UniformBspline acc_traj = vel_traj.getDerivative();
    const double duration = position_traj.getTimeSum();
    const double sample_dt = std::max(0.01, std::min(0.05, duration / 50.0));
    const double vel_limit = pp_.max_vel_ + pp_.vel_tolerance_;
    const double acc_limit = pp_.max_acc_ + pp_.acc_tolerance_;

    for (double t = 0.0; t < duration + 1e-6; t += sample_dt)
    {
      const double tc = std::min(t, duration);
      Eigen::Vector3d vel = vel_traj.evaluateDeBoorT(tc);
      if (vel.norm() > vel_limit)
      {
        RCLCPP_WARN(node_->get_logger(),
                    "Dynamic feasibility failed: velocity at t=%.3f is %.3f > %.3f",
                    tc, vel.norm(), vel_limit);
        return false;
      }

      Eigen::Vector3d acc = acc_traj.evaluateDeBoorT(tc);
      if (acc.norm() > acc_limit)
      {
        RCLCPP_WARN(node_->get_logger(),
                    "Dynamic feasibility failed: acceleration at t=%.3f is %.3f > %.3f",
                    tc, acc.norm(), acc_limit);
        return false;
      }
    }

    return true;
  }

  void SCANPlannerManager::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                         Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
  {
    double time_origin = bspline.getTimeSum();
    int seg_num = bspline.getControlPoint().cols() - 3;
    // double length = bspline.getLength(0.1);
    // int seg_num = ceil(length / pp_.ctrl_pt_dist);

    bspline.lengthenTime(ratio);
    double duration = bspline.getTimeSum();
    dt = duration / double(seg_num);
    time_inc = duration - time_origin;

    vector<Eigen::Vector3d> point_set;
    for (double time = 0.0; time <= duration + 1e-4; time += dt)
    {
      point_set.push_back(bspline.evaluateDeBoorT(time));
    }
    UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  }

} // namespace scan_planner
