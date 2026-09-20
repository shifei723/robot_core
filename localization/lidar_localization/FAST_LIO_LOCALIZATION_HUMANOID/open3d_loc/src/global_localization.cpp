#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32.hpp>

#include <tf2_eigen/tf2_eigen.hpp>
#include <queue>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <open3d/Open3D.h>

#include "open3d_registration/open3d_registration.h"
#include "open3d_conversions/open3d_conversions.h"

#define PI 3.1415926

class KalmanFilter
{
public:
    KalmanFilter() : processVar_(0.0), estimatedMeasVar_(0.0),
                     posteriEstimate_(0.0), posteriErrorEstimate_(1.0)
    {
    }

    void KalmanFilterInit(double processVar, double estimatedMeasVar, double posteriEstimate = 0.0, double posteriErrorEstimate = 1.0)
    {
        processVar_ = processVar;
        estimatedMeasVar_ = estimatedMeasVar;
        posteriEstimate_ = posteriEstimate;
        posteriErrorEstimate_ = posteriErrorEstimate;
    }
    void inputLatestNoisyMeasurement(double measurement)
    {
        double prioriEstimate = posteriEstimate_;
        double prioriErrorEstimate = posteriErrorEstimate_ + processVar_;

        double denominator = prioriErrorEstimate + estimatedMeasVar_;

        if (std::abs(denominator) < 1e-10)
        {
            posteriEstimate_ = measurement;
            posteriErrorEstimate_ = 1.0;
            return;
        }

        double blendingFactor = prioriErrorEstimate / denominator;
        posteriEstimate_ = prioriEstimate + blendingFactor * (measurement - prioriEstimate);
        posteriErrorEstimate_ = (1 - blendingFactor) * prioriErrorEstimate;
    }

    double getLatestEstimatedMeasurement()
    {
        return posteriEstimate_;
    }

private:
    double processVar_;
    double estimatedMeasVar_;
    double posteriEstimate_;
    double posteriErrorEstimate_;
};

class GloabalLocalization : public rclcpp::Node
{
private:
    /* data */
public:
    GloabalLocalization();
    ~GloabalLocalization();

    void LocalizationInitialize();

    void CallbackBaseFootprint2Odom(const nav_msgs::msg::Odometry::SharedPtr base_footprint2odom);
    void CallbackScan(const sensor_msgs::msg::PointCloud2::SharedPtr scan_in_base_footprint);

    void CallbackInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr initialpose);

    void StartLoc();

    void Localization();

    Eigen::Matrix3d Euler2Matrix3d(const Eigen::Vector3d euler);

    bool GetTfTransformToMatrix(
        std::string frame_id, std::string child_frame_id, Eigen::Matrix4d &matrix);

    double ComputeMotionDis(const Eigen::Vector3d &a, const Eigen::Vector3d &b);

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_base_footprint2odom_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_scan_cur_;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initialpose_;

    nav_msgs::msg::Odometry pose_base_footprint2odom_;

    Eigen::Matrix4d mat_base_footprint2odom_;
    Eigen::Matrix4d mat_odom2map_;
    Eigen::Matrix4d mat_odom2map_kalman_;
    Eigen::Matrix4d mat_base_footprint2map_;
    Eigen::Matrix4d mat_initialpose_;

    std::mutex lock_mat_odom2map_;

    Eigen::Matrix4d mat_imulink2base_footprint_;

    std::vector<double> initialpose_;

    std::shared_ptr<open3d::geometry::PointCloud> pcd_map_ori_;
    std::shared_ptr<open3d::geometry::PointCloud> pcd_map_coarse_;
    std::shared_ptr<open3d::geometry::PointCloud> pcd_map_fine_;
    std::shared_ptr<open3d::geometry::PointCloud> pcd_map_cur_;
    std::shared_ptr<open3d::geometry::PointCloud> pcd_scan_cur_;

    std::queue<open3d::geometry::PointCloud> que_pcd_scan_;
    int queue_maxsize_;
    double voxelsize_coarse_;
    double voxelsize_fine_;

    double threshold_fitness_;
    double threshold_fitness_init_;

    std::thread thread_loc_;
    std::mutex lock_scan_;
    std::mutex lock_exit_;
    bool flag_exit_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_base_footprint2map_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_base_footprint2map_kalman_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom2map_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom2map_kalman_;
    rclcpp::Time timestamp_odom_;
    std::mutex lock_timestamp_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_scan_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_scan2map_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_submap_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_localization_3d_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_localization_3d_confidence_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_localization_3d_delay_ms_;

    geometry_msgs::msg::PoseStamped localization_3d_;
    std_msgs::msg::Float32 localization_3d_confidence_;
    std_msgs::msg::Float32 localization_3d_delay_ms_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> br_odom2map_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> br_odom2base_footprint_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

    bool save_scan_;

    double loc_frequence_;

    int maxpoints_source_ = 50000;
    int maxpoints_target_ = 200000;

    bool loc_initialized_ = false;

    double loc_fitness_;

    double confidence_loc_th_;

    KalmanFilter kf_base_footprint_x_;
    KalmanFilter kf_base_footprint_y_;
    KalmanFilter kf_base_footprint_z_;
    KalmanFilter kalman_filter_odom2map_;

    std::vector<double> kf_param_x_;
    std::vector<double> kf_param_y_;
    std::vector<double> kf_param_z_;

    bool filter_odom2map_ = false;
    double kalman_processVar2_ = 0.0;
    double kalman_estimatedMeasVar2_ = 0.0;

    Eigen::Vector3d last_loc_;
    double dis_updatemap_;

    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    Eigen::Matrix4d mat_fastlio_odom2base_footprint_;
};

GloabalLocalization::GloabalLocalization() : Node("global_loc_node"),
                                             tf_buffer_(this->get_clock()),
                                             tf_listener_(std::make_shared<tf2_ros::TransformListener>(tf_buffer_))
{
    flag_exit_ = false;
    loc_initialized_ = false;
    mat_base_footprint2odom_ = Eigen::Matrix4d::Identity();
    mat_odom2map_ = Eigen::Matrix4d::Identity();
    mat_initialpose_ = Eigen::Matrix4d::Identity();
    last_loc_ = Eigen::Vector3d(0, 0, -5000);
    mat_fastlio_odom2base_footprint_ = Eigen::Matrix4d::Identity();

    pcd_map_ori_.reset(new open3d::geometry::PointCloud);
    pcd_map_coarse_.reset(new open3d::geometry::PointCloud);
    pcd_map_cur_.reset(new open3d::geometry::PointCloud);
    pcd_scan_cur_.reset(new open3d::geometry::PointCloud);
    pcd_map_fine_.reset(new open3d::geometry::PointCloud);
    queue_maxsize_ = 5;

    pub_base_footprint2map_ = this->create_publisher<nav_msgs::msg::Odometry>("/base_footprint2map", 100000);
    pub_base_footprint2map_kalman_ = this->create_publisher<nav_msgs::msg::Odometry>("/base_footprint2map_kalman", 100000);
    pub_odom2map_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom2map", 100000);
    pub_odom2map_kalman_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom2map_kalman", 100000);

    pub_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map", 1);
    pub_submap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/submap", 1);
    pub_scan2map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/scan2map", 1);
    pub_scan_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/scan", 1);
    pub_localization_3d_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/localization_3d", 1);
    pub_localization_3d_confidence_ = this->create_publisher<std_msgs::msg::Float32>("/localization_3d_confidence", 1);
    pub_localization_3d_delay_ms_ = this->create_publisher<std_msgs::msg::Float32>("/localization_3d_delay_ms", 1);

    loc_frequence_ = 2.0;
    loc_fitness_ = 0.0;

    sub_base_footprint2odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/Odometry_loc", 50, std::bind(&GloabalLocalization::CallbackBaseFootprint2Odom, this, std::placeholders::_1));
    sub_scan_cur_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud_registered_1", 50, std::bind(&GloabalLocalization::CallbackScan, this, std::placeholders::_1));
    sub_initialpose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 50, std::bind(&GloabalLocalization::CallbackInitialPose, this, std::placeholders::_1));

    pose_base_footprint2odom_ = nav_msgs::msg::Odometry();
    pose_base_footprint2odom_.header.frame_id = "odom";
    pose_base_footprint2odom_.child_frame_id = "base_footprint";
    pose_base_footprint2odom_.pose.pose.orientation.w = 1;
    RCLCPP_INFO(this->get_logger(), "pose base_footprint2odom:\nx: %f, y: %f, z: %f, qx: %f, qy: %f, qz: %f, qw: %f",
                pose_base_footprint2odom_.pose.pose.position.x,
                pose_base_footprint2odom_.pose.pose.position.y,
                pose_base_footprint2odom_.pose.pose.position.z,
                pose_base_footprint2odom_.pose.pose.orientation.x,
                pose_base_footprint2odom_.pose.pose.orientation.y,
                pose_base_footprint2odom_.pose.pose.orientation.z,
                pose_base_footprint2odom_.pose.pose.orientation.w);

    this->declare_parameter<int>("pcd_queue_maxsize", 5);
    this->declare_parameter<bool>("save_scan", false);
    this->declare_parameter<int>("maxpoints_source", 50000);
    this->declare_parameter<int>("maxpoints_target", 200000);

    this->declare_parameter<double>("loc_frequence", 2.0);
    this->declare_parameter<double>("confidence_loc_th", 0.6);

    this->declare_parameter<std::vector<double>>("kf_baselink2map/x", std::vector<double>(2));
    this->declare_parameter<std::vector<double>>("kf_baselink2map/y", std::vector<double>(2));
    this->declare_parameter<std::vector<double>>("kf_baselink2map/z", std::vector<double>(2));

    this->declare_parameter<bool>("filter_odom2map", false);
    this->declare_parameter<double>("kalman_processVar2", 0.02);
    this->declare_parameter<double>("kalman_estimatedMeasVar2", 0.04);
    this->declare_parameter<double>("voxelsize_coarse", 0.2);
    this->declare_parameter<double>("voxelsize_fine", 0.05);
    this->declare_parameter<double>("threshold_fitness_init", 0.9);
    this->declare_parameter<double>("threshold_fitness", 0.9);
    this->declare_parameter<std::vector<double>>("initialpose", std::vector<double>());
    this->declare_parameter<double>("dis_updatemap", 1);

    this->get_parameter("pcd_queue_maxsize", queue_maxsize_);
    this->get_parameter("save_scan", save_scan_);
    this->get_parameter("maxpoints_source", maxpoints_source_);
    this->get_parameter("maxpoints_target", maxpoints_target_);
    this->get_parameter("loc_frequence", loc_frequence_);
    this->get_parameter("confidence_loc_th", confidence_loc_th_);
    this->get_parameter("kf_baselink2map/x", kf_param_x_);
    this->get_parameter("kf_baselink2map/y", kf_param_y_);
    this->get_parameter("kf_baselink2map/z", kf_param_z_);
    this->get_parameter("filter_odom2map", filter_odom2map_);
    this->get_parameter("kalman_processVar2", kalman_processVar2_);
    this->get_parameter("kalman_estimatedMeasVar2", kalman_estimatedMeasVar2_);

    RCLCPP_INFO(this->get_logger(), "Kalman filter parameters:");
    RCLCPP_INFO(this->get_logger(), "  kf_x: [%.6f, %.6f], size: %zu",
                kf_param_x_.size() >= 1 ? kf_param_x_[0] : 0.0,
                kf_param_x_.size() >= 2 ? kf_param_x_[1] : 0.0,
                kf_param_x_.size());
    RCLCPP_INFO(this->get_logger(), "  kf_y: [%.6f, %.6f], size: %zu",
                kf_param_y_.size() >= 1 ? kf_param_y_[0] : 0.0,
                kf_param_y_.size() >= 2 ? kf_param_y_[1] : 0.0,
                kf_param_y_.size());
    RCLCPP_INFO(this->get_logger(), "  kf_z: [%.6f, %.6f], size: %zu",
                kf_param_z_.size() >= 1 ? kf_param_z_[0] : 0.0,
                kf_param_z_.size() >= 2 ? kf_param_z_[1] : 0.0,
                kf_param_z_.size());
    RCLCPP_INFO(this->get_logger(), "  filter_odom2map: %s", filter_odom2map_ ? "true" : "false");
    this->get_parameter("voxelsize_coarse", voxelsize_coarse_);
    this->get_parameter("voxelsize_fine", voxelsize_fine_);
    this->get_parameter("threshold_fitness_init", threshold_fitness_init_);
    this->get_parameter("threshold_fitness", threshold_fitness_);
    this->get_parameter("initialpose", initialpose_);
    this->get_parameter("dis_updatemap", dis_updatemap_);

    for (auto i : initialpose_)
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;
    mat_initialpose_.block<3, 3>(0, 0) = Euler2Matrix3d(Eigen::Vector3d(initialpose_[3], initialpose_[4], initialpose_[5]));
    mat_initialpose_.block<3, 1>(0, 3) = Eigen::Vector3d(initialpose_[0], initialpose_[1], initialpose_[2]);

    std::string path_map = "";
    this->declare_parameter<std::string>("path_map", "");
    this->get_parameter("path_map", path_map);
    open3d::io::ReadPointCloud(path_map, *pcd_map_ori_);
    if (pcd_map_ori_ == nullptr || pcd_map_ori_->IsEmpty())
    {
        RCLCPP_ERROR(this->get_logger(), "read map from path: %s failed", path_map.c_str());
        rclcpp::shutdown();
    }

    if (!pcd_map_ori_->HasColors())
    {
        pcd_map_ori_->PaintUniformColor({1, 0, 0});
    }

    pcd_map_coarse_ = pcd_map_ori_->VoxelDownSample(voxelsize_coarse_);
    pcd_map_coarse_->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(voxelsize_coarse_ * 2, 30));

    sensor_msgs::msg::PointCloud2 pc2_map;
    open3d_conversions::open3dToRos(*pcd_map_coarse_, pc2_map);
    pc2_map.header.frame_id = "map";
    pc2_map.header.stamp = this->now();
    pub_map_->publish(pc2_map);

    pcd_map_fine_ = pcd_map_ori_->VoxelDownSample(voxelsize_fine_);
    pcd_map_fine_->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(voxelsize_fine_ * 2, 30));

    RCLCPP_WARN(this->get_logger(), "initialize finished");

    br_odom2map_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    br_odom2base_footprint_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    StartLoc();
}

GloabalLocalization::~GloabalLocalization()
{
    lock_exit_.lock();
    flag_exit_ = true;
    lock_exit_.unlock();
}

Eigen::Matrix3d GloabalLocalization::Euler2Matrix3d(const Eigen::Vector3d euler)
{
    Eigen::Matrix3d mat3d;
    auto eulerAngle = euler / 180 * M_PI;
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle[0], Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle[1], Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle[2], Eigen::Vector3d::UnitZ()));
    mat3d = rollAngle * pitchAngle * yawAngle;
    return mat3d;
}

bool GloabalLocalization::GetTfTransformToMatrix(std::string frame_id, std::string child_frame_id, Eigen::Matrix4d &matrix)
{
    geometry_msgs::msg::TransformStamped pose_;
    try
    {
        pose_ = tf_buffer_.lookupTransform(frame_id, child_frame_id, rclcpp::Time(0));
    }
    catch (tf2::TransformException &e)
    {
        RCLCPP_ERROR(this->get_logger(), "[GetTransformMatrix]: %s", e.what());
        return false;
    }

    Eigen::Vector3d translation = Eigen::Vector3d(pose_.transform.translation.x, pose_.transform.translation.y, pose_.transform.translation.z);
    Eigen::Quaterniond quat = Eigen::Quaterniond::Identity();

    quat = Eigen::Quaterniond(pose_.transform.rotation.w,
                              pose_.transform.rotation.x,
                              pose_.transform.rotation.y,
                              pose_.transform.rotation.z);
    Eigen::Matrix3d rotation = quat.matrix();

    matrix = Eigen::Matrix4d::Identity();
    matrix.block<3, 3>(0, 0) = rotation;
    matrix.matrix().block<3, 1>(0, 3) = translation;
    return true;
}

void GloabalLocalization::CallbackBaseFootprint2Odom(const nav_msgs::msg::Odometry::SharedPtr base_footprint2odom)
{
    auto odom_cbk_s = std::chrono::high_resolution_clock::now();
    lock_timestamp_.lock();
    timestamp_odom_ = base_footprint2odom->header.stamp;
    lock_timestamp_.unlock();
    
    Eigen::Isometry3d mat_current = Eigen::Isometry3d::Identity();
    tf2::fromMsg(base_footprint2odom->pose.pose, mat_current);
    mat_fastlio_odom2base_footprint_ = mat_current.matrix();

    mat_base_footprint2odom_ = mat_fastlio_odom2base_footprint_;

    Eigen::Isometry3d Isometry3d_base_footprint2map;
    mat_base_footprint2map_ = mat_odom2map_ * mat_base_footprint2odom_;
    Isometry3d_base_footprint2map.matrix() = mat_base_footprint2map_;
    nav_msgs::msg::Odometry base_footprint2map;
    base_footprint2map.pose.pose = tf2::toMsg(Isometry3d_base_footprint2map);
    base_footprint2map.header.frame_id = "map";
    base_footprint2map.child_frame_id = "base_footprint";
    base_footprint2map.header.stamp = base_footprint2odom->header.stamp;
    pub_base_footprint2map_->publish(base_footprint2map);

    Eigen::Isometry3d Isometry3d_odom2map;
    Isometry3d_odom2map.matrix() = mat_odom2map_;
    nav_msgs::msg::Odometry odom2map;
    odom2map.pose.pose = tf2::toMsg(Isometry3d_odom2map);
    odom2map.header.frame_id = "map";
    odom2map.child_frame_id = "odom";
    odom2map.header.stamp = base_footprint2odom->header.stamp;
    pub_odom2map_->publish(odom2map);

    geometry_msgs::msg::TransformStamped transform_odom2map;
    transform_odom2map.header.frame_id = "map";
    transform_odom2map.child_frame_id = "odom";
    transform_odom2map.header.stamp = base_footprint2odom->header.stamp;
    transform_odom2map.transform.translation.x = odom2map.pose.pose.position.x;
    transform_odom2map.transform.translation.y = odom2map.pose.pose.position.y;
    transform_odom2map.transform.translation.z = odom2map.pose.pose.position.z;
    transform_odom2map.transform.rotation = odom2map.pose.pose.orientation;
    br_odom2map_->sendTransform(transform_odom2map);

    geometry_msgs::msg::TransformStamped transform_odom2base_footprint;
    transform_odom2base_footprint.header.frame_id = "odom";
    transform_odom2base_footprint.child_frame_id = "base_footprint";
    transform_odom2base_footprint.header.stamp = base_footprint2odom->header.stamp;
    
    Eigen::Isometry3d base2odom_isometry = Eigen::Isometry3d::Identity();
    base2odom_isometry.matrix() = mat_fastlio_odom2base_footprint_;
    
    geometry_msgs::msg::PoseStamped base2odom_pose;
    base2odom_pose.pose = tf2::toMsg(base2odom_isometry);
    
    transform_odom2base_footprint.transform.translation.x = base2odom_pose.pose.position.x;
    transform_odom2base_footprint.transform.translation.y = base2odom_pose.pose.position.y;
    transform_odom2base_footprint.transform.translation.z = base2odom_pose.pose.position.z;
    transform_odom2base_footprint.transform.rotation = base2odom_pose.pose.orientation;
    
    br_odom2base_footprint_->sendTransform(transform_odom2base_footprint);

    if (loc_initialized_)
    {
        Eigen::Matrix4d mat_base_footprint2map_kalman = Eigen::Matrix4d::Identity();

        if (filter_odom2map_)
        {
            Eigen::Isometry3d Isometry3d_odom2map_kalman;
            Isometry3d_odom2map_kalman.matrix() = mat_odom2map_kalman_;
            nav_msgs::msg::Odometry odom2map_kalman;
            odom2map_kalman.pose.pose = tf2::toMsg(Isometry3d_odom2map_kalman);
            odom2map_kalman.header.frame_id = "map";
            odom2map_kalman.child_frame_id = "odom_kalman";
            odom2map_kalman.header.stamp = base_footprint2odom->header.stamp;
            pub_odom2map_kalman_->publish(odom2map_kalman);

            kf_base_footprint_z_.inputLatestNoisyMeasurement((mat_odom2map_kalman_ * mat_base_footprint2odom_)(2, 3));
            mat_base_footprint2map_kalman = mat_odom2map_kalman_ * mat_base_footprint2odom_;
        }
        else
        {
            double input_x = mat_base_footprint2map_(0, 3);
            double input_y = mat_base_footprint2map_(1, 3);
            double input_z = mat_base_footprint2map_(2, 3);

            kf_base_footprint_x_.inputLatestNoisyMeasurement(input_x);
            kf_base_footprint_y_.inputLatestNoisyMeasurement(input_y);
            kf_base_footprint_z_.inputLatestNoisyMeasurement(input_z);
            mat_base_footprint2map_kalman = mat_base_footprint2map_;

            RCLCPP_DEBUG(this->get_logger(), "KF input: x=%.3f, y=%.3f, z=%.3f", input_x, input_y, input_z);
        }

        double filtered_z = kf_base_footprint_z_.getLatestEstimatedMeasurement();

        if (std::isnan(filtered_z))
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Kalman filter returned NaN (input was: %.3f), using unfiltered value",
                                 mat_base_footprint2map_kalman(2, 3));
            mat_base_footprint2map_kalman(2, 3) = mat_base_footprint2map_(2, 3);
        }
        else
        {
            mat_base_footprint2map_kalman(2, 3) = filtered_z;
        }
        Eigen::Isometry3d Isometry3d_base_footprint2map_kalman;
        Isometry3d_base_footprint2map_kalman.matrix() = mat_base_footprint2map_kalman;
        nav_msgs::msg::Odometry base_footprint2map_kalman;
        base_footprint2map_kalman.pose.pose = tf2::toMsg(Isometry3d_base_footprint2map_kalman);
        base_footprint2map_kalman.header.frame_id = "map";
        base_footprint2map_kalman.header.stamp = base_footprint2odom->header.stamp;
        pub_base_footprint2map_kalman_->publish(base_footprint2map_kalman);

        localization_3d_confidence_.data = loc_fitness_;
        pub_localization_3d_confidence_->publish(localization_3d_confidence_);
        localization_3d_delay_ms_.data = (this->now() - base_footprint2odom->header.stamp).seconds() * 1000.0;
        pub_localization_3d_delay_ms_->publish(localization_3d_delay_ms_);
        localization_3d_.header.frame_id = "map";
        localization_3d_.header.stamp = base_footprint2odom->header.stamp;
        localization_3d_.pose = tf2::toMsg(Isometry3d_base_footprint2map);
        pub_localization_3d_->publish(localization_3d_);
    }
}

void GloabalLocalization::CallbackScan(
    const sensor_msgs::msg::PointCloud2::SharedPtr scan_in_base_footprint)
{
    auto cbk_s = std::chrono::high_resolution_clock::now();
    open3d::geometry::PointCloud pcd_recieved;
    sensor_msgs::msg::PointCloud2::ConstSharedPtr const_scan_ptr = scan_in_base_footprint;
    open3d_conversions::rosToOpen3d(const_scan_ptr, pcd_recieved);

    if (que_pcd_scan_.size() >= static_cast<size_t>(queue_maxsize_))
    {
        std::queue<open3d::geometry::PointCloud> que_temp;
        lock_scan_.lock();
        pcd_scan_cur_->Clear();
        while (!que_pcd_scan_.empty())
        {
            *pcd_scan_cur_ += que_pcd_scan_.front();
            que_temp.push(que_pcd_scan_.front());
            que_pcd_scan_.pop();
        }
        lock_scan_.unlock();
        while (!que_temp.empty())
        {
            que_pcd_scan_.push(que_temp.front());
            que_temp.pop();
        }
        que_pcd_scan_.pop();
    }
    que_pcd_scan_.push(pcd_recieved);

    auto cbk_e = std::chrono::high_resolution_clock::now();
}

void GloabalLocalization::LocalizationInitialize()
{
    std::shared_ptr<open3d::geometry::PointCloud> map_coarse_crop(new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::PointCloud> map_fine_crop(new open3d::geometry::PointCloud);

    std::shared_ptr<open3d::geometry::PointCloud> pcd_scan(new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::PointCloud> pcd_scan2map(new open3d::geometry::PointCloud);

    std::shared_ptr<open3d::geometry::PointCloud> source(new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::PointCloud> target(new open3d::geometry::PointCloud);

    std::shared_ptr<open3d::geometry::OrientedBoundingBox> OBB_map(new open3d::geometry::OrientedBoundingBox);
    std::shared_ptr<open3d::geometry::OrientedBoundingBox> OBB_scan(new open3d::geometry::OrientedBoundingBox);

    Eigen::Matrix4d mat_base_footprint2odom_cur = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d mat_base_footprint2map_cur = Eigen::Matrix4d::Identity();

    OBB_map->extent_ = Eigen::Vector3d(60, 60, 40);
    OBB_map->color_ = Eigen::Vector3d(1, 0.5, 0);
    OBB_scan->extent_ = Eigen::Vector3d(60, 60, 40);
    OBB_scan->color_ = Eigen::Vector3d(0, 1, 0);

    double fitness_initial;
    double loc_cost = 0;
    int count_success = 0;
    while (rclcpp::ok())
    {
        auto loc_s = std::chrono::high_resolution_clock::now();
        lock_scan_.lock();
        if (pcd_scan_cur_->IsEmpty())
        {
            lock_scan_.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }
        else
        {
            mat_base_footprint2odom_cur = mat_base_footprint2odom_;
            mat_base_footprint2map_cur = mat_base_footprint2map_;
            *pcd_scan = *pcd_scan_cur_;
            lock_scan_.unlock();
            lock_mat_odom2map_.lock();

            OBB_map->center_ = mat_base_footprint2map_cur.block<3, 1>(0, 3);
            OBB_map->R_ = mat_base_footprint2map_cur.block<3, 3>(0, 0);
            OBB_scan->center_ = mat_base_footprint2odom_cur.block<3, 1>(0, 3);
            OBB_scan->R_ = mat_base_footprint2odom_cur.block<3, 3>(0, 0);
            *map_fine_crop = *pcd_map_fine_->Crop(*OBB_map);

            auto reg0_s = std::chrono::high_resolution_clock::now();

            Eigen::Matrix4d reg_matrix = Eigen::Matrix4d::Identity();
            reg_matrix = mat_odom2map_;

            *target = *map_fine_crop;
            open3d::utility::LogInfo("before sample, target size: {}, has normal: {}", target->points_.size(), target->HasNormals() ? "true" : "false");
            if (target->points_.size() > static_cast<size_t>(maxpoints_target_))
            {
                target = target->RandomDownSample(double(maxpoints_target_) / target->points_.size());
            }
            open3d::utility::LogInfo("after sample, target size: {}, has normal: {}", target->points_.size(), target->HasNormals() ? "true" : "false");

            source = pcd_scan->Crop(*OBB_scan);
            open3d::utility::LogInfo("source size: {}, has normal: {}", source->points_.size(), source->HasNormals() ? "true" : "false");
            if (source->points_.size() > static_cast<size_t>(maxpoints_source_))
            {
                source = source->RandomDownSample(double(maxpoints_source_) / source->points_.size());
            }
            open3d::utility::LogInfo("source size: {}, has normal: {}", source->points_.size(), source->HasNormals() ? "true" : "false");

            source->Transform(reg_matrix);
            *pcd_scan2map = *source;

            auto multiScale_reg_matrix = pcd_tools::RegistrationMultiScaleIcp(source, target, voxelsize_fine_, 1, {1, 2, 3});
            reg_matrix = multiScale_reg_matrix * reg_matrix;
            source->Transform(multiScale_reg_matrix);
            auto eva_result_coarse = open3d::pipelines::registration::EvaluateRegistration(*source, *target, voxelsize_fine_ * 3);
            open3d::utility::LogInfo("eva fitness: {}", eva_result_coarse.fitness_);
            fitness_initial = eva_result_coarse.fitness_;
            *pcd_scan2map = *source;

            mat_odom2map_ = reg_matrix;
            lock_mat_odom2map_.unlock();
            auto loc_e = std::chrono::high_resolution_clock::now();
            loc_cost = std::chrono::duration_cast<std::chrono::microseconds>(loc_e - loc_s).count() / 1000.0;
            RCLCPP_INFO(this->get_logger(), "localization cost: %f ms", loc_cost);

            if (fitness_initial > threshold_fitness_init_)
            {
                count_success += 1;
                if (count_success >= 2)
                {
                    break;
                }
            }
            else
            {
                count_success = 0;
            }
        }
    }

    open3d::utility::LogInfo("\n\n\nlocalization initialize success!!!!\n\n\n");
}

void GloabalLocalization::Localization()
{
    RCLCPP_INFO(this->get_logger(), "wait for Odometry_loc");
    while (rclcpp::ok() && timestamp_odom_.seconds() == 0.0)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for Odometry_loc...");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    RCLCPP_INFO(this->get_logger(), "Received Odometry_loc");

    RCLCPP_INFO(this->get_logger(), "wait for cloud_registered_1");
    while (rclcpp::ok())
    {
        lock_scan_.lock();
        bool has_scan = !pcd_scan_cur_->IsEmpty();
        lock_scan_.unlock();
        if (has_scan)
        {
            break;
        }
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for cloud_registered_1...");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    RCLCPP_INFO(this->get_logger(), "Received cloud_registered_1");

    mat_odom2map_ = mat_initialpose_;
    LocalizationInitialize();

    Eigen::Matrix4d init_base_footprint2map = mat_odom2map_ * mat_base_footprint2odom_;
    double init_x = init_base_footprint2map(0, 3);
    double init_y = init_base_footprint2map(1, 3);
    double init_z = init_base_footprint2map(2, 3);

    RCLCPP_INFO(this->get_logger(), "Initializing Kalman filters with position: x=%.3f, y=%.3f, z=%.3f",
                init_x, init_y, init_z);

    if (kf_param_x_.size() >= 2 && kf_param_y_.size() >= 2 && kf_param_z_.size() >= 2)
    {
        kf_base_footprint_x_.KalmanFilterInit(kf_param_x_[0], kf_param_x_[1], init_x, 1);
        kf_base_footprint_y_.KalmanFilterInit(kf_param_y_[0], kf_param_y_[1], init_y, 1);
        kf_base_footprint_z_.KalmanFilterInit(kf_param_z_[0], kf_param_z_[1], init_z, 1);
        RCLCPP_INFO(this->get_logger(), "Kalman filters initialized: x[%.6f,%.6f], y[%.6f,%.6f], z[%.6f,%.6f]",
                    kf_param_x_[0], kf_param_x_[1], kf_param_y_[0], kf_param_y_[1],
                    kf_param_z_[0], kf_param_z_[1]);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid Kalman filter parameters! x_size=%zu, y_size=%zu, z_size=%zu",
                     kf_param_x_.size(), kf_param_y_.size(), kf_param_z_.size());
        RCLCPP_ERROR(this->get_logger(), "Kalman filters will NOT be initialized - using default values");
    }

    kalman_filter_odom2map_.KalmanFilterInit(kalman_processVar2_, kalman_estimatedMeasVar2_, init_z, 1);

    loc_initialized_ = true;

    RCLCPP_INFO(this->get_logger(), "Localization initialization complete, Kalman filters ready");

    double fitness = 0;
    auto coordinate_ori = open3d::geometry::TriangleMesh::CreateCoordinateFrame(2.0);
    auto coordinate_loc = open3d::geometry::TriangleMesh::CreateCoordinateFrame(2.0);
    auto coordinate_OBB_scan = open3d::geometry::TriangleMesh::CreateCoordinateFrame(2.0);
    std::shared_ptr<open3d::geometry::PointCloud> pcd_scan(new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::PointCloud> pcd_scancrop(new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::PointCloud> pcd_scan2map(new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::PointCloud> source(new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::PointCloud> target(new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::PointCloud> map_coarse_crop(new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::PointCloud> map_fine_crop(new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::PointCloud> pcd_submap(new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::OrientedBoundingBox> OBB_map(new open3d::geometry::OrientedBoundingBox);
    std::shared_ptr<open3d::geometry::OrientedBoundingBox> OBB_scan(new open3d::geometry::OrientedBoundingBox);
    OBB_map->color_ = Eigen::Vector3d(1, 0.5, 0);
    OBB_map->extent_ = Eigen::Vector3d(60, 60, 40);

    OBB_scan->extent_ = Eigen::Vector3d(60, 60, 40);
    OBB_scan->color_ = Eigen::Vector3d(0, 1, 0);
    rclcpp::Time time_current = timestamp_odom_;
    rclcpp::Time time_last = time_current - rclcpp::Duration(3, 0);

    RCLCPP_INFO(this->get_logger(), "time_last: %f", time_last.seconds());
    RCLCPP_INFO(this->get_logger(), "time_current: %f", time_current.seconds());
    int scan_count = 0;

    std::string save_path = "/home/carlos/mount/E/lixin/data/yq_bag/scan_submap/";

    double time_diff_loc = 5;
    std::chrono::high_resolution_clock::time_point time_last_loc;
    std::chrono::high_resolution_clock::time_point time_this_loc;
    double loc_cost = 0;
    while (rclcpp::ok())
    {

        lock_timestamp_.lock();
        time_current = timestamp_odom_;
        lock_timestamp_.unlock();
        auto time_diff_frame = time_current.seconds() - time_last.seconds();
        time_last = time_current;
        if (std::fabs(time_diff_frame) < 1e-6)
        {
            loc_cost = 0.0;
            continue;
        }

        time_this_loc = std::chrono::high_resolution_clock::now();
        time_diff_loc = std::chrono::duration_cast<std::chrono::microseconds>(time_this_loc - time_last_loc).count() / 1000000.0 + loc_cost / 1000.0;

        if (time_diff_loc < loc_frequence_)
        {
            int wait_time = int((loc_frequence_ - time_diff_loc) * 1000);
            open3d::utility::LogInfo("\n\ntime_this_loc: {}, time_last: {},\ntime_diff: {} s, sleep {} ms",
                                     std::chrono::duration_cast<std::chrono::milliseconds>(time_this_loc.time_since_epoch()).count() / 1000.0,
                                     std::chrono::duration_cast<std::chrono::milliseconds>(time_last_loc.time_since_epoch()).count() / 1000.0, time_diff_loc, wait_time);
            std::this_thread::sleep_for(std::chrono::milliseconds(wait_time));
        }
        else
        {
            open3d::utility::LogInfo("\n\ntime_diff:{} s, localization right now", time_diff_loc);
        }
        auto loc_s = std::chrono::high_resolution_clock::now();

        lock_scan_.lock();
        if (pcd_scan_cur_->IsEmpty())
        {
            lock_scan_.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }
        else
        {
            if (filter_odom2map_)
            {
                kalman_filter_odom2map_.inputLatestNoisyMeasurement(mat_odom2map_(2, 3));
                kalman_filter_odom2map_.inputLatestNoisyMeasurement(mat_odom2map_(2, 3));
                mat_odom2map_kalman_ = mat_odom2map_;
                mat_odom2map_kalman_(2, 3) = kalman_filter_odom2map_.getLatestEstimatedMeasurement();
            }
            Eigen::Matrix4d mat_base_footprint2odom_cur = Eigen::Matrix4d::Identity();
            Eigen::Matrix4d mat_base_footprint2map_cur = Eigen::Matrix4d::Identity();

            mat_base_footprint2odom_cur = mat_base_footprint2odom_;
            mat_base_footprint2map_cur = mat_base_footprint2map_;
            *pcd_scan = *pcd_scan_cur_;
            lock_scan_.unlock();
            Eigen::Vector3d cur_loc(mat_base_footprint2map_cur(0, 3), mat_base_footprint2map_cur(1, 3), mat_base_footprint2map_cur(2, 3));
            auto dis_motion = ComputeMotionDis(last_loc_, cur_loc);
            if (dis_motion > dis_updatemap_)
            {
                auto submap_s = std::chrono::high_resolution_clock::now();

                open3d::utility::LogInfo("\n***\n****\n***\n\n\nlast map update loc: x: {}, y: {}, z{},\n\
                now loc: x: {}, y: {}, z{}, 3d distance: {}, now needpdate submap",
                                         last_loc_.x(), last_loc_.y(), last_loc_.z(), cur_loc.x(), cur_loc.y(), cur_loc.z(), dis_motion);
                last_loc_ = cur_loc;
                OBB_map->center_ = mat_base_footprint2map_cur.block<3, 1>(0, 3);
                OBB_map->R_ = mat_base_footprint2map_cur.block<3, 3>(0, 0);

                *map_fine_crop = *pcd_map_fine_->Crop(*OBB_map);

                auto submap_e = std::chrono::high_resolution_clock::now();
                auto submap_cost = std::chrono::duration_cast<std::chrono::microseconds>(submap_e - submap_s).count() / 1000.0;
                RCLCPP_INFO(this->get_logger(), "submap_cost: %f ms", submap_cost);
            }

            OBB_scan->center_ = mat_base_footprint2odom_cur.block<3, 1>(0, 3);
            OBB_scan->R_ = mat_base_footprint2odom_cur.block<3, 3>(0, 0);

            auto reg0_s = std::chrono::high_resolution_clock::now();

            Eigen::Matrix4d reg_matrix = Eigen::Matrix4d::Identity();

            lock_mat_odom2map_.lock();
            reg_matrix = mat_odom2map_;

            *target = *map_fine_crop;
            open3d::utility::LogInfo("before sample, target size: {}, has normal: {}", target->points_.size(), target->HasNormals() ? "true" : "false");
            if (target->points_.size() > static_cast<size_t>(maxpoints_target_))
            {
                target = target->RandomDownSample(double(maxpoints_target_) / target->points_.size());
            }
            open3d::utility::LogInfo("after sample, target size: {}, has normal: {}", target->points_.size(), target->HasNormals() ? "true" : "false");

            source = pcd_scan->Crop(*OBB_scan);
            open3d::utility::LogInfo("source size: {}, maxpoints_source_: {}", source->points_.size(), maxpoints_source_);
            source = source->VoxelDownSample(voxelsize_fine_);
            open3d::utility::LogInfo("source size after voxel downsample: {}", source->points_.size());
            if (source->points_.size() > static_cast<size_t>(maxpoints_source_))
            {
                source = source->RandomDownSample(double(maxpoints_source_) / source->points_.size());
            }
            open3d::utility::LogInfo("after prerpocess: {}", source->points_.size());

            auto reg_result2 = pcd_tools::RegistrationIcp(source, target, voxelsize_fine_ * 2, reg_matrix, 1);
            reg_matrix = reg_result2.transformation_ * reg_matrix;
            auto eva_result2 = open3d::pipelines::registration::EvaluateRegistration(*source, *target, voxelsize_fine_ * 4, reg_matrix);
            loc_fitness_ = eva_result2.fitness_;
            open3d::utility::LogInfo("reg_result.fitness: {}, eva fitness: {}", reg_result2.fitness_, eva_result2.fitness_);
            if (loc_fitness_ > threshold_fitness_)
            {
                mat_odom2map_ = reg_matrix;
            }
            lock_mat_odom2map_.unlock();

            if (save_scan_)
            {
                pcd_scan->Transform(mat_base_footprint2odom_cur.inverse());
                pcd_scan2map->Transform(mat_base_footprint2map_cur.inverse());
                open3d::io::WritePointCloud(save_path + std::to_string(scan_count) + "_ori.ply", *pcd_scan);
                open3d::io::WritePointCloud(save_path + std::to_string(scan_count) + "_crop.ply", *pcd_scan2map);
                scan_count += 1;
            }

            auto loc_e = std::chrono::high_resolution_clock::now();
            time_last_loc = loc_e;
            loc_cost = std::chrono::duration_cast<std::chrono::microseconds>(loc_e - loc_s).count() / 1000.0;
            RCLCPP_INFO(this->get_logger(), "localization cost: %f ms", loc_cost);
        }
    }
}

void GloabalLocalization::StartLoc()
{
    thread_loc_ = std::thread(&GloabalLocalization::Localization, this);
}

void GloabalLocalization::CallbackInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr initialpose)
{
    std::cout << "mat_odom2map_\n"
              << mat_odom2map_ << std::endl;
    std::cout << "confidence_loc_th_: " << confidence_loc_th_ << " current confidence: " << loc_fitness_ << std::endl;

    if (!(loc_initialized_ && loc_fitness_ > 0.99))
    {
        std::cout << "initpose:x y z, x y z w\n"
                  << initialpose->pose.pose.position.x << " "
                  << initialpose->pose.pose.position.y << " "
                  << initialpose->pose.pose.position.z << " "
                  << initialpose->pose.pose.orientation.x << " "
                  << initialpose->pose.pose.orientation.y << " "
                  << initialpose->pose.pose.orientation.z << " "
                  << initialpose->pose.pose.orientation.w << std::endl;

        Eigen::Quaterniond rotation_q;
        rotation_q.w() = initialpose->pose.pose.orientation.w;
        rotation_q.x() = initialpose->pose.pose.orientation.x;
        rotation_q.y() = initialpose->pose.pose.orientation.y;
        rotation_q.z() = initialpose->pose.pose.orientation.z;
        mat_initialpose_.block<3, 3>(0, 0) = rotation_q.matrix();
        mat_initialpose_.block<3, 1>(0, 3) = Eigen::Vector3d(initialpose->pose.pose.position.x, initialpose->pose.pose.position.y, initialpose->pose.pose.position.z);
        lock_mat_odom2map_.lock();
        mat_odom2map_ = mat_initialpose_;
        lock_mat_odom2map_.unlock();
        std::cout << "\n\n*** update mat_odom2map_" << std::endl;
    }
    std::cout << "mat_odom2map_\n"
              << mat_odom2map_ << std::endl;
}

double GloabalLocalization::ComputeMotionDis(const Eigen::Vector3d &a, const Eigen::Vector3d &b)
{
    return std::sqrt(std::pow(a.x() - b.x(), 2) + std::pow(a.y() - b.y(), 2) + std::pow(a.z() - b.z(), 2));
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GloabalLocalization>();

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}