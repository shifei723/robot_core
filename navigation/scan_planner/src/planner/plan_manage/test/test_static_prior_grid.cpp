#include <gtest/gtest.h>
#include <plan_env/grid_map.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <thread>

TEST(StaticPriorGrid, CannotBeClearedByLiveEvidenceAndReplacesAtomically) {
  rclcpp::init(0,nullptr);
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"grid_map.static_prior_enabled",true},{"grid_map.resolution",.05},
    {"grid_map.sliding_map_size_x",4.},{"grid_map.sliding_map_size_y",4.},{"grid_map.sliding_map_size_z",2.},
    {"grid_map.obstacles_inflation_z_up",.3},{"grid_map.obstacles_inflation_z_down",.4},
    {"grid_map.double_cylinder_radius",.43},{"grid_map.world_bound_enable",false},
    {"grid_map.p_hit",.85},{"grid_map.p_miss",.3},{"grid_map.p_min",.12},{"grid_map.p_max",.98},{"grid_map.p_occ",.8},
    {"grid_map.skip_pixel",2},{"grid_map.depth_filter_margin",0}
  });
  auto node=std::make_shared<rclcpp::Node>("static_prior_grid_test",options);
  GridMap map;map.initMap(node.get());
  auto pub=node->create_publisher<sensor_msgs::msg::PointCloud2>("/dynamic_obstacles/static_prior",rclcpp::QoS(1).transient_local());
  auto send=[&](double x,const std::string& frame="world") {
    sensor_msgs::msg::PointCloud2 msg;msg.header.frame_id=frame;msg.header.stamp=node->now();
    sensor_msgs::PointCloud2Modifier mod(msg);mod.setPointCloud2FieldsByString(1,"xyz");mod.resize(1);
    sensor_msgs::PointCloud2Iterator<float> ix(msg,"x"),iy(msg,"y"),iz(msg,"z");*ix=x;*iy=0;*iz=.21;
    pub->publish(msg);
    for(int i=0;i<25;++i) {rclcpp::spin_some(node);std::this_thread::sleep_for(std::chrono::milliseconds(10));}
  };
  send(1.01);
  const Eigen::Vector3d original(1.01,0,.21);
  EXPECT_EQ(map.getOccupancy(original),1);
  map.setOccupancy(original,0);
  EXPECT_EQ(map.getOccupancy(original),1);
  EXPECT_EQ(map.getInflateOccupancy(Eigen::Vector3d(.81,0,.28),0),1);
  map.resetBuffer();
  EXPECT_EQ(map.getOccupancy(original),1);
  EXPECT_LT(map.queryCeilingHeight(Eigen::Vector3d(1.01,0,.28),0),.22);
  send(.51,"wrong_frame");
  EXPECT_EQ(map.getOccupancy(original),1);
  send(.51);
  EXPECT_EQ(map.getOccupancy(original),0);
  EXPECT_EQ(map.getOccupancy(Eigen::Vector3d(.51,0,.21)),1);
  node.reset();rclcpp::shutdown();
}
