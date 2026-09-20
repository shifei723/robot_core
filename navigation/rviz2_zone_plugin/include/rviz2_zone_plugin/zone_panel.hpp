#ifndef ZONE_PANEL_HPP
#define ZONE_PANEL_HPP

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <QtWidgets>
#include <map>
#include <string>
#include <rviz_common/display_context.hpp>

namespace rviz2_zone_plugin {

// 存储结构：包含中文名和位姿坐标
struct ZoneInfo {
    std::string chinese_name;
    geometry_msgs::msg::Pose pose;
};

class ZonePanel : public rviz_common::Panel {
    Q_OBJECT
public:
    ZonePanel(QWidget* parent = nullptr);
    virtual void onInitialize() override;

protected Q_SLOTS:
    void onRecordButtonClicked();
    void onSaveButtonClicked();
    void onDeleteButtonClicked();

private:
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void publishMarkers();
    void loadFromYaml();
    std::string getYamlPath();

    // ROS 2 组件
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    // UI 组件
    QLineEdit* id_input_;    // ID 输入框 (英文/逻辑用)
    QLineEdit* name_input_;  // 名称输入框 (中文/显示用)
    QPushButton* record_btn_;
    QPushButton* delete_btn_;
    QPushButton* save_btn_;
    QListWidget* zone_list_;

    // 数据成员
    bool is_recording_;
    // key: 英文 ID, value: ZoneInfo 结构体
    std::map<std::string, ZoneInfo> zones_; 
};

} // namespace rviz2_zone_plugin

#endif // ZONE_PANEL_HPP