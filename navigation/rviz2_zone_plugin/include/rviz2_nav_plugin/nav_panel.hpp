#ifndef NAV_PANEL_HPP
#define NAV_PANEL_HPP

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <zone_interfaces/srv/go_to_zone.hpp>
#include <QtWidgets>
#include <map>
#include <string>

namespace rviz2_zone_plugin { // 统一命名空间

class NavPanel : public rviz_common::Panel {
    Q_OBJECT
public:
    NavPanel(QWidget* parent = nullptr);
    virtual void onInitialize() override;

protected Q_SLOTS:
    void onGoButtonClicked();
    void onRefreshClicked();
    void onTopicChanged(); 

private:
    void loadFromYaml();
    std::string getYamlPath();
    void handleResponse(rclcpp::Client<zone_interfaces::srv::GoToZone>::SharedFuture future);
    void commandCallback(const std_msgs::msg::String::SharedPtr msg);
    void executeNavigation(const std::string& chinese_name);

    // ROS 2 
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<zone_interfaces::srv::GoToZone>::SharedPtr client_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

    // ros2 topic feed back 
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr feedback_pub_;
    
    // UI 组件成员变量
    QLineEdit* topic_input_;
    QListWidget* nav_list_;
    QPushButton* go_btn_;
    QPushButton* refresh_btn_; // 必须在这里声明
    QLabel* status_label_;

    std::map<std::string, std::string> name_to_id_;
};

} // namespace rviz2_zone_plugin
#endif