#include "rviz2_nav_plugin/nav_panel.hpp" 
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <filesystem>

namespace rviz2_zone_plugin { // 统一命名空间

NavPanel::NavPanel(QWidget* parent) : rviz_common::Panel(parent) {
    QVBoxLayout* layout = new QVBoxLayout;

    layout->addWidget(new QLabel("<b>监听指令 Topic (回车生效):</b>"));
    topic_input_ = new QLineEdit("/nav_commands");
    layout->addWidget(topic_input_);

    layout->addWidget(new QLabel("<b>目的地列表 (中文):</b>"));
    nav_list_ = new QListWidget;
    layout->addWidget(nav_list_);

    go_btn_ = new QPushButton("下发导航任务");
    go_btn_->setStyleSheet("background-color: #1976D2; color: white; font-weight: bold; height: 35px;");
    layout->addWidget(go_btn_);

    status_label_ = new QLabel("状态: 待机");
    status_label_->setStyleSheet("padding: 5px; border: 1px solid #ccc;");
    layout->addWidget(status_label_);

    // 修正：去掉前面的 QPushButton*，直接赋值给类成员
    refresh_btn_ = new QPushButton("刷新地址簿"); 
    layout->addWidget(refresh_btn_);

    setLayout(layout);

    connect(go_btn_, &QPushButton::clicked, this, &NavPanel::onGoButtonClicked);
    connect(refresh_btn_, &QPushButton::clicked, this, &NavPanel::onRefreshClicked);
    connect(topic_input_, &QLineEdit::returnPressed, this, &NavPanel::onTopicChanged);
}

void NavPanel::onInitialize() {
    auto node_ptr = getDisplayContext()->getRosNodeAbstraction().lock();
    if (node_ptr) {
        node_ = node_ptr->get_raw_node();
        // 创建导航客户端
        client_ = node_->create_client<zone_interfaces::srv::GoToZone>("/go_to_zone");
        // 创建反馈发布者
        feedback_pub_ = node_->create_publisher<std_msgs::msg::String>("/nav_feedback", 10);
        onTopicChanged();
        loadFromYaml();
    }
}

void NavPanel::onTopicChanged() {
    std::string new_topic = topic_input_->text().trimmed().toStdString();
    if (new_topic.empty()) return;

    sub_ = node_->create_subscription<std_msgs::msg::String>(
        new_topic, 10, std::bind(&NavPanel::commandCallback, this, std::placeholders::_1));
    
    status_label_->setText("已切换监听 Topic: " + QString::fromStdString(new_topic));
}

void NavPanel::commandCallback(const std_msgs::msg::String::SharedPtr msg) {
    std::string cmd = msg->data;
    RCLCPP_INFO(node_->get_logger(), "收到 Topic 指令: %s", cmd.c_str());

    for (auto const& [chinese_name, id] : name_to_id_) {
        if (cmd.find(chinese_name) != std::string::npos) {
            executeNavigation(chinese_name);
            return;
        }
    }
    status_label_->setText("未找到匹配地点: " + QString::fromStdString(cmd));
}

void NavPanel::executeNavigation(const std::string& chinese_name) {
    if (name_to_id_.find(chinese_name) == name_to_id_.end()) return;

    std::string target_id = name_to_id_[chinese_name];

    if (!client_->wait_for_service(std::chrono::seconds(1))) {
        status_label_->setText("错误: 服务未启动");
        return;
    }

    auto request = std::make_shared<zone_interfaces::srv::GoToZone::Request>();
    request->zone_name = target_id;

    status_label_->setText("正在下发: " + QString::fromStdString(chinese_name));
    status_label_->setStyleSheet("color: blue; font-weight: bold;");

    client_->async_send_request(request, std::bind(&NavPanel::handleResponse, this, std::placeholders::_1));
}

void NavPanel::handleResponse(rclcpp::Client<zone_interfaces::srv::GoToZone>::SharedFuture future) {
    auto feedback_msg = std_msgs::msg::String();
    try {
        auto response = future.get();
        if (response->success) {
            status_label_->setText("导航成功");
            status_label_->setStyleSheet("color: green; font-weight: bold;");
            feedback_msg.data = "success";
        } else {
            status_label_->setText("导航失败");
            status_label_->setStyleSheet("color: red;");
            feedback_msg.data = "failure";
        }
    } catch (...) {
        status_label_->setText("异常");
        feedback_msg.data = "exception";
    }
    feedback_pub_->publish(feedback_msg);
}

std::string NavPanel::getYamlPath() {
    std::filesystem::path current_file(__FILE__);
    return current_file.parent_path().parent_path().string() + "/locations.yaml";
}

void NavPanel::loadFromYaml() {
    std::string path = getYamlPath();
    if (!std::filesystem::exists(path)) return;
    try {
        YAML::Node config = YAML::LoadFile(path);
        nav_list_->clear();
        name_to_id_.clear();
        if (config["zones"]) {
            for (auto it = config["zones"].begin(); it != config["zones"].end(); ++it) {
                std::string id = it->first.as<std::string>();
                std::string name = it->second["name"].as<std::string>();
                nav_list_->addItem(QString::fromStdString(name));
                name_to_id_[name] = id;
            }
        }
    } catch (...) {}
}

void NavPanel::onGoButtonClicked() {
    if (auto item = nav_list_->currentItem()) {
        executeNavigation(item->text().toStdString());
    }
}

void NavPanel::onRefreshClicked() { loadFromYaml(); }

} // namespace rviz2_zone_plugin

// 修正：确保这里的命名空间和上面定义的 NavPanel 所在空间一致
PLUGINLIB_EXPORT_CLASS(rviz2_zone_plugin::NavPanel, rviz_common::Panel)