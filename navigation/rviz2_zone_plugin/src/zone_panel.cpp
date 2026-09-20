#include "rviz2_zone_plugin/zone_panel.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <filesystem>
#include <QMessageBox>

namespace rviz2_zone_plugin {

ZonePanel::ZonePanel(QWidget* parent) : rviz_common::Panel(parent), is_recording_(false) {
    // 1. 构建 UI 布局
    QVBoxLayout* layout = new QVBoxLayout;
    
    // ID 输入区域 (英文/数字标识)
    layout->addWidget(new QLabel("<b>1. 区域 ID (如: kitchen_01)</b>"));
    id_input_ = new QLineEdit;
    id_input_->setPlaceholderText("请输入英文 ID...");
    layout->addWidget(id_input_);

    // 中文名称输入区域 (用于 UI 显示)
    layout->addWidget(new QLabel("<b>2. 中文名称 (如: 厨房)</b>"));
    name_input_ = new QLineEdit;
    name_input_->setPlaceholderText("请输入中文名称...");
    layout->addWidget(name_input_);

    record_btn_ = new QPushButton("3. 开启录制 (地图点击 2D Goal Pose)");
    layout->addWidget(record_btn_);

    zone_list_ = new QListWidget;
    layout->addWidget(new QLabel("已记录区域 (中文名 + ID):"));
    layout->addWidget(zone_list_);

    delete_btn_ = new QPushButton("删除选中区域");
    delete_btn_->setStyleSheet("QPushButton { color: white; background-color: #E53935; }");
    layout->addWidget(delete_btn_);

    save_btn_ = new QPushButton("4. 保存并同步到 YAML");
    save_btn_->setStyleSheet("QPushButton { font-weight: bold; height: 30px; }");
    layout->addWidget(save_btn_);

    setLayout(layout);

    // 2. 信号槽绑定
    connect(record_btn_, &QPushButton::clicked, this, &ZonePanel::onRecordButtonClicked);
    connect(save_btn_, &QPushButton::clicked, this, &ZonePanel::onSaveButtonClicked);
    connect(delete_btn_, &QPushButton::clicked, this, &ZonePanel::onDeleteButtonClicked);
}

std::string ZonePanel::getYamlPath() {
    std::filesystem::path current_file(__FILE__);
    // 默认保存在功能包根目录下的 locations.yaml
    return current_file.parent_path().parent_path().string() + "/locations.yaml";
}

void ZonePanel::onInitialize() {
    auto context = this->getDisplayContext();
    auto node_ptr = context->getRosNodeAbstraction().lock();
    if (node_ptr) {
        node_ = node_ptr->get_raw_node();
        pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&ZonePanel::poseCallback, this, std::placeholders::_1));
        marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/zone_markers", 10);
        
        // 延迟加载，确保 RViz 启动稳定
        QTimer::singleShot(1000, this, [this](){ loadFromYaml(); });
    }
}

void ZonePanel::onDeleteButtonClicked() {
    QListWidgetItem* item = zone_list_->currentItem();
    if (!item) {
        QMessageBox::warning(this, "提示", "请先在列表中选择一个区域！");
        return;
    }

    // 从显示文本 "厨房 (kitchen_01)" 中提取括号内的 ID
    std::string text = item->text().toStdString();
    size_t start = text.find_last_of("(") + 1;
    size_t end = text.find_last_of(")");
    
    if (start != std::string::npos && end != std::string::npos) {
        std::string id = text.substr(start, end - start);
        zones_.erase(id);
    }

    delete zone_list_->takeItem(zone_list_->row(item));
    publishMarkers();
}

void ZonePanel::onRecordButtonClicked() {
    if (id_input_->text().trimmed().isEmpty() || name_input_->text().trimmed().isEmpty()) {
        QMessageBox::warning(this, "警告", "ID 和中文名称都不能为空！");
        return;
    }
    is_recording_ = true;
    record_btn_->setText("正在录制... 请在地图点击 2D Goal Pose");
    record_btn_->setStyleSheet("background-color: #FBC02D; color: black;");
}

void ZonePanel::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!is_recording_) return;

    std::string zone_id = id_input_->text().trimmed().toStdString();
    std::string zone_name = name_input_->text().trimmed().toStdString();

    // 1. 存储到内存 map
    ZoneInfo info;
    info.chinese_name = zone_name;
    info.pose = msg->pose;
    zones_[zone_id] = info;

    // 2. 更新 UI 列表显示： "中文名 (ID)"
    QString display_text = QString::fromStdString(zone_name + " (" + zone_id + ")");
    
    bool exists = false;
    for(int i = 0; i < zone_list_->count(); ++i) {
        if(zone_list_->item(i)->text().contains(QString::fromStdString("(" + zone_id + ")"))) {
            zone_list_->item(i)->setText(display_text + " [更新]");
            exists = true;
            break;
        }
    }
    if(!exists) {
        zone_list_->addItem(display_text);
    }
    
    // 重置状态
    is_recording_ = false;
    record_btn_->setText("1. 开启录制 (地图点击 2D Goal Pose)");
    record_btn_->setStyleSheet("");
    publishMarkers();
}

void ZonePanel::onSaveButtonClicked() {
    YAML::Node config;
    for (const auto& [id, info] : zones_) {
        YAML::Node zone_node;
        zone_node["name"] = info.chinese_name; // 关键：中文存入 name 字段
        zone_node["x"] = info.pose.position.x;
        zone_node["y"] = info.pose.position.y;
        zone_node["z"] = info.pose.orientation.z;
        zone_node["w"] = info.pose.orientation.w;
        config["zones"][id] = zone_node; // 关键：以英文 ID 为键
    }

    std::string path = getYamlPath();
    std::ofstream fout(path);
    if (fout.is_open()) {
        fout << config;
        fout.close();
        QMessageBox::information(this, "成功", "配置文件已同步至：\n" + QString::fromStdString(path));
    } else {
        QMessageBox::critical(this, "错误", "保存失败，请检查目录读写权限！");
    }
}

void ZonePanel::publishMarkers() {
    if (!marker_pub_) return;

    visualization_msgs::msg::MarkerArray ma;
    // 发送清除所有标记的命令
    visualization_msgs::msg::Marker clear_m;
    clear_m.action = visualization_msgs::msg::Marker::DELETEALL;
    ma.markers.push_back(clear_m);
    marker_pub_->publish(ma);
    ma.markers.clear();

    if (zones_.empty()) return;

    int idx = 0;
    for (const auto& [id, info] : zones_) {
        // 文字标记 (显示中文名称)
        visualization_msgs::msg::Marker text_m;
        text_m.header.frame_id = "map";
        text_m.ns = "zone_labels";
        text_m.id = idx++;
        text_m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_m.action = visualization_msgs::msg::Marker::ADD;
        text_m.pose = info.pose;
        text_m.pose.position.z += 0.6; 
        text_m.scale.z = 0.35;
        text_m.color.r = 1.0; text_m.color.g = 1.0; text_m.color.b = 0.0; text_m.color.a = 1.0;
        text_m.text = id;
        ma.markers.push_back(text_m);

        // 箭头标记 (指示朝向)
        visualization_msgs::msg::Marker arrow_m;
        arrow_m.header.frame_id = "map";
        arrow_m.ns = "zone_arrows";
        arrow_m.id = idx++;
        arrow_m.type = visualization_msgs::msg::Marker::ARROW;
        arrow_m.pose = info.pose;
        arrow_m.scale.x = 0.6; arrow_m.scale.y = 0.12; arrow_m.scale.z = 0.12;
        arrow_m.color.r = 0.1; arrow_m.color.g = 0.8; arrow_m.color.b = 0.1; arrow_m.color.a = 0.9;
        ma.markers.push_back(arrow_m);
    }
    marker_pub_->publish(ma);
}

void ZonePanel::loadFromYaml() {
    std::string path = getYamlPath();
    if (!std::filesystem::exists(path)) return;

    try {
        YAML::Node config = YAML::LoadFile(path);
        zones_.clear();
        zone_list_->clear();
        if (config["zones"]) {
            for (auto it = config["zones"].begin(); it != config["zones"].end(); ++it) {
                std::string id = it->first.as<std::string>();
                ZoneInfo info;
                // 支持旧格式兼容：如果没有 name 字段，就使用 ID 充当名称
                info.chinese_name = it->second["name"] ? it->second["name"].as<std::string>() : id;
                info.pose.position.x = it->second["x"].as<double>();
                info.pose.position.y = it->second["y"].as<double>();
                info.pose.orientation.z = it->second["z"].as<double>();
                info.pose.orientation.w = it->second["w"].as<double>();
                
                zones_[id] = info;
                zone_list_->addItem(QString::fromStdString(info.chinese_name + " (" + id + ")"));
            }
        }
        publishMarkers();
    } catch (...) {
        RCLCPP_WARN(node_->get_logger(), "YAML 加载失败，可能是文件为空或格式不匹配。");
    }
}

} // namespace rviz2_zone_plugin

// 导出插件
PLUGINLIB_EXPORT_CLASS(rviz2_zone_plugin::ZonePanel, rviz_common::Panel)