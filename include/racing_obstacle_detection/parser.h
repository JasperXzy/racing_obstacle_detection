#pragma once

#include <memory>
#include <vector>
#include <string>
#include <chrono>
#include "ai_msgs/msg/perception_targets.hpp"
#include "rclcpp/rclcpp.hpp"

// 单目标推理结果结构体
struct DetResult {
    int class_id;
    float confidence;
    float xmin;
    float ymin;
    float xmax;
    float ymax;
    std::string class_name;
};

class Yolo11Parser {
public:
    Yolo11Parser();
    // 解析函数: 从BPU模型输出，后处理出所有目标并填充DetResult
    int postprocess(
        hbDNNTensor* outputs, int output_count, // 推理输出
        int input_width, int input_height,      // 输入尺寸
        float conf_threshold, float nms_threshold,
        std::vector<std::string>& class_names,
        std::vector<DetResult>& results);

    // 发布到ros2
    void publish_to_ros(
        rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr pub,
        const std::vector<DetResult>& results,
        const rclcpp::Time& stamp = rclcpp::Clock().now());
};
