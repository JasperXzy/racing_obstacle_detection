#include "parser.h"
#include "sensor_msgs/msg/region_of_interest.hpp"

// OpenCV NMS
#include <opencv2/opencv.hpp>
#include <opencv2/dnn/dnn.hpp>

Yolo11Parser::Yolo11Parser() {}

int Yolo11Parser::postprocess(
    hbDNNTensor* outputs, int output_count,
    int input_width, int input_height,
    float conf_threshold, float nms_threshold,
    std::vector<std::string>& class_names,
    std::vector<DetResult>& results)
{
    // 假定outputs的格式、顺序你已经和你的模型配置对好（同你主推理代码）
    // 假定所有输出都已经同步到CPU内存

    // <<<<<<<<<<<< 需根据你的模型实际输出格式略做适配 >>>>>>>>>>>
    // 本例使用YOLO11/yolov8(RDK导出)兼容格式: 3组特征层,每组2个输出(分类+回归)
    // 建议先复用你已有主程序的后处理, 输出每一帧 DetResult

    // 假设你已经from outputs 得到 float xmin,ymin,xmax,ymax,class_id,confidence,class_name
    // 全部塞到 results

    // 例：(这里只做示意，实际逻辑建议copy你自己主程序的代码)
    // outputs ==> postprocess ==> results.push_back({class_id, conf, xmin, ymin, xmax, ymax, class_name})
    // ...
    return 0;
}

void Yolo11Parser::publish_to_ros(
    rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr pub,
    const std::vector<DetResult>& results,
    const rclcpp::Time& stamp)
{
    ai_msgs::msg::PerceptionTargets msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "camera";
    msg.fps = 0; // 可选

    for (const auto& det : results) {
        ai_msgs::msg::PerceptionTarget target;
        target.type = det.class_name;
        target.track_id = 0;

        ai_msgs::msg::Roi roi;
        roi.type = "";
        roi.rect.x_offset = static_cast<uint32_t>(std::max(det.xmin, 0.f));
        roi.rect.y_offset = static_cast<uint32_t>(std::max(det.ymin, 0.f));
        roi.rect.width    = static_cast<uint32_t>(std::max(det.xmax - det.xmin, 0.f));
        roi.rect.height   = static_cast<uint32_t>(std::max(det.ymax - det.ymin, 0.f));
        roi.confidence = det.confidence;
        roi.rect.do_rectify = false;

        target.rois.push_back(roi);
        msg.targets.push_back(target);
    }
    pub->publish(msg);
}

auto pub = node->create_publisher<ai_msgs::msg::PerceptionTargets>("/racing_obstacle_detection", 10);

// .... 你的推理代码得到outputs[6] ...

Yolo11Parser parser;
std::vector<DetResult> dets;
parser.postprocess(outputs, 6, input_W, input_H, SCORE_THRESHOLD, NMS_THRESHOLD, object_names, dets);
parser.publish_to_ros(pub, dets, node->now());
