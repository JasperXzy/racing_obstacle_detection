#include "racing_obstacle_detection/parser.h"
#include "racing_obstacle_detection/image_utils.h"
#include <rclcpp/rclcpp.hpp>
#include <hbm_img_msgs/msg/hbm_msg1080_p.hpp>
#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include "ai_msgs/msg/perception_targets.hpp"
#include "ai_msgs/msg/roi.hpp"
#include "sensor_msgs/msg/region_of_interest.hpp"

class ImageNV12Subscriber : public rclcpp::Node
{
public:
    ImageNV12Subscriber(RacingObstacleDetection& detector) 
        : Node("image_nv12_subscriber"), 
          frame_count_(0),
          detector_(detector)
    {
        publisher_ = this->create_publisher<ai_msgs::msg::PerceptionTargets>(
            "/racing_obstacle_detection", 10);

        sub_ = this->create_subscription<hbm_img_msgs::msg::HbmMsg1080P>(
            "/hbmem_img", 
            rclcpp::SensorDataQoS(),
            std::bind(&ImageNV12Subscriber::img_callback, this, std::placeholders::_1));
            last_time_ = std::chrono::steady_clock::now();

        // cv::namedWindow("Obstacle Detection", cv::WINDOW_AUTOSIZE); 
    }

    // ~ImageNV12Subscriber() {
    //     cv::destroyWindow("Obstacle Detection"); 
    // }
 
private:
    void img_callback(const hbm_img_msgs::msg::HbmMsg1080P::SharedPtr msg)
    {
        ++frame_count_; 

        auto& frame = msg->data;
        int src_w = 640, src_h = 480;
        int dst_w = 640, dst_h = 640;
 
        std::vector<uint8_t> output_nv12(dst_w * dst_h * 3 / 2);
 
        int x_shift, y_shift;
        float x_scale, y_scale;
 
        letterbox_nv12(
            frame.data(), src_w, src_h, 
            output_nv12.data(), dst_w, dst_h,
            x_shift, y_shift, x_scale, y_scale
        );

        // // 将NV12转换为BGR格式用于显示
        // cv::Mat nv12_mat(dst_h * 3 / 2, dst_w, CV_8UC1, output_nv12.data());
        // cv::Mat bgr_mat;
        // cv::cvtColor(nv12_mat, bgr_mat, cv::COLOR_YUV2BGR_NV12);
        
        detector_.detect(output_nv12.data());
        detector_.postprocessing(x_shift, y_shift, x_scale, y_scale, src_w, src_h);
 
        // // 获取检测结果并在图像上绘制
        // const auto& objects = detector_.get_detected_objects();
        // for (const auto& obj : objects) {

        //     // 将原始图像坐标转换为letterbox图像坐标
        //     int box_x = static_cast<int>(obj.x * x_scale + x_shift);
        //     int box_y = static_cast<int>(obj.y * y_scale + y_shift);
        //     int box_width = static_cast<int>(obj.width * x_scale);
        //     int box_height = static_cast<int>(obj.height * y_scale);

        //     // 确保坐标在图像范围内
        //     box_x = std::max(0, std::min(box_x, dst_w - 1));
        //     box_y = std::max(0, std::min(box_y, dst_h - 1));
        //     box_width = std::max(1, std::min(box_width, dst_w - box_x));
        //     box_height = std::max(1, std::min(box_height, dst_h - box_y));
 
        //     // 绘制边界框
        //     cv::rectangle(bgr_mat, 
        //                   cv::Point(box_x, box_y), 
        //                   cv::Point(box_x + box_width, box_y + box_height),
        //                   cv::Scalar(0, 255, 0),  
        //                   2);
            
        //     // 创建标签文本 (类别 + 置信度)
        //     std::string label = obj.class_name + ": " + std::to_string(obj.confidence);
        //     label = label.substr(0, label.find(".") + 3); 
            
        //     // 在框上方绘制标签背景
        //     int baseline;
        //     cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
        //     int text_y = std::max(box_y - 5, text_size.height + 5); 
            
        //     cv::rectangle(bgr_mat, 
        //                   cv::Point(box_x, text_y - text_size.height - 5),
        //                   cv::Point(box_x + text_size.width, text_y),
        //                   cv::Scalar(0, 255, 0),
        //                   cv::FILLED);
            
        //     // 绘制标签文本
        //     cv::putText(bgr_mat, label,
        //                 cv::Point(box_x, text_y - 5),
        //                 cv::FONT_HERSHEY_SIMPLEX, 0.5,
        //                 cv::Scalar(0, 0, 0),  // Black text
        //                 1);
        // }
        
        // // 显示处理后的图像
        // cv::imshow("Obstacle Detection", bgr_mat);
        // cv::waitKey(1);

        publish_detection_results();

        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - last_time_).count();
        if (duration >= 1) {
            RCLCPP_INFO(this->get_logger(), "Frame rate: %.2f fps, last frame_id published: %lu",
                  static_cast<float>(frame_count_) / duration, frame_count_); 
            frame_count_ = 0; // 重置帧计数器用于下一个周期的FPS计算
            last_time_ = now;
        }
    }

    void publish_detection_results() {
        auto targets_msg = ai_msgs::msg::PerceptionTargets();
        
        targets_msg.header.stamp = this->now();
        targets_msg.header.frame_id = std::to_string(frame_count_); 
        
        const auto& objects = detector_.get_detected_objects();
        for (const auto& obj : objects) {
            ai_msgs::msg::Target target;
            target.type = obj.class_name;
            target.track_id = 0; 
            
            ai_msgs::msg::Roi roi;
            roi.type = "rect";
            roi.rect.x_offset = obj.x;
            roi.rect.y_offset = obj.y;
            roi.rect.height = obj.height;
            roi.rect.width = obj.width;
            roi.rect.do_rectify = false;
            roi.confidence = obj.confidence;
            
            target.rois.push_back(roi);
            targets_msg.targets.push_back(target);
        }
        
        publisher_->publish(targets_msg);
    }
 
    rclcpp::Subscription<hbm_img_msgs::msg::HbmMsg1080P>::SharedPtr sub_;
    size_t frame_count_; 
    std::chrono::steady_clock::time_point last_time_;
    RacingObstacleDetection& detector_; 
    rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
    RacingObstacleDetection obstacleDetector;
    obstacleDetector.load_config();
    int code = obstacleDetector.load_bin_model();
    std::cout << "[INFO] Racing Obstacle Detection Init completed with code: " << code << std::endl << std::endl;
    std::cout << "================================================" << std::endl << std::endl;

    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ImageNV12Subscriber>(obstacleDetector);
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
