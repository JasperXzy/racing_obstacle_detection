#include "racing_obstacle_detection/parser.h"
#include "racing_obstacle_detection/image_utils.h"
#include <rclcpp/rclcpp.hpp>
#include <hbm_img_msgs/msg/hbm_msg1080_p.hpp>
#include <chrono>

class ImageNV12Subscriber : public rclcpp::Node
{
public:
    ImageNV12Subscriber() : Node("image_nv12_subscriber"), frame_count_(0)
    {
        sub_ = this->create_subscription<hbm_img_msgs::msg::HbmMsg1080P>(
            "/hbmem_img", 
            rclcpp::SensorDataQoS(),
            std::bind(&ImageNV12Subscriber::img_callback, this, std::placeholders::_1));
            last_time_ = std::chrono::steady_clock::now();
    }

private:
    void img_callback(const hbm_img_msgs::msg::HbmMsg1080P::SharedPtr msg)
    {
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
        
        // LetterBox Debug
        // cv::Mat yuv(dst_h * 3 / 2, dst_w, CV_8UC1, output_nv12.data());
        // cv::Mat bgr;
        // cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR_NV12);
        // cv::imwrite("debug_letterbox.jpg", bgr);

        ++frame_count_;
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - last_time_).count();
        if (duration >= 1) {
            RCLCPP_INFO(this->get_logger(), "Frame rate: %.2f fps",
                  static_cast<float>(frame_count_) / duration);
            frame_count_ = 0;
            last_time_ = now;
        }
    }

    rclcpp::Subscription<hbm_img_msgs::msg::HbmMsg1080P>::SharedPtr sub_;
    size_t frame_count_;
    std::chrono::steady_clock::time_point last_time_;
};

int main(int argc, char * argv[]) {
    RacingObstacleDetection obstacleDetector;
    obstacleDetector.load_config();
    int code = obstacleDetector.load_bin_model();
    std::cout << "[INFO] Racing Obstacle Detection completed with code: " << code << std::endl << std::endl;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageNV12Subscriber>());
    rclcpp::shutdown();
    return 0;
}
