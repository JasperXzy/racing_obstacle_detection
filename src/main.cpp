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
      "/hbmem_img", 10,
      std::bind(&ImageNV12Subscriber::img_callback, this, std::placeholders::_1));
    last_time_ = std::chrono::steady_clock::now();
  }

private:
  void img_callback(const hbm_img_msgs::msg::HbmMsg1080P::SharedPtr msg)
  {
    size_t img_size = msg->data.size();

    ++frame_count_;
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - last_time_).count();
    if (duration >= 1) {
      RCLCPP_INFO(this->get_logger(), "Frame rate: %.2f fps, Last image size: %zu bytes",
                  static_cast<float>(frame_count_) / duration, img_size);
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
