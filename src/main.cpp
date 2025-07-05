#include "racing_obstacle_detection/parser.h"

int main() {
    RacingObstacleDetection obstacleDetector;
    obstacleDetector.load_config();
    int code = obstacleDetector.load_bin_model();
    std::cout << "[INFO] Racing Obstacle Detection completed with code: " << code << std::endl;

    return 0;
}