#include "racing_obstacle_detection/parser.h"

void RacingObstacleDetection::load_config()
{
    std::ifstream config_file("config/yolov8.json");
    if (!config_file.is_open())
    {
        std::cerr << "Failed to open config file." << std::endl;
        return;
    }
 
    nlohmann::json config;
    config_file >> config;
 
    model_file = config["model_file"];
    class_num = config["class_num"];
    dnn_parser = config["dnn_Parser"];
    cls_names_list = config["cls_names_list"].get<std::vector<std::string>>();
    preprocess_type = config["preprocess_type"];
    nms_threshold = config["nms_threshold"];
    score_threshold = config["score_threshold"];
    nms_top_k = config["nms_top_k"];
    reg = config["reg"];
    font_size = config["font_size"];
    font_thickness = config["font_thickness"];
    line_size = config["line_size"];

    config_file.close();
    
    std::cout << "Model File: " << model_file << std::endl;
    std::cout << "DNN Parser: " << dnn_parser << std::endl;
    std::cout << "Class Number: " << class_num << std::endl;
    std::cout << "Class Names List: ";
    for (const auto& name : cls_names_list)
    {
        std::cout << name << " ";
    }
    std::cout << std::endl;
    if (preprocess_type == 0)
    {
        std::cout << "Preprocess Type: Resize" << std::endl;
    }
    else if (preprocess_type == 1)
    {
        std::cout << "Preprocess Type: Letterbox" << std::endl;
    }
    std::cout << "NMS Threshold: " << nms_threshold << std::endl;
    std::cout << "Score Threshold: " << score_threshold << std::endl;
    std::cout << "NMS Top K: " << nms_top_k << std::endl;
    std::cout << "Regression: " << reg << std::endl;
    std::cout << "Font Size: " << font_size << std::endl;
    std::cout << "Font Thickness: " << font_thickness << std::endl;
    std::cout << "Line Size: " << line_size << std::endl;
    std::cout << "Load Config Successfully!" << std::endl;
}
