#ifndef PARSER_H
#define PARSER_H

// C/C++ Standard Librarys
#include <iostream>
#include <vector>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <string>

// Third Party Librarys
#include <opencv2/opencv.hpp>
#include <opencv2/dnn/dnn.hpp>

// RDK BPU libDNN API
#include "dnn/hb_dnn.h"
#include "dnn/hb_dnn_ext.h"
#include "dnn/plugin/hb_dnn_layer.h"
#include "dnn/plugin/hb_dnn_plugin.h"
#include "dnn/hb_sys.h"

#include <nlohmann/json.hpp>
 
struct DetectedObject {
    std::string class_name;
    float confidence;
    uint32_t x;
    uint32_t y;
    uint32_t width;
    uint32_t height;
};

class RacingObstacleDetection
{
public:
    const std::vector<DetectedObject>& get_detected_objects() const;

    void load_config();
    int load_bin_model();
    void detect(uint8_t* ynv12);
    int postprocessing(float x_shift, float y_shift, float x_scale, float y_scale, int src_w, int src_h);
    void release_model();

private:
    std::vector<DetectedObject> detected_objects_;
    std::string model_file;
    int class_num;
    std::string dnn_parser;
    std::vector<std::string> cls_names_list;
    int preprocess_type;
    float nms_threshold;
    float score_threshold;
    int nms_top_k;
    int reg;
    float font_size;
    float font_thickness;
    float line_size;
    int32_t output_count = 0;
    int32_t input_H, input_W;
    int order[6] = {0, 1, 2, 3, 4, 5};

    hbDNNHandle_t dnn_handle;
    hbPackedDNNHandle_t packed_dnn_handle;
    hbDNNTensorProperties input_properties;
    hbDNNTensor input;
    hbDNNTensor* output;
    hbDNNTaskHandle_t task_handle = nullptr;
    std::vector<std::vector<cv::Rect2d>> bboxes;
    std::vector<std::vector<float>> scores;
    std::vector<std::vector<int>> indices;

    int rdk_check_success(int value, const std::string &errmsg);
};

#endif // PARSER_H
