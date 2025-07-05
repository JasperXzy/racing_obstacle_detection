// C/C++ Standard Librarys
#include <iostream>
#include <vector>
#include <algorithm>
#include <chrono>
#include <fstream>

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

class RacingObstacleDetection
{
public:
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
   
    void load_config();
    int load_bin_model();

private:
    int rdk_check_success(int value, const std::string &errmsg);
};
