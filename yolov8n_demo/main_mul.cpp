/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

Copyright (c) 2024，WuChao D-Robotics.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// D-Robotics *.bin 模型路径
#define MODEL_PATH "/userdata/racing_obstacle_detection/yolov8n_demo/yolov8n_detect_bayese_640x640_nv12_modified_cv2_cv3.bin"

// 输入图片文件夹路径
#define INPUT_IMAGES_DIR "/userdata/racing_obstacle_detection/yolov8n_demo/images"

// 输出结果保存文件夹路径
#define OUTPUT_RESULTS_DIR "/userdata/racing_obstacle_detection/yolov8n_demo/results/"

// 前处理方式选择
#define RESIZE_TYPE 0
#define LETTERBOX_TYPE 1
#define PREPROCESS_TYPE LETTERBOX_TYPE

// 模型参数
#define CLASSES_NUM 1
#define NMS_THRESHOLD 0.5
#define SCORE_THRESHOLD 0.25
#define NMS_TOP_K 300
#define REG 16

// 渲染参数
#define FONT_SIZE 1.0
#define FONT_THICKNESS 1.0
#define LINE_SIZE 2.0

// 头文件
#include <iostream>
#include <vector>
#include <algorithm>
#include <chrono>
#include <dirent.h>
#include <sys/stat.h>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include "dnn/hb_dnn.h"
#include "dnn/hb_dnn_ext.h"
#include "dnn/plugin/hb_dnn_layer.h"
#include "dnn/plugin/hb_dnn_plugin.h"
#include "dnn/hb_sys.h"

#define RDK_CHECK_SUCCESS(value, errmsg)                                         \
    do                                                                           \
    {                                                                            \
        auto ret_code = value;                                                   \
        if (ret_code != 0)                                                       \
        {                                                                        \
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << std::endl; \
            std::cout << errmsg << ", error code:" << ret_code << std::endl;     \
            return ret_code;                                                     \
        }                                                                        \
    } while (0);

// 类别名称
std::vector<std::string> object_names = {"obstacle"};

// 获取目录中的文件列表
std::vector<std::string> getFilesInDirectory(const std::string& directory) {
    std::vector<std::string> files;
    DIR* dir;
    struct dirent* ent;
    if ((dir = opendir(directory.c_str())) != nullptr) {
        while ((ent = readdir(dir)) != nullptr) {
            std::string filename = ent->d_name;
            if (filename != "." && filename != "..") {
                files.push_back(directory + "/" + filename);
            }
        }
        closedir(dir);
    }
    return files;
}

// 创建目录（如果不存在）
void createDirectoryIfNotExists(const std::string& path) {
    struct stat info;
    if (stat(path.c_str(), &info) != 0) {
        mkdir(path.c_str(), 0777);
    } else if (!(info.st_mode & S_IFDIR)) {
        std::cerr << path << " exists but is not a directory!" << std::endl;
    }
}

int main()
{
    // 创建输出目录
    createDirectoryIfNotExists(OUTPUT_RESULTS_DIR);

    // 0. 加载bin模型
    auto begin_time = std::chrono::system_clock::now();
    hbPackedDNNHandle_t packed_dnn_handle;
    const char *model_file_name = MODEL_PATH;
    RDK_CHECK_SUCCESS(
        hbDNNInitializeFromFiles(&packed_dnn_handle, &model_file_name, 1),
        "hbDNNInitializeFromFiles failed");
    std::cout << "\033[31m Load D-Robotics Quantize model time = " << std::fixed << std::setprecision(2) << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - begin_time).count() / 1000.0 << " ms\033[0m" << std::endl;

    // 1. 打印模型信息
    const char **model_name_list;
    int model_count = 0;
    RDK_CHECK_SUCCESS(
        hbDNNGetModelNameList(&model_name_list, &model_count, packed_dnn_handle),
        "hbDNNGetModelNameList failed");

    if (model_count > 1) {
        std::cout << "This model file have more than 1 model, only use model 0.";
    }
    const char *model_name = model_name_list[0];
    std::cout << "[model name]: " << model_name << std::endl;

    hbDNNHandle_t dnn_handle;
    RDK_CHECK_SUCCESS(
        hbDNNGetModelHandle(&dnn_handle, packed_dnn_handle, model_name),
        "hbDNNGetModelHandle failed");

    // 2. 获取输入输出属性
    int32_t input_count = 0;
    RDK_CHECK_SUCCESS(
        hbDNNGetInputCount(&input_count, dnn_handle),
        "hbDNNGetInputCount failed");

    hbDNNTensorProperties input_properties;
    RDK_CHECK_SUCCESS(
        hbDNNGetInputTensorProperties(&input_properties, dnn_handle, 0),
        "hbDNNGetInputTensorProperties failed");

    int32_t input_H = input_properties.validShape.dimensionSize[2];
    int32_t input_W = input_properties.validShape.dimensionSize[3];
    std::cout << "input tensor valid shape: (" 
              << input_properties.validShape.dimensionSize[0] << ", "
              << input_properties.validShape.dimensionSize[1] << ", "
              << input_H << ", " << input_W << ")" << std::endl;

    int32_t output_count = 0;
    RDK_CHECK_SUCCESS(
        hbDNNGetOutputCount(&output_count, dnn_handle),
        "hbDNNGetOutputCount failed");

    // 3. 确定输出顺序
    int order[6] = {0, 1, 2, 3, 4, 5};
    int32_t H_8 = input_H / 8;
    int32_t H_16 = input_H / 16;
    int32_t H_32 = input_H / 32;
    int32_t W_8 = input_W / 8;
    int32_t W_16 = input_W / 16;
    int32_t W_32 = input_W / 32;
    int32_t order_we_want[6][3] = {
        {H_8, W_8, CLASSES_NUM}, {H_8, W_8, 64},
        {H_16, W_16, CLASSES_NUM}, {H_16, W_16, 64},
        {H_32, W_32, CLASSES_NUM}, {H_32, W_32, 64}
    };
    
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            hbDNNTensorProperties output_properties;
            hbDNNGetOutputTensorProperties(&output_properties, dnn_handle, j);
            int32_t h = output_properties.validShape.dimensionSize[1];
            int32_t w = output_properties.validShape.dimensionSize[2];
            int32_t c = output_properties.validShape.dimensionSize[3];
            if (h == order_we_want[i][0] && w == order_we_want[i][1] && c == order_we_want[i][2]) {
                order[i] = j;
                break;
            }
        }
    }

    std::cout << "Outputs order: {";
    for (int i = 0; i < 6; i++) {
        std::cout << order[i] << (i < 5 ? ", " : "");
    }
    std::cout << "}" << std::endl;

    // 4. 获取所有图片文件
    std::vector<std::string> image_files = getFilesInDirectory(INPUT_IMAGES_DIR);
    if (image_files.empty()) {
        std::cerr << "No images found in directory: " << INPUT_IMAGES_DIR << std::endl;
        return -1;
    }
    std::cout << "Found " << image_files.size() << " images in directory." << std::endl;

    // 5. 循环处理每张图片
    for (const auto& image_path : image_files) {
        auto total_begin = std::chrono::system_clock::now();
        std::cout << "\nProcessing image: " << image_path << std::endl;

        // 5.1 读取图像
        cv::Mat img = cv::imread(image_path);
        if (img.empty()) {
            std::cerr << "Failed to load image: " << image_path << std::endl;
            continue;
        }
        std::cout << "Original image size: " << img.cols << "x" << img.rows << std::endl;

        // 5.2 前处理
        auto preprocess_begin = std::chrono::system_clock::now();
        float y_scale = 1.0;
        float x_scale = 1.0;
        int x_shift = 0;
        int y_shift = 0;
        cv::Mat resize_img;
        
        if (PREPROCESS_TYPE == LETTERBOX_TYPE) {
            // LetterBox处理
            x_scale = std::min(1.0f * input_H / img.rows, 1.0f * input_W / img.cols);
            y_scale = x_scale;
            
            int new_w = img.cols * x_scale;
            x_shift = (input_W - new_w) / 2;
            int new_h = img.rows * y_scale;
            y_shift = (input_H - new_h) / 2;
            
            cv::resize(img, resize_img, cv::Size(new_w, new_h));
            cv::copyMakeBorder(resize_img, resize_img, 
                               y_shift, input_H - new_h - y_shift,
                               x_shift, input_W - new_w - x_shift,
                               cv::BORDER_CONSTANT, cv::Scalar(127, 127, 127));
        } else {
            // Resize处理
            cv::resize(img, resize_img, cv::Size(input_W, input_H));
            y_scale = 1.0f * input_H / img.rows;
            x_scale = 1.0f * input_W / img.cols;
        }
        
        auto preprocess_time = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now() - preprocess_begin).count() / 1000.0;
        std::cout << "\033[31m Preprocess time: " << preprocess_time << " ms\033[0m" << std::endl;
        std::cout << "Scaling factors: x=" << x_scale << ", y=" << y_scale 
                  << ", Shifts: x=" << x_shift << ", y=" << y_shift << std::endl;

        // 5.3 转换为NV12格式
        auto convert_begin = std::chrono::system_clock::now();
        cv::Mat yuv_mat;
        cv::cvtColor(resize_img, yuv_mat, cv::COLOR_BGR2YUV_I420);
        
        cv::Mat img_nv12(input_H * 3 / 2, input_W, CV_8UC1);
        uint8_t* ynv12 = img_nv12.ptr<uint8_t>();
        uint8_t* yuv = yuv_mat.ptr<uint8_t>();
        
        int y_size = input_H * input_W;
        int uv_size = input_H * input_W / 4;
        memcpy(ynv12, yuv, y_size);
        
        uint8_t* u_data = yuv + y_size;
        uint8_t* v_data = u_data + uv_size;
        uint8_t* nv12_uv = ynv12 + y_size;
        
        for (int i = 0; i < uv_size; i++) {
            *nv12_uv++ = *u_data++;
            *nv12_uv++ = *v_data++;
        }
        
        auto convert_time = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now() - convert_begin).count() / 1000.0;
        std::cout << "\033[31m BGR to NV12 conversion time: " << convert_time << " ms\033[0m" << std::endl;

        // 5.4 准备输入张量
        hbDNNTensor input_tensor;
        input_tensor.properties = input_properties;
        hbSysAllocCachedMem(&input_tensor.sysMem[0], input_properties.alignedByteSize);
        memcpy(input_tensor.sysMem[0].virAddr, ynv12, input_properties.alignedByteSize);
        hbSysFlushMem(&input_tensor.sysMem[0], HB_SYS_MEM_CACHE_CLEAN);

        // 5.5 准备输出张量
        hbDNNTensor* output_tensors = new hbDNNTensor[output_count];
        hbDNNTensor* output_tensor_ptrs[output_count];
        
        for (int i = 0; i < output_count; i++) {
            hbDNNGetOutputTensorProperties(&output_tensors[i].properties, dnn_handle, i);
            hbSysAllocCachedMem(&output_tensors[i].sysMem[0], output_tensors[i].properties.alignedByteSize);
            output_tensor_ptrs[i] = &output_tensors[i];
        }

        // 5.6 推理
        auto inference_begin = std::chrono::system_clock::now();
        hbDNNTaskHandle_t task_handle = nullptr;
        hbDNNInferCtrlParam infer_ctrl_param;
        HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&infer_ctrl_param);
        
        int ret = hbDNNInfer(&task_handle, output_tensor_ptrs, &input_tensor, dnn_handle, &infer_ctrl_param);
        if (ret != 0) {
            std::cerr << "[ERROR] hbDNNInfer failed for image: " << image_path 
                      << ", error code: " << ret << std::endl;
            hbSysFreeMem(&input_tensor.sysMem[0]);
            for (int i = 0; i < output_count; i++) {
                hbSysFreeMem(&output_tensors[i].sysMem[0]);
            }
            delete[] output_tensors;
            continue;
        }
        
        hbDNNWaitTaskDone(task_handle, 0);
        hbDNNReleaseTask(task_handle);
        
        auto inference_time = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now() - inference_begin).count() / 1000.0;
        std::cout << "\033[31m Inference time: " << inference_time << " ms\033[0m" << std::endl;

        // 5.7 后处理
        auto postprocess_begin = std::chrono::system_clock::now();
        float CONF_THRES_RAW = -log(1 / SCORE_THRESHOLD - 1);
        std::vector<std::vector<cv::Rect2d>> bboxes(CLASSES_NUM);
        std::vector<std::vector<float>> scores(CLASSES_NUM);
 
        // 定义处理特征图的函数
        auto process_feature_map = [&](int cls_index, int bbox_index, int stride, int height, int width) {
            // 刷新内存
            hbSysFlushMem(&(output_tensors[cls_index].sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
            hbSysFlushMem(&(output_tensors[bbox_index].sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
            
            // 获取数据指针
            auto* cls_raw = reinterpret_cast<float*>(output_tensors[cls_index].sysMem[0].virAddr);
            auto* bbox_raw = reinterpret_cast<int32_t*>(output_tensors[bbox_index].sysMem[0].virAddr);
            auto* bbox_scale = reinterpret_cast<float*>(output_tensors[bbox_index].properties.scale.scaleData);
            
            // 处理每个位置
            for (int h = 0; h < height; h++) {
                for (int w = 0; w < width; w++) {
                    float* cur_cls_raw = cls_raw;
                    int32_t* cur_bbox_raw = bbox_raw;
                    
                    // 找到最高分数的类别
                    int cls_id = 0;
                    for (int i = 1; i < CLASSES_NUM; i++) {
                        if (cur_cls_raw[i] > cur_cls_raw[cls_id]) {
                            cls_id = i;
                        }
                    }
                    
                    // 检查置信度阈值
                    if (cur_cls_raw[cls_id] < CONF_THRES_RAW) {
                        cls_raw += CLASSES_NUM;
                        bbox_raw += REG * 4;
                        continue;
                    }
                    
                    // 计算分数
                    float score = 1.0f / (1.0f + std::exp(-cur_cls_raw[cls_id]));
                    
                    // 解码边界框
                    float ltrb[4];
                    for (int i = 0; i < 4; i++) {
                        float sum = 0.0f;
                        float weighted_sum = 0.0f;
                        for (int j = 0; j < REG; j++) {
                            float dfl = std::exp(static_cast<float>(cur_bbox_raw[REG * i + j]) * bbox_scale[REG * i + j]);
                            weighted_sum += dfl * j;
                            sum += dfl;
                        }
                        ltrb[i] = weighted_sum / sum;
                    }
                    
                    // 检查边界框有效性
                    if (ltrb[0] + ltrb[2] <= 0 || ltrb[1] + ltrb[3] <= 0) {
                        cls_raw += CLASSES_NUM;
                        bbox_raw += REG * 4;
                        continue;
                    }
                    
                    // 计算边界框坐标
                    float x1 = (w + 0.5f - ltrb[0]) * stride;
                    float y1 = (h + 0.5f - ltrb[1]) * stride;
                    float x2 = (w + 0.5f + ltrb[2]) * stride;
                    float y2 = (h + 0.5f + ltrb[3]) * stride;
                    
                    // 存储结果
                    bboxes[cls_id].emplace_back(x1, y1, x2 - x1, y2 - y1);
                    scores[cls_id].push_back(score);
                    
                    cls_raw += CLASSES_NUM;
                    bbox_raw += REG * 4;
                }
            }
        };
        
        // 处理不同尺度的特征图
        process_feature_map(order[0], order[1], 8, H_8, W_8);  // 小目标
        process_feature_map(order[2], order[3], 16, H_16, W_16); // 中目标
        process_feature_map(order[4], order[5], 32, H_32, W_32); // 大目标
        
        // NMS处理
        std::vector<std::vector<int>> indices(CLASSES_NUM);
        for (int i = 0; i < CLASSES_NUM; i++) {
            cv::dnn::NMSBoxes(bboxes[i], scores[i], SCORE_THRESHOLD, NMS_THRESHOLD, indices[i], 1.0f, NMS_TOP_K);
        }
        
        auto postprocess_time = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now() - postprocess_begin).count() / 1000.0;
        std::cout << "\033[31m Postprocess time: " << postprocess_time << " ms\033[0m" << std::endl;
 

        // 5.8 绘制检测结果
        auto draw_begin = std::chrono::system_clock::now();
        for (int cls_id = 0; cls_id < CLASSES_NUM; cls_id++) {
            for (const auto& idx : indices[cls_id]) {
                // 将边界框坐标转换回原始图像空间
                float x1 = (bboxes[cls_id][idx].x - x_shift) / x_scale;
                float y1 = (bboxes[cls_id][idx].y - y_shift) / y_scale;
                float x2 = x1 + bboxes[cls_id][idx].width / x_scale;
                float y2 = y1 + bboxes[cls_id][idx].height / y_scale;
                
                // 确保坐标在图像范围内
                x1 = std::max(0.0f, std::min(x1, static_cast<float>(img.cols - 1)));
                y1 = std::max(0.0f, std::min(y1, static_cast<float>(img.rows - 1)));
                x2 = std::max(0.0f, std::min(x2, static_cast<float>(img.cols - 1)));
                y2 = std::max(0.0f, std::min(y2, static_cast<float>(img.rows - 1)));
                
                float score = scores[cls_id][idx];
                std::string name = object_names[cls_id];
                
                // 绘制边界框
                cv::rectangle(img, cv::Point(x1, y1), cv::Point(x2, y2), 
                             cv::Scalar(0, 0, 255), LINE_SIZE);
                
                // 绘制类别标签和置信度
                std::string label = name + ": " + std::to_string(static_cast<int>(score * 100)) + "%";
                int baseLine;
                cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 
                                                    FONT_SIZE, FONT_THICKNESS, &baseLine);
                cv::rectangle(img, cv::Point(x1, y1 - labelSize.height - baseLine),
                             cv::Point(x1 + labelSize.width, y1), 
                             cv::Scalar(0, 0, 255), cv::FILLED);
                cv::putText(img, label, cv::Point(x1, y1 - baseLine), 
                           cv::FONT_HERSHEY_SIMPLEX, FONT_SIZE, 
                           cv::Scalar(255, 255, 255), FONT_THICKNESS);
                
                // 打印检测结果
                std::cout << "Detected " << name << " at (" 
                          << x1 << ", " << y1 << ") - (" 
                          << x2 << ", " << y2 << ") with confidence " 
                          << score << std::endl;
            }
        }
        
        auto draw_time = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now() - draw_begin).count() / 1000.0;
        std::cout << "\033[31m Drawing time: " << draw_time << " ms\033[0m" << std::endl;
        
        // 5.9 保存结果
        std::string filename = image_path.substr(image_path.find_last_of("/\\") + 1);
        std::string output_path = OUTPUT_RESULTS_DIR + filename;
        cv::imwrite(output_path, img);
        std::cout << "Saved result to: " << output_path << std::endl;
        
        // 5.10 释放资源
        hbSysFreeMem(&input_tensor.sysMem[0]);
        for (int i = 0; i < output_count; i++) {
            hbSysFreeMem(&output_tensors[i].sysMem[0]);
        }
        delete[] output_tensors;
        
        auto total_time = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now() - total_begin).count() / 1000.0;
        std::cout << "\033[31m Total processing time for " << image_path 
                  << ": " << total_time << " ms\033[0m" << std::endl;
    }
    
    // 6. 释放模型
    hbDNNRelease(packed_dnn_handle);
    
    return 0;
}
