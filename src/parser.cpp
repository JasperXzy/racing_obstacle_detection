#include "racing_obstacle_detection/parser.h"

int RacingObstacleDetection::rdk_check_success(int value, const std::string &errmsg)
{
    if (value != 0)
    {
        std::cerr << "[ERROR] " << errmsg << ", error code: " << value << std::endl;
        return value;
    }
    return 0;
}

void RacingObstacleDetection::load_config()
{
    std::cout << "================================================" << std::endl;
    std::cout << "[INFO] Loading Configuration From config/yolov8.json" << std::endl;
    std::ifstream config_file("config/yolov8.json");
    if (!config_file.is_open())
    {
        std::cerr << "[ERROR] Failed to open config file." << std::endl;
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
    
    std::cout << "[INFO] Model File: " << model_file << std::endl;
    std::cout << "[INFO] DNN Parser: " << dnn_parser << std::endl;
    std::cout << "[INFO] Class Number: " << class_num << std::endl;
    std::cout << "[INFO] Class Names List: ";
    for (const auto& name : cls_names_list)
    {
        std::cout << name << " ";
    }
    std::cout << std::endl;
    if (preprocess_type == 0)
    {
        std::cout << "[INFO] Preprocess Type: Resize" << std::endl;
    }
    else if (preprocess_type == 1)
    {
        std::cout << "[INFO] Preprocess Type: Letterbox" << std::endl;
    }
    std::cout << "[INFO] NMS Threshold: " << nms_threshold << std::endl;
    std::cout << "[INFO] Score Threshold: " << score_threshold << std::endl;
    std::cout << "[INFO] NMS Top K: " << nms_top_k << std::endl;
    std::cout << "[INFO] Regression: " << reg << std::endl;
    std::cout << "[INFO] Font Size: " << font_size << std::endl;
    std::cout << "[INFO] Font Thickness: " << font_thickness << std::endl;
    std::cout << "[INFO] Line Size: " << line_size << std::endl;
    std::cout << "[INFO] Load Configuration Successfully!" << std::endl;
    std::cout << "================================================" << std::endl << std::endl;
}

int RacingObstacleDetection::load_bin_model()
{
    std::cout << "================================================" << std::endl;
    std::cout << "[INFO] Loading Binary Model From: " << model_file << std::endl;
    std::cout << "[INFO] OpenCV Version: " << CV_VERSION << std::endl;
    
    auto begin_time = std::chrono::system_clock::now();
    const char *model_file_name = model_file.c_str();
    rdk_check_success(
        hbDNNInitializeFromFiles(&packed_dnn_handle, &model_file_name, 1),
        "hbDNNInitializeFromFiles failed");
    std::cout << "\033[31m[INFO] Load D-Robotics Quantize model time = " << std::fixed << std::setprecision(2)
              << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - begin_time).count() / 1000.0 << " ms\033[0m" << std::endl;
    
    // 1. 打印模型名称
    const char **model_name_list;
    int model_count = 0;
    rdk_check_success(
        hbDNNGetModelNameList(&model_name_list, &model_count, packed_dnn_handle),
        "hbDNNGetModelNameList failed");

    // 如果这个bin模型有多个打包，则只使用第一个，一般只有一个
    if (model_count > 1)
    {
        std::cout << "[WARN] This model file have more than 1 model, only use model 0.";
    }
    const char *model_name = model_name_list[0];
    std::cout << "[INFO] Model Name: " << model_name << std::endl;

    // 2. 获得Packed模型的第一个模型的handle
    rdk_check_success(
        hbDNNGetModelHandle(&dnn_handle, packed_dnn_handle, model_name),
        "hbDNNGetModelHandle failed");

    // 3. 模型输入检查
    int32_t input_count = 0;
    rdk_check_success(
        hbDNNGetInputCount(&input_count, dnn_handle),
        "hbDNNGetInputCount failed");

    rdk_check_success(
        hbDNNGetInputTensorProperties(&input_properties, dnn_handle, 0),
        "hbDNNGetInputTensorProperties failed");

    // 3.1. D-Robotics YOLOv8 *.bin 模型应该为单输入
    if (input_count > 1)
    {
        std::cout << "[ERROR] Your Model have more than 1 input, please check!" << std::endl;
        return -1;
    }

    // 3.2. D-Robotics YOLOv8 *.bin 模型输入Tensor类型应为nv12
    if (input_properties.tensorType == HB_DNN_IMG_TYPE_NV12)
    {
        std::cout << "[INFO] Input Tensor Type: HB_DNN_IMG_TYPE_NV12" << std::endl;
    }
    else
    {
        std::cout << "[ERROR] Input Tensor Type is not HB_DNN_IMG_TYPE_NV12, please check!" << std::endl;
        return -1;
    }

    // 3.3. D-Robotics YOLOv8 *.bin 模型输入Tensor数据排布应为NCHW
    if (input_properties.tensorLayout == HB_DNN_LAYOUT_NCHW)
    {
        std::cout << "[INFO] Input Tensor Layout: HB_DNN_LAYOUT_NCHW" << std::endl;
    }
    else
    {
        std::cout << "[ERROR] Input Tensor Layout is not HB_DNN_LAYOUT_NCHW, please check!" << std::endl;
        return -1;
    }

    // 3.4. D-Robotics YOLOv8 *.bin 模型输入Tensor数据的valid shape应为(1,3,H,W)
    if (input_properties.validShape.numDimensions == 4)
    {
        input_H = input_properties.validShape.dimensionSize[2];
        input_W = input_properties.validShape.dimensionSize[3];
        std::cout << "[INFO] Input Tensor Valid Shape: (" << input_properties.validShape.dimensionSize[0];
        std::cout << ", " << input_properties.validShape.dimensionSize[1];
        std::cout << ", " << input_H;
        std::cout << ", " << input_W << ")" << std::endl;
    }
    else
    {
        std::cout << "[ERROR] Input Tensor Valid Shape.numDimensions is not 4 such as (1,3,640,640), please check!" << std::endl;
        return -1;
    }

    // 4. 模型输出检查
    rdk_check_success(
        hbDNNGetOutputCount(&output_count, dnn_handle),
        "hbDNNGetOutputCount failed");

    // 4.1. D-Robotics YOLOv8 *.bin 模型应该有6个输出
    if (output_count == 6)
    {
        for (int i = 0; i < 6; i++)
        {
            hbDNNTensorProperties output_properties;
            rdk_check_success(
                hbDNNGetOutputTensorProperties(&output_properties, dnn_handle, i),
                "hbDNNGetOutputTensorProperties failed");
            std::cout << "[INFO] Output[" << i << "] ";
            std::cout << "Valid Shape: (" << output_properties.validShape.dimensionSize[0];
            std::cout << ", " << output_properties.validShape.dimensionSize[1];
            std::cout << ", " << output_properties.validShape.dimensionSize[2];
            std::cout << ", " << output_properties.validShape.dimensionSize[3] << "), ";
            if (output_properties.quantiType == SHIFT)
                std::cout << "QuantiType: SHIFT" << std::endl;
            if (output_properties.quantiType == SCALE)
                std::cout << "QuantiType: SCALE" << std::endl;
            if (output_properties.quantiType == NONE)
                std::cout << "QuantiType: NONE" << std::endl;
        }
    }
    else
    {
        std::cout << "[ERROR] Your Model's outputs num is not 6, please check!" << std::endl;
        return -1;
    }

    // 4.2. 调整输出头顺序的映射
    int32_t H_8 = input_H / 8;
    int32_t H_16 = input_H / 16;
    int32_t H_32 = input_H / 32;
    int32_t W_8 = input_W / 8;
    int32_t W_16 = input_W / 16;
    int32_t W_32 = input_W / 32;
    int32_t order_we_want[6][3] = {
        {H_8, W_8, class_num},       // output[order[3]]: (1, H // 8,  W // 8,  class_num)
        {H_8, W_8, 64},              // output[order[0]]: (1, H // 8,  W // 8,  64)
        {H_16, W_16, class_num},     // output[order[4]]: (1, H // 16, W // 16, class_num)
        {H_16, W_16, 64},            // output[order[1]]: (1, H // 16, W // 16, 64)
        {H_32, W_32, class_num},     // output[order[5]]: (1, H // 32, W // 32, class_num)
        {H_32, W_32, 64},            // output[order[2]]: (1, H // 32, W // 32, 64)
    };
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            hbDNNTensorProperties output_properties;
            rdk_check_success(
                hbDNNGetOutputTensorProperties(&output_properties, dnn_handle, j),
                "hbDNNGetOutputTensorProperties failed");
            int32_t h = output_properties.validShape.dimensionSize[1];
            int32_t w = output_properties.validShape.dimensionSize[2];
            int32_t c = output_properties.validShape.dimensionSize[3];
            if (h == order_we_want[i][0] && w == order_we_want[i][1] && c == order_we_want[i][2])
            {
                order[i] = j;
                break;
            }
        }
    }

    // 4.3. 打印并检查调整后的输出头顺序的映射
    if (order[0] + order[1] + order[2] + order[3] + order[4] + order[5] == 0 + 1 + 2 + 3 + 4 + 5)
    {
        std::cout << "[INFO] Outputs Order Check SUCCESS, continue." << std::endl;
        std::cout << "[INFO] Order = {";
        for (int i = 0; i < 6; i++)
        {
            std::cout << order[i] << ", ";
        }
        std::cout << "}" << std::endl;
    }
    else
    {
        std::cout << "[ERROR] Outputs Order Check FAILED, use default" << std::endl;
        for (int i = 0; i < 6; i++)
            order[i] = i;
    }

    std::cout << "[INFO] Load Binary Model Successfully!" << std::endl;
    std::cout << "================================================" << std::endl << std::endl;

    return 0;
}

void RacingObstacleDetection::detect(uint8_t* ynv12) {
    // 1. 准备输入张量
    task_handle = nullptr;
    input.properties = input_properties;
    hbSysAllocCachedMem(&input.sysMem[0], int(3 * input_H * input_W / 2));
    memcpy(input.sysMem[0].virAddr, ynv12, int(3 * input_H * input_W / 2));
    hbSysFlushMem(&input.sysMem[0], HB_SYS_MEM_CACHE_CLEAN);
 
    // 2. 准备输出张量
    output = new hbDNNTensor[output_count];
    for (int i = 0; i < 6; i++) {
        hbDNNTensorProperties& output_properties = output[i].properties;
        hbDNNGetOutputTensorProperties(&output_properties, dnn_handle, i);
        int out_aligned_size = output_properties.alignedByteSize;
        hbSysMem& mem = output[i].sysMem[0];
        hbSysAllocCachedMem(&mem, out_aligned_size);
    }
 
    // 3. 执行推理
    hbDNNInferCtrlParam infer_ctrl_param;
    HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&infer_ctrl_param);
    hbDNNInfer(&task_handle, &output, &input, dnn_handle, &infer_ctrl_param);
    hbDNNWaitTaskDone(task_handle, 0);
}

int RacingObstacleDetection::postprocessing(float x_shift, float y_shift, float x_scale, float y_scale, int src_w, int src_h)
{
    // 1. YOLOv8-Detect 后处理
    float CONF_THRES_RAW = -log(1 / score_threshold - 1);       // 利用反函数作用阈值，利用单调性筛选
    std::vector<std::vector<cv::Rect2d>> bboxes(class_num);     // 每个id的xyhw 信息使用一个std::vector<cv::Rect2d>存储
    std::vector<std::vector<float>> scores(class_num);          // 每个id的score信息使用一个std::vector<float>存储

    // 1.1 小目标特征图
    // output[order[0]]: (1, H // 8,  W // 8,  class_num)
    // output[order[1]]: (1, H // 8,  W // 8,  4 * reg)

    // 1.1.1 检查反量化类型是否符合RDK Model Zoo的README导出的bin模型规范
    if (output[order[0]].properties.quantiType != NONE)
    {
        std::cout << "output[order[0]] QuantiType is not NONE, please check!" << std::endl;
        return -1;
    }
    if (output[order[1]].properties.quantiType != SCALE)
    {
        std::cout << "output[order[1]] QuantiType is not SCALE, please check!" << std::endl;
        return -1;
    }

    // 1.1.2 对缓存的BPU内存进行刷新
    hbSysFlushMem(&(output[order[0]].sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
    hbSysFlushMem(&(output[order[1]].sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);

    // 1.1.3 将BPU推理完的内存地址转换为对应类型的指针
    auto *s_cls_raw = reinterpret_cast<float *>(output[order[0]].sysMem[0].virAddr);
    auto *s_bbox_raw = reinterpret_cast<int32_t *>(output[order[1]].sysMem[0].virAddr);
    auto *s_bbox_scale = reinterpret_cast<float *>(output[order[1]].properties.scale.scaleData);
    for (int h = 0; h < (input_H / 8); h++)
    {
        for (int w = 0; w < (input_W / 8); w++)
        {
            // 1.1.4 取对应H和W位置的C通道, 记为数组的形式
            // cls对应class_num个分数RAW值, 也就是Sigmoid计算之前的值，这里利用函数单调性先筛选, 再计算
            // bbox对应4个坐标乘以reg的RAW值, 也就是DFL计算之前的值, 仅仅分数合格了, 才会进行这部分的计算
            float *cur_s_cls_raw = s_cls_raw;
            int32_t *cur_s_bbox_raw = s_bbox_raw;

            // 1.1.5 找到分数的最大值索引, 如果最大值小于阈值，则舍去
            int cls_id = 0;
            for (int i = 1; i < class_num; i++)
            {
                if (cur_s_cls_raw[i] > cur_s_cls_raw[cls_id])
                {
                    cls_id = i;
                }
            }

            // 1.1.6 不合格则直接跳过, 避免无用的反量化, DFL和dist2bbox计算
            if (cur_s_cls_raw[cls_id] < CONF_THRES_RAW)
            {
                s_cls_raw += class_num;
                s_bbox_raw += reg * 4;
                continue;
            }

            // 1.1.7 计算这个目标的分数
            float score = 1 / (1 + std::exp(-cur_s_cls_raw[cls_id]));

            // 1.1.8 对bbox_raw信息进行反量化, DFL计算
            float ltrb[4], sum, dfl;
            for (int i = 0; i < 4; i++)
            {
                ltrb[i] = 0.;
                sum = 0.;
                for (int j = 0; j < reg; j++)
                {
                    int index_id = reg * i + j;
                    dfl = std::exp(float(cur_s_bbox_raw[index_id]) * s_bbox_scale[index_id]);
                    ltrb[i] += dfl * j;
                    sum += dfl;
                }
                ltrb[i] /= sum;
            }

            // 1.1.9 剔除不合格的框   if(x1 >= x2 || y1 >=y2) continue;
            if (ltrb[2] + ltrb[0] <= 0 || ltrb[3] + ltrb[1] <= 0)
            {
                s_cls_raw += class_num;
                s_bbox_raw += reg * 4;
                continue;
            }

            // 1.1.10 dist 2 bbox (ltrb 2 xyxy)
            float x1 = (w + 0.5 - ltrb[0]) * 8.0;
            float y1 = (h + 0.5 - ltrb[1]) * 8.0;
            float x2 = (w + 0.5 + ltrb[2]) * 8.0;
            float y2 = (h + 0.5 + ltrb[3]) * 8.0;

            // 1.1.11 对应类别加入到对应的std::vector中
            bboxes[cls_id].push_back(cv::Rect2d(x1, y1, x2 - x1, y2 - y1));
            scores[cls_id].push_back(score);

            s_cls_raw += class_num;
            s_bbox_raw += reg * 4;
        }
    }

    // 1.2 中目标特征图
    // output[order[2]]: (1, H // 16,  W // 16,  class_num)
    // output[order[3]]: (1, H // 16,  W // 16,  4 * reg)

    // 1.2.1 检查反量化类型是否符合RDK Model Zoo的README导出的bin模型规范
    if (output[order[2]].properties.quantiType != NONE)
    {
        std::cout << "output[order[2]] QuantiType is not NONE, please check!" << std::endl;
        return -1;
    }
    if (output[order[3]].properties.quantiType != SCALE)
    {
        std::cout << "output[order[3]] QuantiType is not SCALE, please check!" << std::endl;
        return -1;
    }

    // 1.2.2 对缓存的BPU内存进行刷新
    hbSysFlushMem(&(output[order[2]].sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
    hbSysFlushMem(&(output[order[3]].sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);

    // 1.2.3 将BPU推理完的内存地址转换为对应类型的指针
    auto *m_cls_raw = reinterpret_cast<float *>(output[order[2]].sysMem[0].virAddr);
    auto *m_bbox_raw = reinterpret_cast<int32_t *>(output[order[3]].sysMem[0].virAddr);
    auto *m_bbox_scale = reinterpret_cast<float *>(output[order[3]].properties.scale.scaleData);
    for (int h = 0; h < (input_H / 16); h++)
    {
        for (int w = 0; w < (input_W / 16); w++)
        {
            // 1.2.4 取对应H和W位置的C通道, 记为数组的形式
            // cls对应class_num个分数RAW值, 也就是Sigmoid计算之前的值，这里利用函数单调性先筛选, 再计算
            // bbox对应4个坐标乘以reg的RAW值, 也就是DFL计算之前的值, 仅仅分数合格了, 才会进行这部分的计算
            float *cur_m_cls_raw = m_cls_raw;
            int32_t *cur_m_bbox_raw = m_bbox_raw;

            // 1.2.5 找到分数的最大值索引, 如果最大值小于阈值，则舍去
            int cls_id = 0;
            for (int i = 1; i < class_num; i++)
            {
                if (cur_m_cls_raw[i] > cur_m_cls_raw[cls_id])
                {
                    cls_id = i;
                }
            }

            // 1.2.6 不合格则直接跳过, 避免无用的反量化, DFL和dist2bbox计算
            if (cur_m_cls_raw[cls_id] < CONF_THRES_RAW)
            {
                m_cls_raw += class_num;
                m_bbox_raw += reg * 4;
                continue;
            }

            // 1.2.7 计算这个目标的分数
            float score = 1 / (1 + std::exp(-cur_m_cls_raw[cls_id]));

            // 1.2.8 对bbox_raw信息进行反量化, DFL计算
            float ltrb[4], sum, dfl;
            for (int i = 0; i < 4; i++)
            {
                ltrb[i] = 0.;
                sum = 0.;
                for (int j = 0; j < reg; j++)
                {
                    int index_id = reg * i + j;
                    dfl = std::exp(float(cur_m_bbox_raw[index_id]) * m_bbox_scale[index_id]);
                    ltrb[i] += dfl * j;
                    sum += dfl;
                }
                ltrb[i] /= sum;
            }

            // 1.2.9 剔除不合格的框   if(x1 >= x2 || y1 >=y2) continue;
            if (ltrb[2] + ltrb[0] <= 0 || ltrb[3] + ltrb[1] <= 0)
            {
                m_cls_raw += class_num;
                m_bbox_raw += reg * 4;
                continue;
            }

            // 1.2.10 dist 2 bbox (ltrb 2 xyxy)
            float x1 = (w + 0.5 - ltrb[0]) * 16.0;
            float y1 = (h + 0.5 - ltrb[1]) * 16.0;
            float x2 = (w + 0.5 + ltrb[2]) * 16.0;
            float y2 = (h + 0.5 + ltrb[3]) * 16.0;

            // 1.2.11 对应类别加入到对应的std::vector中
            bboxes[cls_id].push_back(cv::Rect2d(x1, y1, x2 - x1, y2 - y1));
            scores[cls_id].push_back(score);

            m_cls_raw += class_num;
            m_bbox_raw += reg * 4;
        }
    }

    // 1.3 大目标特征图
    // output[order[4]]: (1, H // 32,  W // 32,  class_num)
    // output[order[5]]: (1, H // 32,  W // 32,  4 * reg)

    // 1.3.1 检查反量化类型是否符合RDK Model Zoo的README导出的bin模型规范
    if (output[order[4]].properties.quantiType != NONE)
    {
        std::cout << "output[order[4]] QuantiType is not NONE, please check!" << std::endl;
        return -1;
    }
    if (output[order[5]].properties.quantiType != SCALE)
    {
        std::cout << "output[order[5]] QuantiType is not SCALE, please check!" << std::endl;
        return -1;
    }

    // 1.3.2 对缓存的BPU内存进行刷新
    hbSysFlushMem(&(output[order[4]].sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
    hbSysFlushMem(&(output[order[5]].sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);

    // 1.3.3 将BPU推理完的内存地址转换为对应类型的指针
    auto *l_cls_raw = reinterpret_cast<float *>(output[order[4]].sysMem[0].virAddr);
    auto *l_bbox_raw = reinterpret_cast<int32_t *>(output[order[5]].sysMem[0].virAddr);
    auto *l_bbox_scale = reinterpret_cast<float *>(output[order[5]].properties.scale.scaleData);
    for (int h = 0; h < (input_H / 32); h++)
    {
        for (int w = 0; w < (input_W / 32); w++)
        {
            // 1.3.4 取对应H和W位置的C通道, 记为数组的形式
            // cls对应class_num个分数RAW值, 也就是Sigmoid计算之前的值，这里利用函数单调性先筛选, 再计算
            // bbox对应4个坐标乘以reg的RAW值, 也就是DFL计算之前的值, 仅仅分数合格了, 才会进行这部分的计算
            float *cur_l_cls_raw = l_cls_raw;
            int32_t *cur_l_bbox_raw = l_bbox_raw;

            // 1.3.5 找到分数的最大值索引, 如果最大值小于阈值，则舍去
            int cls_id = 0;
            for (int i = 1; i < class_num; i++)
            {
                if (cur_l_cls_raw[i] > cur_l_cls_raw[cls_id])
                {
                    cls_id = i;
                }
            }

            // 1.3.6 不合格则直接跳过, 避免无用的反量化, DFL和dist2bbox计算
            if (cur_l_cls_raw[cls_id] < CONF_THRES_RAW)
            {
                l_cls_raw += class_num;
                l_bbox_raw += reg * 4;
                continue;
            }

            // 1.3.7 计算这个目标的分数
            float score = 1 / (1 + std::exp(-cur_l_cls_raw[cls_id]));

            // 1.3.8 对bbox_raw信息进行反量化, DFL计算
            float ltrb[4], sum, dfl;
            for (int i = 0; i < 4; i++)
            {
                ltrb[i] = 0.;
                sum = 0.;
                for (int j = 0; j < reg; j++)
                {
                    int index_id = reg * i + j;
                    dfl = std::exp(float(cur_l_bbox_raw[index_id]) * l_bbox_scale[index_id]);
                    ltrb[i] += dfl * j;
                    sum += dfl;
                }
                ltrb[i] /= sum;
            }

            // 1.3.9 剔除不合格的框   if(x1 >= x2 || y1 >=y2) continue;
            if (ltrb[2] + ltrb[0] <= 0 || ltrb[3] + ltrb[1] <= 0)
            {
                l_cls_raw += class_num;
                l_bbox_raw += reg * 4;
                continue;
            }

            // 1.3.10 dist 2 bbox (ltrb 2 xyxy)
            float x1 = (w + 0.5 - ltrb[0]) * 32.0;
            float y1 = (h + 0.5 - ltrb[1]) * 32.0;
            float x2 = (w + 0.5 + ltrb[2]) * 32.0;
            float y2 = (h + 0.5 + ltrb[3]) * 32.0;

            // 1.3.11 对应类别加入到对应的std::vector中
            bboxes[cls_id].push_back(cv::Rect2d(x1, y1, x2 - x1, y2 - y1));
            scores[cls_id].push_back(score);

            l_cls_raw += class_num;
            l_bbox_raw += reg * 4;
        }
    }

    // 1.4 对每一个类别进行NMS
    std::vector<std::vector<int>> indices(class_num);
    for (int i = 0; i < class_num; i++)
    {
        cv::dnn::NMSBoxes(bboxes[i], scores[i], score_threshold, nms_threshold, indices[i], 1.f, nms_top_k);
    }

    detected_objects_.clear();
    for (int cls_id = 0; cls_id < class_num; cls_id++) {
        for (std::vector<int>::iterator it = indices[cls_id].begin(); it != indices[cls_id].end(); ++it) {
            // 坐标转换（逆letterbox变换）
            float x1 = (bboxes[cls_id][*it].x - x_shift) / x_scale;
            float y1 = (bboxes[cls_id][*it].y - y_shift) / y_scale;
            float w = bboxes[cls_id][*it].width / x_scale;
            float h = bboxes[cls_id][*it].height / y_scale;
            
            // 边界检查
            x1 = std::max(0.0f, std::min(x1, static_cast<float>(src_w)));
            y1 = std::max(0.0f, std::min(y1, static_cast<float>(src_h)));
            w = std::min(w, static_cast<float>(src_w) - x1);
            h = std::min(h, static_cast<float>(src_h) - y1);
            
            // 存储检测结果
            DetectedObject obj;
            obj.class_name = cls_names_list[cls_id % class_num];
            obj.confidence = scores[cls_id][*it];
            obj.x = static_cast<uint32_t>(x1);
            obj.y = static_cast<uint32_t>(y1);
            obj.width = static_cast<uint32_t>(w);
            obj.height = static_cast<uint32_t>(h);
            
            detected_objects_.push_back(obj);
        }
    }

    // 2. 释放任务
    hbDNNReleaseTask(task_handle);

    // 3. 释放内存
    hbSysFreeMem(&(input.sysMem[0]));
    for (int i = 0; i < 6; i++)
        hbSysFreeMem(&(output[i].sysMem[0]));

    return 0;
}

void RacingObstacleDetection::release_model(){
    hbDNNRelease(packed_dnn_handle);
}

const std::vector<DetectedObject>& RacingObstacleDetection::get_detected_objects() const {
    return detected_objects_;
}
