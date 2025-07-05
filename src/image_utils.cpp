#include "racing_obstacle_detection/image_utils.h"

void letterbox_nv12(const uint8_t* nv12, int src_w, int src_h,
                    uint8_t* out_nv12, int dst_w, int dst_h,
                    int& x_shift, int& y_shift, float& x_scale, float& y_scale) {
    // 1. 拆分Y/UV
    const uint8_t* y_in = nv12;
    const uint8_t* uv_in = nv12 + src_w * src_h;
 
    // 2. 构建Y和UV的Mat，分别处理
    cv::Mat y_plane_in(src_h, src_w, CV_8UC1, (void*)y_in);
    cv::Mat uv_plane_in(src_h/2, src_w/2, CV_8UC2, (void*)uv_in);
 
    // 3. 计算缩放 & padding
    x_scale = std::min(1.0 * dst_h / src_h, 1.0 * dst_w / src_w);
    y_scale = x_scale;
 
    int new_w = int(src_w * x_scale + 0.5);
    int new_h = int(src_h * y_scale + 0.5);
 
    x_shift = (dst_w - new_w) / 2;
    int x_other = dst_w - new_w - x_shift;
    y_shift = (dst_h - new_h) / 2;
    int y_other = dst_h - new_h - y_shift;
 
    // 4. 分别resize、letterbox Y和UV平面
 
    // Y
    cv::Mat y_plane_resize, y_plane_out;
    cv::resize(y_plane_in, y_plane_resize, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);
    cv::copyMakeBorder(y_plane_resize, y_plane_out, y_shift, y_other, x_shift, x_other, cv::BORDER_CONSTANT, cv::Scalar(127));
 
    // UV（UV分辨率减半）
    cv::Mat uv_resize, uv_out;
    int uv_new_w = new_w / 2;
    int uv_new_h = new_h / 2;
    int uv_x_shift = x_shift / 2;
    int uv_x_other = (dst_w/2) - uv_new_w - uv_x_shift;
    int uv_y_shift = y_shift / 2;
    int uv_y_other = (dst_h/2) - uv_new_h - uv_y_shift;
 
    cv::resize(uv_plane_in, uv_resize, cv::Size(uv_new_w, uv_new_h), 0, 0, cv::INTER_LINEAR);
    cv::copyMakeBorder(uv_resize, uv_out, uv_y_shift, uv_y_other, uv_x_shift, uv_x_other, cv::BORDER_CONSTANT, cv::Scalar(127, 127));
 
    // 5. 合并为 NV12
    // Y
    memcpy(out_nv12, y_plane_out.data, dst_w * dst_h);
    // UV
    memcpy(out_nv12 + dst_w * dst_h, uv_out.data, (dst_w/2) * (dst_h/2) * 2);
}
