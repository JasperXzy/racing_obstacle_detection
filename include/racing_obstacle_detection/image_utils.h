#ifndef IMAGE_UTILS_H
#define IMAGE_UTILS_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>
#include <cstdint>

void letterbox_nv12(const uint8_t* nv12, int src_w, int src_h,
                    uint8_t* out_nv12, int dst_w, int dst_h,
                    int& x_shift, int& y_shift, float& x_scale, float& y_scale);

#endif // IMAGE_UTILS_H
