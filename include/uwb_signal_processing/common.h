#ifndef _UWB_COMMON_H_
#define _UWB_COMMON_H_

#include <iostream>
#include <cmath>

namespace uwb_signal_processing {

// 定义PI常量
const float PI = 3.14159265358979323846;

// 弧度转换为度数
float radiansToDegrees(float radians) {
    return radians * (180.0 / PI);
}

// 度数转换为弧度
float degreesToRadians(float degrees) {
    return degrees * (PI / 180.0);
}

int sgn(float num) {
    return (num > 0.0f) ? 1 : ((num < 0.0f) ? -1 : 0);
}


} // namespace uwb_signal_processing 

#endif  // _UWB_COMMON_H_