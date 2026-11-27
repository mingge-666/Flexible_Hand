/**
 * @file    AverageFilter.h
 * @brief   平均值类滤波算法（去极值平均）。
 */

#ifndef AVERAGE_FILTER_H
#define AVERAGE_FILTER_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 冒泡排序后去掉一个最大值和一个最小值，再求平均。
 *
 * @param samples 输入数组；函数会就地升序排序，调用方不要依赖原顺序。
 * @param len     元素个数。len==0 返回 0.0f；len<3 时退化为普通平均。
 * @return 滤波结果（单位与输入一致）。
 */
float average_filter_trimmed_mean(float *samples, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* AVERAGE_FILTER_H */
