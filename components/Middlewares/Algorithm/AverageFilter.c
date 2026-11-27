/**
 * @file    AverageFilter.c
 * @brief   平均值类滤波：冒泡排序 + 去最大最小值 + 求平均。
 */

#include "AverageFilter.h"

#include <stdbool.h>

/**
 * @brief 冒泡排序后去掉一个最大值和一个最小值，再求平均。
 */
float average_filter_trimmed_mean(float *samples, size_t len)
{
    if (samples == NULL || len == 0) {
        return 0.0f;
    }

    /* 少于 3 个数无法去极值，退化为普通平均 */
    if (len < 3) {
        float sum = 0.0f;
        for (size_t i = 0; i < len; ++i) {
            sum += samples[i];
        }
        return sum / (float)len;
    }

    /* 冒泡排序（升序），10 个元素开销可忽略 */
    for (size_t i = 0; i + 1 < len; ++i) {
        bool swapped = false;
        for (size_t j = 0; j + 1 < len - i; ++j) {
            if (samples[j] > samples[j + 1]) {
                float tmp = samples[j];
                samples[j] = samples[j + 1];
                samples[j + 1] = tmp;
                swapped = true;
            }
        }
        if (!swapped) {
            break;          /* 已有序，提前结束 */
        }
    }

    /* 去掉 samples[0]（最小）与 samples[len-1]（最大） */
    double sum = 0.0;
    for (size_t i = 1; i + 1 < len; ++i) {
        sum += samples[i];
    }
    return (float)(sum / (double)(len - 2));
}

