#ifndef APP_FINGER_H
#define APP_FINGER_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>
#include "finger_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 手指编号：枚举值即手指编号（FINGER_1=1, FINGER_2=2），数组访问时用 finger-1 */
typedef enum {
    FINGER_1 = 1,   /* 手指1：PCA9685 通道0 舵机 + GPIO5 压力 */
    FINGER_2 = 2,   /* 手指2：PCA9685 通道1 舵机 + GPIO7 压力 */
    FINGER_3 = 3,   /* 手指3：PCA9685 通道2 舵机 + 无压力 */
    FINGER_COUNT = 3,
} finger_id_t;

typedef enum {
    FINGER_STATE_IDLE = 0,          /* 空闲 */
    FINGER_APPROACHING,             /* 接近：向目标角度运动，接触即停 */
    FINGER_APPROACHED,              /* 已接近到位（到位或接触物体），等待 catch */
    FINGER_CATCHING,                /* 抓取：PID 力控闭环 */
    FINGER_CATCHED,                 /* 已抓取到位（力稳定在目标值附近） */
    FINGER_MOVING,                  /* 直接定位：平滑运动到目标角度，无接触检测 */
} finger_state_t;

/* 加速度/速度预设，具体数值见 finger_config.h */
typedef enum {
    FINGER_ACCEL_MODE_0 = 0,   /* 危险/最慢：150°/s², 50°/s */
    FINGER_ACCEL_MODE_1,       /* 慢速：250°/s², 80°/s */
    FINGER_ACCEL_MODE_2,       /* 默认：350°/s², 140°/s */
    FINGER_ACCEL_MODE_3,       /* 中速：500°/s², 200°/s */
    FINGER_ACCEL_MODE_4,       /* 快速：900°/s², 280°/s */
} finger_accel_mode_t;

/* 单根手指状态：命令任务写 target/state，定时器写 cur/speed，打印任务只读 */
typedef struct {
    uint8_t channel;          /* PCA9685 PWM 通道号（0-15） */
    finger_state_t  state;    /* 手指当前的状态 */
    float cur_deg;            /* 当前角度（0-180°） */
    float target_deg;         /* 目标角度（0-180°） */
    float min_deg;            /* 机械允许最小角 */
    float max_deg;            /* 机械允许最大角 */
    float cur_speed;          /* 当前速度 度/秒，初值 0 */
    float last_sent_deg;      /* 最近一次成功写入舵机的角度（写去抖用） */
    float max_speed;          /* 最大速度 度/秒 */
    float min_speed;          /* 最小速度 度/秒，防太慢 */
    float accel;              /* 加速度 度/秒² */

    float cur_force;          /* 当前压力（由 ADC 换算） */
    float target_force;       /* 目标压力 */

    float     contact_threshold; /* 接触判定阈值：压力超过它即认为接触物体 */
    int8_t    grasp_dir;         /* 抓取方向：+1 增大角度、-1 减小角度 */
    bool      pid_reset;         /* 命令任务置位，定时器消费一次后清零 */
    bool      contacted;         /* true=接近时真正压到物体；false=只是走到目标角 */
    uint32_t  catch_stable;      /* CATCHED 判定连续稳定计数 */
    uint32_t  catch_ticks;       /* CATCHING 持续节拍数，用于超时退出 */
    uint32_t  catched_lost;      /* CATCHED 下力偏离目标的连续节拍数 */
} finger_t;


/**
 * @brief 设置手指的加速度模式。
 * @param finger 手指编号（1..FINGER_COUNT）
 * @param mode   0=危险/最慢, 1=慢速, 2=默认, 3=中速, 4=快速（数值见 finger_config.h）
 */
void app_finger_set_accmode(finger_id_t finger, finger_accel_mode_t mode);


/**
 * @brief 初始化手指舵机与 I2C/UART，启动串口命令任务和 5ms 控制任务。
 */
esp_err_t app_finger_start(void);

/**
 * @brief 两根手指同时接近到指定“闭合量”；过程中接触物体即停止并进入 APPROACHED。
 * @param angle_deg 闭合量：0=全开，180=全闭。镜像安装的手指会自动换算目标角
 *                  （见 FINGER_POSITION_MIRROR）。
 */
esp_err_t app_finger_approach(float angle_deg);

/**
 * @brief 两根手指同时进入 PID 力控抓取（仅在 APPROACHED/CATCHING/CATCHED 下有效）。
 */
esp_err_t app_finger_catch(float force);

/**
 * @brief 立即停止运动并保持当前位置（回到 IDLE）。
 */
esp_err_t app_finger_stop(void);

/**
 * @brief 两根手指直接平滑运动到指定“闭合量”（不做接触检测、不受力控约束），
 *        用于抓取后放开手指等直接定位场景。
 * @param angle_deg 闭合量：0=全开，180=全闭。镜像安装的手指会自动换算目标角。
 */
esp_err_t app_finger_move(float angle_deg);



//每次控制1根手指
esp_err_t app_onefinger_approach(float angle_deg, finger_id_t finger);

esp_err_t app_onefinger_catch(float force, finger_id_t finger);

esp_err_t app_onefinger_move(float angle_deg, finger_id_t finger);

/**
 * @brief 获取某根手指的状态快照（持锁拷贝，供打印任务读取）。
 * @param out 输出副本，调用方持有，不与控制任务竞争。
 * @return true 成功；false 参数或手指编号非法。
 */
bool app_finger_get_snapshot(finger_id_t finger, finger_t *out);

#ifdef __cplusplus
}
#endif

#endif /* APP_FINGER_H */
