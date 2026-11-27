#ifndef PID_H
#define PID_H
 
#include <stdbool.h>

typedef struct {
    /* ---- 参数 ---- */
    float kp;            /* 比例系数 */
    float ki;            /* 积分系数 */
    float kd;            /* 微分系数 */
    float offset;        /* 输出偏置 */
    
    float MaxIntegral;    /* 积分限幅 */

    /* ---- 目标与反馈 ---- */
    float target;        /* 目标值 */
    float feedback;      /* 当前反馈值（原 CurValue） */
    float feedback_prev; /* 上次反馈值（原 PreValue） */

    /* ---- 误差与中间量 ---- */
    float err;           /* 本次误差 */
    float err_prev;      /* 上次误差 */
    float integral;      /* 积分累加（按标称周期归一） */
    float integral_step; /* 本拍积分增量，供外层抗饱和回滚 */
    float derivative;    /* 微分项（按标称周期归一） */

    /* ---- 输出 ---- */
    float output;        /* 本拍实际输出增量（已按 dt 缩放） */

    float MaxOutput;     /* 输出限幅（每个标称周期的增量） */
    float integral_start_err;

    /* ---- 时间基准 ---- */
    float dt_nominal;    /* 标称控制周期（秒），用于 dt 归一化 */
} pid_t;


void vPID_Init(pid_t *pid);

/**
 * @brief 计算一拍 PID。
 * @param dt 本拍实际时间间隔（秒）。内部按 dt/dt_nominal 做时间归一，
 *           标称周期下与不带 dt 的旧实现完全一致；抖动/延迟时按真实时间补偿。
 *           dt <= 0 时退化为 dt_nominal。
 */
void fPID(pid_t *pid, float dt);

#endif

