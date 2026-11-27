#include "PID.h"
#include <string.h> 
#include <math.h>

void vPID_Init(pid_t *pid){
        if (pid == NULL) {
        return;
    }
    memset(pid, 0, sizeof(*pid));
}

void fPID(pid_t *pid, float dt)
{
    if (pid == NULL) return;

    /* 0. 时间归一：scale = 实际 dt / 标称 dt。标称周期下 scale=1，行为与旧实现一致 */
    if (dt <= 0.0f) dt = pid->dt_nominal;
    float scale = 1.0f;
    if (pid->dt_nominal > 0.0f && dt > 0.0f) {
        scale = dt / pid->dt_nominal;
        if (scale <= 0.0f) scale = 1.0f;
    }

    /* 1. 误差 */
    pid->err = pid->target - pid->feedback;

    /* 2. 微分：先算，避免用上一拍的旧值（换算到“每标称周期”的误差变化率） */
    pid->derivative = (pid->err - pid->err_prev) / scale;

    /* 3. 用当前积分算未限幅输出，判断饱和 */
    float out_raw = pid->kp * pid->err
                  + pid->ki * pid->integral
                  + pid->kd * pid->derivative;

    bool saturated = (out_raw > pid->MaxOutput) || (out_raw < -pid->MaxOutput);

    /* 4. 积分分离 + 抗 windup：未饱和且误差小才积分 */
    pid->integral_step = 0.0f;
    if (!saturated && fabsf(pid->err) < pid->integral_start_err) {
        pid->integral_step = pid->err * scale;
        pid->integral += pid->integral_step;
        if (pid->integral > pid->MaxIntegral)  pid->integral = pid->MaxIntegral;
        if (pid->integral < -pid->MaxIntegral) pid->integral = -pid->MaxIntegral;

        /* 积分变了，重算输出 */
        out_raw = pid->kp * pid->err
                + pid->ki * pid->integral
                + pid->kd * pid->derivative;
    }

    /* 5. 输出限幅（按标称周期），再换算成“本拍实际角度增量” */
    pid->output = out_raw;
    if (pid->output > pid->MaxOutput)  pid->output = pid->MaxOutput;
    if (pid->output < -pid->MaxOutput) pid->output = -pid->MaxOutput;
    pid->output *= scale;

    /* 6. 保存误差 */
    pid->err_prev = pid->err;
}