#include "finger_force.h"

#include <math.h>

#include "finger_sensor.h"

/* ---------- PID 调控抓取 ---------- */

void finger_pid_reset(pid_t *pid)
{
    pid->err        = 0.0f;
    pid->err_prev   = 0.0f;
    pid->integral      = 0.0f;
    pid->integral_step = 0.0f;
    pid->derivative    = 0.0f;
    pid->output        = 0.0f;
    pid->feedback      = 0.0f;
}

static void finger_pid_configure(pid_t *pid)
{
    pid->kp                = FINGER_PID_KP;
    pid->ki                = FINGER_PID_KI;
    pid->kd                = FINGER_PID_KD;
    pid->MaxOutput         = FINGER_PID_MAX_OUTPUT;
    pid->MaxIntegral       = FINGER_PID_MAX_INTEGRAL;
    pid->integral_start_err = FINGER_PID_INTEGRAL_START;
    pid->dt_nominal        = FINGER_UPDATE_DT;
}

static pid_t s_fingersPID[FINGER_COUNT];

void finger_pid_init_all(void)
{
    for (int i = 0; i < FINGER_COUNT; i++) {
        vPID_Init(&s_fingersPID[i]);
        finger_pid_configure(&s_fingersPID[i]);
    }
}

pid_t *finger_pid_get(int idx)
{
    if (idx < 0 || idx >= FINGER_COUNT) {
        return NULL;
    }
    return &s_fingersPID[idx];
}

/* ---------- 压力读取与接触判定 ---------- */

/* 手指下标 0/1 与压力传感器 SENSOR_1/SENSOR_2 一一对应 */
bool finger_has_sensor(int idx)
{
    return idx >= 0 && idx < SENSOR_COUNT;
}

float finger_read_force(int idx)
{
    //如果对应的手指没有压力传感器，则返回0.0f
    if(idx < 0 || idx >= SENSOR_COUNT){
        return 0.0f;
    }
    app_adc_data_t data;
    if (!app_adc_get_latest((sensor_id_t)idx, &data)) {
        return 0.0f;
    }
    return raw_to_force(data.raw);
}

/* ---------- 压力换算（标定参数见 finger_config.h） ---------- */

/* 备用标定路径：由电压换算力，当前主流程使用 raw_to_force */
float voltage_to_force_v(float v)
{
    if (v < 0.01f) v = 0.01f;
    if (v > 3.29f) v = 3.29f;

    float invR = (3.3f - v) / (10.0f * v);
    float f = (invR - FINGER_FORCE_V_OFFSET) / FINGER_FORCE_V_SLOPE;

    if (f < 0.0f)             f = 0.0f;
    if (f > FINGER_FORCE_MAX) f = FINGER_FORCE_MAX;
    return f;
}

/* 注意：三段的阈值与系数是当前粗略标定，分段点附近不保证连续；
 * 正式使用前应按实际传感器重新标定 FINGER_FORCE_SEGx_* 参数。 */
float raw_to_force(int raw)
{
    if (raw <= 0) {
        return 0.0f;    /* 无采样/传感器开路：避免除零产生 inf */
    }

    float x = (FINGER_FORCE_RAW_FULL / (float)raw) - 1.0f;

    float f;
    if (raw > FINGER_FORCE_SEG1_RAW) {
        f = FINGER_FORCE_SEG1_SLOPE * x;
    } else if (raw > FINGER_FORCE_SEG2_RAW) {
        f = FINGER_FORCE_SEG2_SLOPE * powf(x, FINGER_FORCE_SEG2_EXP);
    } else {
        f = FINGER_FORCE_SEG3_SLOPE * powf(x, FINGER_FORCE_SEG3_EXP);
    }

    if (f < 0.0f)             f = 0.0f;
    if (f > FINGER_FORCE_MAX) f = FINGER_FORCE_MAX;
    return f;
}
