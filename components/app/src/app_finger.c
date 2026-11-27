/**
 * @file    app_finger.c
 * @brief   两个手指（PCA9685 + I2C 舵机）的串口控制与接近/抓取状态机。
 *
 * 命令协议（单行文本）：
 *   approach <角度>   : 两根手指同时接近到指定角度；过程中接触物体立即停止并进入 APPROACHED
 *   catch    <力度>   : 两根手指进入 PID 力控抓取（需先处于 APPROACHED）
 *   stop              : 停止运动并保持当前位置
 *
 * 状态机（每根手指独立）：
 *   IDLE -> APPROACHING -> APPROACHED -> CATCHING -> CATCHED
 *   APPROACHING 中检测到接触（压力 > contact_threshold）会立即冻结角度并跳到 APPROACHED，
 *   不继续运动，直到收到 catch 命令才通过 PID 闭环控制舵机施加目标压力。
 *
 * 独立控制任务（5ms 周期）调用 finger_update 推进每根手指的状态机，
 * 命令任务/控制任务/打印快照共用一把互斥锁。
 */

#include "app_finger.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "PID.h"
#include "Servo.h"
#include "finger_sensor.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "uart.h"
#include "finger_config.h"
#include "finger_state.h"
#include "finger_force.h"
#include "finger_cmd.h"


static const char *TAG = "APP_FINGER";
static bool s_started = false;

/* 保护 s_fingers：命令任务、控制任务、打印快照共用 */
static SemaphoreHandle_t s_finger_mutex = NULL;

static void finger_lock(void)//加上互斥锁，保证任务期间用到的 s_fingers 不会被其他任务修改
{
    if (s_finger_mutex != NULL) {
        xSemaphoreTake(s_finger_mutex, portMAX_DELAY);
    }
}

static void finger_unlock(void)//解锁，让其他任务可以访问 s_fingers
{
    if (s_finger_mutex != NULL) {
        xSemaphoreGive(s_finger_mutex);
    }
}

static finger_t s_fingers[FINGER_COUNT] = {
    {   /* FINGER_1（数组下标 0） */
        .channel           = FINGER1_CHANNEL,
        .state             = FINGER_STATE_IDLE,
        .cur_deg           = 90.0f,
        .target_deg        = 90.0f,
        .min_deg           = FINGER1_MIN_DEG,
        .max_deg           = FINGER1_MAX_DEG,
        .cur_speed         = 0.0f,
        .last_sent_deg     = 90.0f,
        .max_speed         = FINGER_DEFAULT_MAX_SPEED,
        .min_speed         = FINGER_DEFAULT_MIN_SPEED,
        .accel             = FINGER_DEFAULT_ACCEL_DPS2,
        .cur_force         = 0.0f,
        .target_force      = 0.0f,
        .contact_threshold = FINGER_CONTACT_FORCE_THRESHOLD,
        .grasp_dir         = FINGER1_GRASP_DIR,
        .pid_reset         = false,
        .contacted         = false,
        .catch_stable      = 0,
        .catch_ticks       = 0,
        .catched_lost      = 0,
    },
    {   /* FINGER_2（数组下标 1） */
        .channel           = FINGER2_CHANNEL,
        .state             = FINGER_STATE_IDLE,
        .cur_deg           = 90.0f,
        .target_deg        = 90.0f,
        .min_deg           = FINGER2_MIN_DEG,
        .max_deg           = FINGER2_MAX_DEG,
        .cur_speed         = 0.0f,
        .last_sent_deg     = 90.0f,
        .max_speed         = FINGER_DEFAULT_MAX_SPEED,
        .min_speed         = FINGER_DEFAULT_MIN_SPEED,
        .accel             = FINGER_DEFAULT_ACCEL_DPS2,
        .cur_force         = 0.0f,
        .target_force      = 0.0f,
        .contact_threshold = FINGER_CONTACT_FORCE_THRESHOLD,
        .grasp_dir         = FINGER2_GRASP_DIR,
        .pid_reset         = false,
        .contacted         = false,
        .catch_stable      = 0,
        .catch_ticks       = 0,
        .catched_lost      = 0,
    },
    {   /* FINGER_3（数组下标 2） */
        .channel           = FINGER3_CHANNEL,
        .state             = FINGER_STATE_IDLE,
        .cur_deg           = 90.0f,
        .target_deg        = 90.0f,
        .min_deg           = FINGER3_MIN_DEG,
        .max_deg           = FINGER3_MAX_DEG,
        .cur_speed         = 0.0f,
        .last_sent_deg     = 90.0f,
        .max_speed         = FINGER_DEFAULT_MAX_SPEED,
        .min_speed         = FINGER_DEFAULT_MIN_SPEED,
        .accel             = FINGER_DEFAULT_ACCEL_DPS2,
        .cur_force         = 0.0f,
        .target_force      = 0.0f,
        .contact_threshold = FINGER_CONTACT_FORCE_THRESHOLD,
        .grasp_dir         = FINGER3_GRASP_DIR,
        .pid_reset         = false,
        .contacted         = false,
        .catch_stable      = 0,
        .catch_ticks       = 0,
        .catched_lost      = 0,
    },
};


/* ---------- 控制任务 ---------- */

/**
 * @brief 控制任务：每 5ms 推进一次状态机。
 *        用实测 dt（esp_timer）驱动规划与 PID，抵消调度抖动；
 *        不再在 FreeRTOS 定时器回调里做阻塞 I2C。
 */
static void finger_ctrl_task(void *arg)
{
    (void)arg;

    int64_t last = esp_timer_get_time();
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(FINGER_UPDATE_PERIOD_MS));

        int64_t now = esp_timer_get_time();
        float dt = (float)(now - last) / 1000000.0f;
        last = now;

        /* 限幅：过小视为异常，过大则按 20ms 上限处理，避免单拍跳变 */
        if (dt < 0.001f) {
            dt = FINGER_UPDATE_DT;
        } else if (dt > 0.020f) {
            dt = 0.020f;
        }

        finger_lock();
        for (int i = 0; i < FINGER_COUNT; ++i) {
            finger_update(&s_fingers[i], i, dt);
        }
        finger_unlock();
    }
}


/* ---------- 公开 API ---------- */

/* 位置指令统一按“闭合量”解释：0=全开，180=全闭。
 * 镜像安装（grasp_dir<0）的手指目标角取反，保证两根手指同向动作。 */
static float finger_target_from_openness(const finger_t *f, float openness)
{
#if FINGER_POSITION_MIRROR
    float target = (f->grasp_dir > 0) ? openness
                                      : (SERVO_MAX_ANGLE_DEG - openness);
#else
    float target = openness;
#endif

    /* 按该手指的机械行程再钳一次 */
    if (target < f->min_deg) target = f->min_deg;
    if (target > f->max_deg) target = f->max_deg;
    return target;
}

//同时控制多根手指

esp_err_t app_finger_approach(float angle_deg)
{
    if (angle_deg < 0.0f)                    angle_deg = 0.0f;
    if (angle_deg > SERVO_MAX_ANGLE_DEG)     angle_deg = SERVO_MAX_ANGLE_DEG;

    finger_lock();
    for (int i = 0; i < FINGER_COUNT; ++i) {
        s_fingers[i].target_deg   = finger_target_from_openness(&s_fingers[i], angle_deg);
        s_fingers[i].target_force = 0.0f;
        s_fingers[i].cur_speed    = 0.0f;
        s_fingers[i].catch_stable = 0;
        s_fingers[i].catch_ticks  = 0;
        s_fingers[i].pid_reset    = false;
        s_fingers[i].contacted    = false;
        s_fingers[i].state        = FINGER_APPROACHING;
    }
    finger_unlock();

    printf("approach -> %.1f deg (openness)\n", (double)angle_deg);
    fflush(stdout);
    return ESP_OK;
}

esp_err_t app_finger_catch(float force)
{
    if (force < 0.0f) force = 0.0f;
    if (force > FINGER_MAX_FORCE) force = FINGER_MAX_FORCE;

    finger_lock();

    /* 只校验/控制带压力传感器的手指；无传感器手指不参与力控 */
    for (int i = 0; i < FINGER_COUNT; ++i) {
        if (!finger_has_sensor(i)) {
            continue;
        }
        if (s_fingers[i].state != FINGER_APPROACHED &&
            s_fingers[i].state != FINGER_CATCHING &&
            s_fingers[i].state != FINGER_CATCHED) {
            int state = (int)s_fingers[i].state;
            finger_unlock();
            printf("finger %d not approached yet (state=%d), ignore catch\n",
                   i + 1, state);
            fflush(stdout);
            return ESP_ERR_INVALID_STATE;
        }
#if FINGER_CATCH_REQUIRE_CONTACT
        if (!s_fingers[i].contacted) {
            finger_unlock();
            printf("finger %d not contacted, ignore catch (avoid pressing into nothing)\n",
                   i + 1);
            fflush(stdout);
            return ESP_ERR_INVALID_STATE;
        }
#endif
    }

    for (int i = 0; i < FINGER_COUNT; ++i) {
        if (!finger_has_sensor(i)) {
            continue;
        }
        s_fingers[i].target_force = force;
        s_fingers[i].catch_stable = 0;
        s_fingers[i].catch_ticks  = 0;
        s_fingers[i].catched_lost = 0;
        s_fingers[i].pid_reset    = true;
        s_fingers[i].state        = FINGER_CATCHING;
    }

    finger_unlock();

    printf("catch -> force %.1f\n", (double)force);
    fflush(stdout);
    return ESP_OK;
}

esp_err_t app_finger_stop(void)
{
    finger_lock();
    for (int i = 0; i < FINGER_COUNT; ++i) {
        s_fingers[i].state       = FINGER_STATE_IDLE;
        s_fingers[i].target_deg  = s_fingers[i].cur_deg;
        s_fingers[i].cur_speed   = 0.0f;
        s_fingers[i].pid_reset   = false;
        s_fingers[i].catch_ticks = 0;
    }
    finger_unlock();

    printf("stop, hold position\n");
    fflush(stdout);
    return ESP_OK;
}

esp_err_t app_finger_move(float angle_deg)
{
    if (angle_deg < 0.0f)                angle_deg = 0.0f;
    if (angle_deg > SERVO_MAX_ANGLE_DEG) angle_deg = SERVO_MAX_ANGLE_DEG;

    finger_lock();
    for (int i = 0; i < FINGER_COUNT; ++i) {
        s_fingers[i].target_deg   = finger_target_from_openness(&s_fingers[i], angle_deg);
        s_fingers[i].target_force = 0.0f;
        s_fingers[i].cur_speed    = 0.0f;
        s_fingers[i].catch_stable = 0;
        s_fingers[i].catch_ticks  = 0;
        s_fingers[i].pid_reset    = false;
        s_fingers[i].contacted    = false;
        s_fingers[i].state        = FINGER_MOVING;
    }
    finger_unlock();

    printf("move -> %.1f deg (openness)\n", (double)angle_deg);
    fflush(stdout);
    return ESP_OK;
}

//每次控制1根手指
esp_err_t app_onefinger_approach(float angle_deg, finger_id_t finger)
{
    if (finger < FINGER_1 || finger > FINGER_COUNT) {
        printf("Invalid finger id: %d\n", (int)finger);
        return ESP_ERR_INVALID_ARG;
    }
    int idx = (int)finger - 1;
    if (angle_deg < 0.0f)                    angle_deg = 0.0f;
    if (angle_deg > SERVO_MAX_ANGLE_DEG)     angle_deg = SERVO_MAX_ANGLE_DEG;

    finger_lock();
    s_fingers[idx].target_deg   = finger_target_from_openness(&s_fingers[idx], angle_deg);
    s_fingers[idx].target_force = 0.0f;
    s_fingers[idx].cur_speed    = 0.0f;
    s_fingers[idx].catch_stable = 0;
    s_fingers[idx].catch_ticks  = 0;
    s_fingers[idx].pid_reset    = false;
    s_fingers[idx].contacted    = false;
    s_fingers[idx].state        = FINGER_APPROACHING;
    finger_unlock();

    printf("finger %d approach -> %.1f deg\n", (int)finger, (double)angle_deg);
    fflush(stdout);
    return ESP_OK;
}

esp_err_t app_onefinger_catch(float force, finger_id_t finger)
{
    if (finger < FINGER_1 || finger > FINGER_COUNT) {
        printf("Invalid finger id: %d\n", (int)finger);
        return ESP_ERR_INVALID_ARG;
    }
    int idx = (int)finger - 1;

    if (force < 0.0f) force = 0.0f;
    if (force > FINGER_MAX_FORCE) force = FINGER_MAX_FORCE;

    if (!finger_has_sensor(idx)) {
        printf("finger %d has no force sensor, catch unsupported\n", (int)finger);
        fflush(stdout);
        return ESP_ERR_NOT_SUPPORTED;
    }

    finger_lock();

    /* 需先完成接近（或已在抓取中/已抓取） */
    if (s_fingers[idx].state != FINGER_APPROACHED &&
        s_fingers[idx].state != FINGER_CATCHING &&
        s_fingers[idx].state != FINGER_CATCHED) {
        int state = (int)s_fingers[idx].state;
        finger_unlock();
        printf("finger %d not approached yet (state=%d), ignore catch\n",
                (int)finger, state);
        fflush(stdout);
        return ESP_ERR_INVALID_STATE;
    }
#if FINGER_CATCH_REQUIRE_CONTACT
    if (!s_fingers[idx].contacted) {
        finger_unlock();
        printf("finger %d not contacted, ignore catch (avoid pressing into nothing)\n",
                (int)finger);
        fflush(stdout);
        return ESP_ERR_INVALID_STATE;
    }
#endif

    s_fingers[idx].target_force = force;
    s_fingers[idx].catch_stable = 0;
    s_fingers[idx].catch_ticks  = 0;
    s_fingers[idx].catched_lost = 0;
    s_fingers[idx].pid_reset    = true;
    s_fingers[idx].state        = FINGER_CATCHING;
    finger_unlock();

    printf("finger %d catch -> force %.1f\n", (int)finger, (double)force);
    fflush(stdout);
    return ESP_OK;
}


esp_err_t app_onefinger_move(float angle_deg, finger_id_t finger)
{
    if (finger < FINGER_1 || finger > FINGER_COUNT) {
        printf("Invalid finger id: %d\n", (int)finger);
        return ESP_ERR_INVALID_ARG;
    }
    int idx = (int)finger - 1;

    if (angle_deg < 0.0f)                angle_deg = 0.0f;
    if (angle_deg > SERVO_MAX_ANGLE_DEG) angle_deg = SERVO_MAX_ANGLE_DEG;

    finger_lock();
    s_fingers[idx].target_deg   = finger_target_from_openness(&s_fingers[idx], angle_deg);
    s_fingers[idx].target_force = 0.0f;
    s_fingers[idx].cur_speed    = 0.0f;
    s_fingers[idx].catch_stable = 0;
    s_fingers[idx].catch_ticks  = 0;
    s_fingers[idx].pid_reset    = false;
    s_fingers[idx].contacted    = false;
    s_fingers[idx].state        = FINGER_MOVING;
    finger_unlock();

    printf("finger %d move -> %.1f deg\n", (int)finger, (double)angle_deg);
    fflush(stdout);
    return ESP_OK;
}

void app_finger_set_accmode(finger_id_t finger, finger_accel_mode_t mode){

    if (finger < FINGER_1 || finger > FINGER_COUNT) {
        printf("Invalid finger id: %d\n", (int)finger);
        return;
    }
    int idx = (int)finger - 1;

    finger_lock();
    switch(mode){
        case FINGER_ACCEL_MODE_0:
            s_fingers[idx].accel = FINGER_DANGER_ACCEL_DPS2;
            s_fingers[idx].max_speed = FINGER_DANGER_MAX_SPEED;
            break;
        case FINGER_ACCEL_MODE_1:
            s_fingers[idx].accel = FINGER_SLOW_ACCEL_DPS2;
            s_fingers[idx].max_speed = FINGER_SLOW_MAX_SPEED;
            break;
        case FINGER_ACCEL_MODE_2:
            s_fingers[idx].accel = FINGER_DEFAULT_ACCEL_DPS2;
            s_fingers[idx].max_speed = FINGER_DEFAULT_MAX_SPEED;
            break;
        case FINGER_ACCEL_MODE_3:
            s_fingers[idx].accel = FINGER_MIDDLE_ACCEL_DPS2;
            s_fingers[idx].max_speed = FINGER_MIDDLE_MAX_SPEED;
            break;
        case FINGER_ACCEL_MODE_4:
            s_fingers[idx].accel = FINGER_FAST_ACCEL_DPS2;
            s_fingers[idx].max_speed = FINGER_FAST_MAX_SPEED;
            break;
        default:
            finger_unlock();
            printf("Invalid finger acceleration mode: %d\n", (int)mode);
            return;
    }
    finger_unlock();
}


bool app_finger_get_snapshot(finger_id_t finger, finger_t *out)
{
    if (out == NULL) {
        return false;
    }
    int idx = (int)finger - 1;
    if (idx < 0 || idx >= FINGER_COUNT) {
        return false;
    }

    finger_lock();
    *out = s_fingers[idx];
    finger_unlock();
    return true;
}

esp_err_t app_finger_start(void)//舵机任务创建
{
    if (s_started) {
        return ESP_OK;
    }

    if (s_finger_mutex == NULL) {
        s_finger_mutex = xSemaphoreCreateMutex();//创建互斥锁
        if (s_finger_mutex == NULL) {
            ESP_LOGE(TAG, "xSemaphoreCreateMutex failed");
            return ESP_ERR_NO_MEM;
        }
    }

    /* 安装 UART0 驱动（115200，GPIO43 TX / GPIO44 RX），用于接收命令 */
    esp_err_t ret = uart_cmd_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_cmd_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 首次调用会初始化 I2C(SCL=GPIO8, SDA=GPIO4) 与 PCA9685，之后按通道居中 */
    for (int i = 0; i < FINGER_COUNT; ++i) {
        ret = servo_init(s_fingers[i].channel);   /* 初始化并居中 90° */
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "servo_init(finger%d) failed: %s", i + 1, esp_err_to_name(ret));
            return ret;
        }
    }

    finger_pid_init_all();

    TaskHandle_t cmd_handle = NULL;
    BaseType_t ok = xTaskCreatePinnedToCore(finger_cmd_task, "finger_cmd",
                                            FINGER_CMD_TASK_STACK, NULL,
                                            FINGER_CMD_TASK_PRIO, &cmd_handle, 0);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "create finger_cmd task failed");
        return ESP_ERR_NO_MEM;
    }

    /* 控制任务：替代软件定时器，5ms 周期推进状态机（允许阻塞 I2C） */
    ok = xTaskCreatePinnedToCore(finger_ctrl_task, "finger_ctrl",
                                 FINGER_CTRL_TASK_STACK, NULL,
                                 FINGER_CTRL_TASK_PRIO, NULL, 0);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "create finger_ctrl task failed");
        if (cmd_handle != NULL) {
            vTaskDelete(cmd_handle);   /* 回滚：没有控制任务时命令任务无意义 */
        }
        return ESP_ERR_NO_MEM;
    }

    s_started = true;
    return ESP_OK;
}
