#include "finger_motion.h"

#include <math.h>

#include "PID.h"
#include "Servo.h"
#include "esp_log.h"
#include "finger_force.h"

static const char *TAG = "FINGER_MOTION";

/**
 * @brief 写入当前角度；I2C 失败时限频打印，避免故障被静默忽略。
 */
static void apply_servo(finger_t *f)
{
    static uint32_t err_count = 0;

    /* 角度变化小于死区不重复写 I2C（小于 PCA9685 一个计数） */
    if (fabsf(f->cur_deg - f->last_sent_deg) < SERVO_WRITE_DEADBAND_DEG) {
        return;
    }

    esp_err_t err = servo_set_angle(f->channel, f->cur_deg);
    if (err == ESP_OK) {
        f->last_sent_deg = f->cur_deg;   /* 只有写成功才更新，失败下拍重试 */
    } else if ((++err_count % 200U) == 1U) {
        ESP_LOGE(TAG, "servo ch%d -> %.1f failed: %s (total=%lu)",
                 f->channel, (double)f->cur_deg, esp_err_to_name(err),
                 (unsigned long)err_count);
    }
}


/* ---------- 状态机各步 ---------- */

/**
 * @brief 梯形速度规划向 target_deg 走一步；返回 true 表示已到位。
 *        只更新 cur_deg/cur_speed 并写舵机，不改变状态。
 */
static bool finger_move_toward(finger_t *f, float dt)
{
    float err  = f->target_deg - f->cur_deg;
    float dist = fabsf(err);

    /* 到位判定 */
    if (dist < 0.3f) {
        f->cur_deg   = f->target_deg;
        f->cur_speed = 0.0f;
        apply_servo(f);
        return true;
    }

    /* 刹车距离：以当前速度减速到 0 需要走多远（V*V = 2aS） */
    float brake_dist = (f->cur_speed * f->cur_speed) / (2.0f * f->accel);

    if (dist <= brake_dist + 0.5f) {
        /* 减速段 */
        f->cur_speed -= f->accel * dt;
        if (f->cur_speed < f->min_speed) {
            f->cur_speed = f->min_speed;
        }
    } else {
        /* 加速段（到达上限后即匀速段） */
        f->cur_speed += f->accel * dt;
        if (f->cur_speed > f->max_speed) {
            f->cur_speed = f->max_speed;
        }
    }

    /* 按当前速度走一步，别冲过头 */
    float step = f->cur_speed * dt;
    if (step > dist) step = dist;

    f->cur_deg += (err > 0.0f) ? step : -step;
    apply_servo(f);
    return false;
}

/**
 * @brief 把 PID 输出叠加到当前角度并写舵机；顶到 0/180 时回滚本拍积分。
 */
static void finger_apply_pid_output(finger_t *f, pid_t *pid)
{
    float raw_angle = f->cur_deg + f->grasp_dir * pid->output;

    /* 角度已被该手指的机械限位钳制：回滚本拍积分，避免积分继续累积（anti-windup） */
    if (raw_angle < f->min_deg || raw_angle > f->max_deg) {
        pid->integral -= pid->integral_step;
        if (pid->integral > pid->MaxIntegral)  pid->integral = pid->MaxIntegral;
        if (pid->integral < -pid->MaxIntegral) pid->integral = -pid->MaxIntegral;
    }

    if (raw_angle < f->min_deg) raw_angle = f->min_deg;
    if (raw_angle > f->max_deg) raw_angle = f->max_deg;
    f->cur_deg = raw_angle;
    apply_servo(f);
}

/**
 * @brief APPROACHING：梯形速度规划向 target_deg 运动，每步先检测接触。
 *        接触即冻结当前位置并切换到 APPROACHED，不再继续运动。
 */
void finger_approach_step(finger_t *f, int idx, float dt)
{
    float force = finger_read_force(idx);
    f->cur_force = force;

    /* 接触物体：立即停止，保持当前位置，等待 catch 命令 */
    if (force > f->contact_threshold) {
        f->state      = FINGER_APPROACHED;
        f->contacted  = true;       /* 真正压到了物体 */
        f->target_deg = f->cur_deg;
        f->cur_speed  = 0.0f;
        apply_servo(f);
        ESP_LOGI(TAG, "finger %d contact detected @ %.1f deg, force %.1f",
                 idx + 1, (double)f->cur_deg, (double)force);
        return;
    }

    /* 只是走完行程，不代表压到物体 */
    if (finger_move_toward(f, dt)) {
        f->state     = FINGER_APPROACHED;
        f->contacted = false;
    }
}


/**
 * @brief CATCHING：PID 力控闭环。目标=target_force，反馈=当前压力，
 *        输出为角度增量，按 grasp_dir 叠加到当前角度。
 */
void finger_catch_step(finger_t *f, int idx, float dt)
{
    pid_t *pid = finger_pid_get(idx);
    if (pid == NULL) {
        return;                 /* 防御：idx 越界时不做任何力控 */
    }
    if (!finger_has_sensor(idx)) {
        /* 没有压力传感器就无法闭环，保持当前位置，退回空闲 */
        f->state = FINGER_STATE_IDLE;
        f->cur_speed = 0.0f;
        apply_servo(f);
        return;
    }

    /* 命令任务置位的复位标志，在控制任务上下文里消费，避免与 PID 并发读写 */
    if (f->pid_reset) {
        finger_pid_reset(pid);
        f->pid_reset = false;
    }

    float force = finger_read_force(idx);//得到压力值
    f->cur_force = force;

    /* 超时：长时间达不到目标力（空抓/物体太硬/已顶到机械限位）就放弃并保持 */
    if (++f->catch_ticks >= FINGER_CATCH_TIMEOUT_TICKS) {
        f->state       = FINGER_APPROACHED;
        f->cur_speed   = 0.0f;
        f->catch_ticks = 0;
        apply_servo(f);
        ESP_LOGW(TAG, "finger %d catch timeout: force %.1f/%.1f @ %.1f deg, hold",
                 idx + 1, (double)force, (double)f->target_force, (double)f->cur_deg);
        return;
    }

    pid->target   = f->target_force;
    pid->feedback = force;
    fPID(pid, dt);
    finger_apply_pid_output(f, pid);

    /* 力稳定判定：误差连续在容差内则抓取到位 */
    if (fabsf(pid->err) <= FINGER_CATCHED_FORCE_TOL) {
        if (++f->catch_stable >= FINGER_CATCHED_STABLE_TICKS) {
            f->state        = FINGER_CATCHED;
            f->catched_lost = 0;
            ESP_LOGI(TAG, "finger %d catched, force %.1f @ %.1f deg",
                     idx + 1, (double)force, (double)f->cur_deg);
        }
    } else {
        f->catch_stable = 0;
    }
}

/**
 * @brief CATCHED：继续保力。力偏离目标超过阈值并持续一定节拍，
 *        认为物体滑落，回到 CATCHING 重新稳定判定。
 */
void finger_hold_step(finger_t *f, int idx, float dt)
{
    pid_t *pid = finger_pid_get(idx);
    if (pid == NULL || !finger_has_sensor(idx)) {
        return;
    }

    float force = finger_read_force(idx);
    f->cur_force = force;

    pid->target   = f->target_force;
    pid->feedback = force;
    fPID(pid, dt);

    if (fabsf(pid->err) > FINGER_CATCHED_LOST_FORCE_TOL) {
        if (++f->catched_lost >= FINGER_CATCHED_LOST_TICKS) {
            f->catched_lost = 0;
            f->catch_stable = 0;
            f->catch_ticks  = 0;
            f->state        = FINGER_CATCHING;
            ESP_LOGW(TAG, "finger %d force lost (%.1f/%.1f), recatching",
                     idx + 1, (double)force, (double)f->target_force);
            return;
        }
    } else {
        f->catched_lost = 0;
    }

    finger_apply_pid_output(f, pid);
}

/**
 * @brief MOVING：纯梯形速度规划运动到 target_deg，不做接触检测，
 *        到位后回到 IDLE。
 */
void finger_move_step(finger_t *f, float dt)
{
    if (finger_move_toward(f, dt)) {
        f->state = FINGER_STATE_IDLE;
    }
}

